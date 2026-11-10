"""Exercise the installed P2 nodes with synthetic sensors in an isolated ROS domain."""
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from uuid import uuid4

import pytest

pytestmark = pytest.mark.ros_contract
NS = 1_000_000_000


class EncoderImuFixture:
    def __init__(self, root):
        import rclpy
        from cmr_interfaces.msg import P2Context
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, LaserScan
        from tf2_msgs.msg import TFMessage

        self.root = root
        self.run = "test_" + uuid4().hex[:12]
        self.namespace = f"/cmr_p2/{self.run}"
        self.rclpy = rclpy
        self.Clock, self.Imu, self.Odom, self.Scan, self.Transform, self.Tf = (
            Clock, Imu, Odometry, LaserScan, TransformStamped, TFMessage)
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = rclpy.create_node("p201_fixture", context=self.context)
        time.sleep(0.25)
        other = [item for item in self.node.get_node_names_and_namespaces() if item != ("p201_fixture", "/")]
        if other:
            self.node.destroy_node()
            rclpy.shutdown(context=self.context)
            raise RuntimeError(f"isolated ROS domain already in use: {other}")
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.publishers = {
            key: self.node.create_publisher(typ, topic, 5)
            for key, typ, topic in (
                ("clock", Clock, "/clock"), ("odom", Odometry, "/odom"),
                ("imu", Imu, "/imu"), ("scan", LaserScan, "/scan"), ("tf", TFMessage, "/tf"))
        }
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static = self.node.create_publisher(TFMessage, "/tf_static", durable)
        self.states, self.outputs = [], []
        self.node.create_subscription(P2Context, self.namespace + "/context",
                                      lambda v: self.states.append((time.monotonic_ns(), v)), durable)
        self.node.create_subscription(Odometry, self.namespace + "/odometry/filtered",
                                      lambda v: self.outputs.append(v.header.stamp), 10)
        self.trace = root / "inputs.csv"
        model = root / "fixture-model.json"
        model.write_text(json.dumps({"schema": "cmr.p2/1.1",
                                     "wheel_vx_variance": 0.001, "imu_wz_variance": 0.001,
                                     "imu_wz_bias": 0.0, "source_level": "E1_ORACLE"}))
        self.log = (root / "launch.log").open("x")
        self.proc = subprocess.Popen(
            ["ros2", "launch", "--noninteractive", "cmr_tracking_ros", "p2_estimation.launch.py",
             "run_id:=" + self.run, "mode:=shadow", "use_sim_time:=true",
             "authority:=OFFLINE_LOCAL", "source_node:=p201_fixture",
             "model_file:=" + str(model), "trace_path:=" + str(self.trace)],
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        self.now = 10 * NS
        self.last_scan = 0
        self.last = {}
        self.static.publish(self.Tf(transforms=[
            self.transform("base_footprint", "base_link"),
            self.transform("base_link", "imu_link"), self.transform("base_link", "base_scan")]))

    @staticmethod
    def stamp(stamp, value):
        stamp.sec, stamp.nanosec = divmod(value, NS)

    def transform(self, parent, child):
        tf = self.Transform()
        self.stamp(tf.header.stamp, self.now)
        tf.header.frame_id, tf.child_frame_id = parent, child
        tf.transform.rotation.w = 1.0
        return tf

    def send(self, key, stamp):
        if key == "clock":
            msg = self.Clock()
            self.stamp(msg.clock, stamp)
        elif key == "tf":
            msg = self.Tf(transforms=[self.transform("odom", "base_footprint")])
        else:
            typ, frame = {"odom": (self.Odom, "odom"), "imu": (self.Imu, "imu_link"),
                          "scan": (self.Scan, "base_scan")}[key]
            msg = typ()
            msg.header.frame_id = frame
            self.stamp(msg.header.stamp, stamp)
            if key == "odom":
                msg.child_frame_id = "base_footprint"
                msg.pose.pose.orientation.w = 1.0
            if key == "scan":
                msg.angle_increment = 0.01
                msg.angle_max = 0.03
                msg.range_min, msg.range_max = 0.05, 5.0
                msg.ranges = [1.0] * 4
        self.publishers[key].publish(msg)
        self.last[key] = stamp

    def drive(self, seconds, *, omit=(), predicate=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.proc.poll() is not None:
                raise RuntimeError("P2 launched process exited")
            tick = time.monotonic()
            if "clock" not in omit:
                self.now += 10_000_000
            for key in ("clock", "tf", "odom", "imu", "scan"):
                if key in omit or (key == "scan" and self.now - self.last_scan < 200_000_000):
                    continue
                self.send(key, self.now)
                if key == "scan":
                    self.last_scan = self.now
            while time.monotonic() - tick < 0.01:
                self.executor.spin_once(timeout_sec=0.002)
            if predicate and predicate():
                return True
        return not predicate

    def healthy(self):
        return self.states and self.states[-1][1].health == 0

    def invalid(self, reason):
        return self.states and self.states[-1][1].health == 2 and self.states[-1][1].reason == reason

    def trace_has(self, key, stamp, reason):
        if not self.trace.exists():
            return False
        with self.trace.open() as stream:
            return any(row.get("topic") == "/" + key and row.get("stamp_ns") == str(stamp)
                       and row.get("reason") == reason for row in csv.DictReader(stream))

    def close(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGTERM)
                self.proc.wait(timeout=5)
        self.log.close()
        with (self.root / "contexts.jsonl").open("x") as out:
            for steady, v in self.states:
                print(json.dumps({"steady_ns": steady, "clock_epoch": v.clock_epoch,
                                  "odom_epoch": v.odom_epoch, "seq": v.sequence,
                                  "health": v.health, "reason": v.reason}), file=out)
        self.executor.shutdown()
        self.node.destroy_node()
        self.rclpy.shutdown(context=self.context)


@pytest.fixture
def sensor_fixture(request, p2_manifest):
    domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if domain < 73 or domain > 100 or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST":
        pytest.fail("requires an unused test ROS domain 73–100 with LOCALHOST discovery")
    mark = request.node.get_closest_marker("p2_case")
    root = Path(p2_manifest["evidence"]) / "tests" / mark.args[0]
    root = root / ("ros-" + uuid4().hex[:10])
    root.mkdir(parents=True, exist_ok=False)
    source = EncoderImuFixture(root)
    try:
        yield source
    finally:
        source.close()


@pytest.mark.p2_case("P201.TFOwnership")
@pytest.mark.parametrize("repeat", range(3))
def test_tf_single_owner_and_duplicate(sensor_fixture, repeat):
    source = sensor_fixture
    assert source.drive(7, predicate=source.healthy)
    rogue = source.node.create_publisher(source.Tf, "/tf", 5)
    try:
        for _ in range(5):
            rogue.publish(source.Tf(transforms=[source.transform("odom", "base_footprint")]))
            source.drive(0.05)
        assert source.invalid("TF_UNAVAILABLE")
        assert "Duplicate TF owner" in (source.root / "launch.log").read_text()
    finally:
        source.node.destroy_publisher(rogue)


@pytest.mark.p2_case("P201.HealthRecovery")
def test_health_recovery(sensor_fixture):
    source = sensor_fixture
    assert source.drive(7, predicate=source.healthy)
    for key in ("imu", "odom"):
        for _ in range(5):
            assert source.drive(1, omit=(key,), predicate=lambda: source.invalid("INPUT_STALE"))
            start = time.monotonic_ns()
            source.drive(1.8)
            assert not source.healthy()
            assert source.drive(3, predicate=source.healthy)
            assert time.monotonic_ns() - start >= 2 * NS


@pytest.mark.p2_case("P201.ResetRace")
@pytest.mark.parametrize("repeat", range(3))
def test_epoch_reset_blocks_old_source(sensor_fixture, repeat):
    source = sensor_fixture
    assert source.drive(7, predicate=source.healthy)
    old = source.states[-1][1]
    old_stamp = source.now
    source.now -= 3 * NS
    source.send("clock", source.now)
    assert source.drive(2, predicate=lambda: source.invalid("CLOCK_RESET"))
    after = source.states[-1][1]
    assert old.clock_epoch != after.clock_epoch and old.odom_epoch != after.odom_epoch
    source.drive(0.2)
    previous = len(source.outputs)
    for step in range(1000):
        source.send("imu", old_stamp + step * 1000)
        source.send("odom", old_stamp + step * 1000)
        source.now += 1_000_000
        source.send("clock", source.now)
        source.executor.spin_once(timeout_sec=0)
    source.drive(2.3)
    assert source.invalid("CLOCK_RESET")
    assert len(source.outputs) == previous


@pytest.mark.p2_case("P201.FaultTime")
@pytest.mark.parametrize("repeat", range(3))
def test_inputs_reject_time_and_tf_faults(sensor_fixture, repeat):
    source = sensor_fixture
    assert source.drive(7, predicate=source.healthy)

    def sample(offset_ns, reason):
        stamp = source.now + offset_ns
        source.send("imu", stamp)
        # Keep /clock fixed while this exact boundary sample is received.
        # DDS delivery plus the next clock tick would turn 100 ms into 110 ms.
        deadline = time.monotonic() + 0.24
        while time.monotonic() < deadline and not source.trace_has("imu", stamp, reason):
            source.executor.spin_once(timeout_sec=0.002)
        assert source.trace_has("imu", stamp, reason)

    for delay, expected in ((50, "NONE"), (100, "NONE"), (200, "INPUT_STALE")):
        source.drive(0.36, omit=("imu",))
        sample(-delay * 1_000_000, expected)
        source.drive(0.3)
    sample(100_000_000, "FUTURE_STAMP")
    # Only accepted data advances the timestamp watermark. A rejected future
    # sample is deliberately not eligible to be the duplicate fixture.
    source.drive(0.3)
    duplicate = source.last["imu"]
    deadline = time.monotonic() + 0.2
    while time.monotonic() < deadline and not source.trace_has("imu", duplicate, "NONE"):
        source.executor.spin_once(timeout_sec=0.002)
    assert source.trace_has("imu", duplicate, "NONE")
    source.drive(0.04, omit=("imu",))
    source.send("imu", duplicate)
    assert source.drive(0.2, omit=("imu",),
                        predicate=lambda: source.trace_has("imu", duplicate, "DUPLICATE_STAMP"))
    sample(duplicate - 1_000_000 - source.now, "OUT_OF_ORDER")
    assert source.drive(5, predicate=source.healthy)
    assert source.drive(1, omit=("tf",), predicate=lambda: source.invalid("TF_UNAVAILABLE"))
    assert source.drive(5, predicate=source.healthy)
    assert source.drive(1, omit=("clock", "scan", "tf", "odom", "imu"),
                        predicate=lambda: source.invalid("INPUT_STALE"))
    assert source.drive(5, predicate=source.healthy)
    source.now -= NS
    source.send("clock", source.now)
    assert source.drive(2, predicate=lambda: source.invalid("CLOCK_RESET"))
