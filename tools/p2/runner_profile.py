"""Narrow P2 opt-in delta applied only after the original Runner validates P1 inputs."""
from pathlib import Path
import json
from hashlib import sha256
import math
import re
import subprocess
import time
import xml.etree.ElementTree as ET

import yaml

from tools.runner.runner_core import atomic_write_new, digest, strict_json
from tools.p2.provenance import source_fingerprint


def verified_file(ref):
    path = Path(ref["path"])
    if not path.is_absolute() or not path.is_file() or digest(path) != ref["sha256"]:
        raise ValueError(f"P2 input identity mismatch: {path}")
    return path


def apply(profile, profile_path, spec_path, identity):
    if profile.get("profile_version") != 4 or profile["delivery_v1"]["case"] != "G01D":
        raise ValueError("P2 motion and hold experiments require the delegated Mission/Gate profile")
    spec = strict_json(Path(spec_path).read_text())
    fields = {"schema", "run_id", "p1_profile_sha256", "mode", "hold_sim_seconds", "maneuver",
              "model", "qualification"}
    if "fault_test" in spec:
        fields.add("fault_test")
        if spec["fault_test"] not in ("P201.TFOwnership", "P201.FaultTime") or spec["mode"] != "shadow":
            raise ValueError("fault test requires the explicit shadow profile")
    if set(spec) != fields or spec["schema"] != "cmr.p2/1.1" or spec["run_id"] != identity:
        raise ValueError("P2 run profile schema or identity mismatch")
    if not re.fullmatch(r"[a-z][a-z0-9_]{1,70}", identity):
        raise ValueError("P2 run identifier invalid for Gate")
    if spec["p1_profile_sha256"] != digest(profile_path):
        raise ValueError("P2 profile must bind the exact reviewed P1 parent")
    mode = spec["mode"]
    if mode not in ("calibration", "shadow", "replacement"):
        raise ValueError("P2 unknown estimation mode")
    hold = spec["hold_sim_seconds"]
    if type(hold) is not int or hold not in (0, 60, 600):
        raise ValueError("P2 unexpected hold duration")
    if spec["maneuver"] not in ("delivery", "stationary", "straight", "rotate_left", "rotate_right"):
        raise ValueError("P2 unknown maneuver")
    if mode == "calibration":
        if spec["model"] is not None:
            raise ValueError("calibration only records the original sensor path")
    else:
        verified_file(spec["model"])
    if mode == "replacement":
        qualification = strict_json(verified_file(spec["qualification"]).read_text())
        head = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
        if (qualification.get("code_sha256") != source_fingerprint() or
                qualification.get("model") != spec["model"] or
                qualification["head"] != head or qualification["schema"] != "cmr.p2/1.1" or
                verified_file(qualification["shadow"]).name != "shadow-result.json" or
                verified_file(qualification["p1_regression"]).name != "p1-regression-result.json"):
            raise ValueError("P2 replacement requires identified shadow and P1 regression evidence")
        for field in ("shadow", "p1_regression"):
            report = strict_json(verified_file(qualification[field]).read_text())
            if (report.get("status") != "PASS" or report.get("head") != qualification["head"] or
                    report.get("code_sha256") != qualification["code_sha256"]):
                raise ValueError(f"P2 replacement prerequisite has not passed: {field}")
        original = yaml.safe_load(Path(profile["execution_v1"]["bridge"]).read_text())
        profile["p2_bridge"] = replacement_bridge(original)
    elif spec["qualification"] is not None:
        raise ValueError("qualification applies only to replacement")
    namespace = f"/cmr_p2/{identity}"
    if spec.get("fault_test"):
        profile["recording"]["topics"] += [
            namespace + "/fault_input/" + name
            for name in ("scan", "odom", "imu", "clock", "tf", "tf_static")]
    profile["recording"]["topics"] += ["/imu", "/joint_states"]
    if mode != "calibration":
        profile["recording"]["topics"] += [
            f"{namespace}/{suffix}" for suffix in ("context", "wheel_odom", "imu_validated",
                                                   "odometry/filtered", "filter_output_internal")]
    if hold:
        profile["budgets"]["work"] += 2 * hold
    profile["p2_v1"] = spec
    return profile


def replacement_bridge(bridge):
    def is_old_tf(row):
        return row.get("ros_topic_name") == "/tf" and row.get("gz_topic_name") == "/tf" and (
            row.get("direction") == "GZ_TO_ROS" and row.get("ros_type_name") == "tf2_msgs/msg/TFMessage")

    result = [row for row in bridge if not is_old_tf(row)]
    if len(result) != len(bridge) - 1 or any(
            row.get("ros_topic_name") == "/tf" or row.get("gz_topic_name") == "/tf"
            for row in result):
        raise ValueError("P2 replacement requires exactly one removable old dynamic TF route")
    return result


def mount_imu(robot_bytes, urdf_bytes):
    """Put the existing sensor on the physical base at the advertised URDF pose."""
    robot = ET.fromstring(robot_bytes)
    model = robot.find("model")
    detached = model.find("link[@name='imu_link']")
    base = model.find("link[@name='base_link']")
    sensor = detached.find("sensor[@name='tb3_imu']")
    urdf = ET.fromstring(urdf_bytes)
    joint = urdf.find("joint[@name='imu_joint']")
    origin = joint.find("origin")
    if (sensor is None or len(detached) != 1 or sensor.find("pose") is not None or
            any(j.findtext("child") == "imu_link" or j.findtext("parent") == "imu_link"
                for j in model.findall("joint")) or joint.get("type") != "fixed" or
            joint.find("parent").get("link") != "base_link" or
            joint.find("child").get("link") != "imu_link" or
            sensor.get("type") != "imu" or sensor.findtext("gz_frame_id") != "imu_link" or
            len(list(model.iterfind(".//sensor[@type='imu']"))) != 1):
        raise ValueError("P2 IMU source model differs from the reviewed detached-link layout")
    mount = origin.get("xyz") + " " + origin.get("rpy")
    if len(mount.split()) != 6 or not all(math.isfinite(float(x)) for x in mount.split()):
        raise ValueError("P2 IMU URDF extrinsics malformed")
    ET.SubElement(sensor, "pose").text = mount
    detached.remove(sensor)
    model.remove(detached)
    base.append(sensor)
    return ET.tostring(robot, encoding="utf-8", xml_declaration=True), ET.tostring(sensor)


def robot_snapshot(path):
    urdf = Path("/opt/ros/jazzy/share/nav2_minimal_tb3_sim/urdf/turtlebot3_waffle.urdf")
    robot, sensor = mount_imu(Path(path).read_bytes(), urdf.read_bytes())
    tree = ET.fromstring(robot)
    frequency = tree.find(".//plugin[@name='gz::sim::systems::DiffDrive']/odom_publish_frequency")
    if frequency is None or frequency.text != "30":
        raise ValueError("unexpected original DiffDrive publishing rate")
    # The 20 ms exact-time TF wait cannot cover a 36 ms odom publication gap.
    # Increase only the P2 derived source rate; retain wheel feedback integration.
    frequency.text = "100"
    return ET.tostring(tree, encoding="utf-8", xml_declaration=True), sensor + b"|wheel-odom-hz=100"


def snapshot(run, profile):
    spec = profile["p2_v1"]
    robot, sensor = robot_snapshot(profile["files"]["robot"]["path"])
    atomic_write_new(run / "artifacts/robot.sdf", robot)
    sensor_hash = sha256(sensor).hexdigest()
    atomic_write_new(run / "artifacts/p2-imu-source.json", json.dumps({
        "sensor_sha256": sensor_hash,
        "original_robot_sha256": profile["files"]["robot"]["sha256"],
        "mount_link": "base_link", "published_frame": "imu_link",
        "wheel_odom_publish_frequency_hz": 100, "original_frequency_hz": 30,
    }).encode())
    if spec["model"]:
        model = verified_file(spec["model"]).read_bytes()
        calibration = strict_json(model.decode())
        if calibration.get("sensor_sha256") != sensor_hash or calibration.get("qualification") != "PASS":
            raise ValueError("P2 covariance model does not qualify this IMU source")
        atomic_write_new(run / "artifacts/p2-model.json", model)
    if spec["mode"] == "replacement":
        bridge = run / "artifacts/execution_bridge.yaml"
        original = Path(profile["execution_v1"]["bridge"])
        if replacement_bridge(yaml.safe_load(original.read_text())) != profile["p2_bridge"]:
            raise ValueError("P2 bridge changed after validation")
        atomic_write_new(run / "artifacts/p2-old-execution-bridge.yaml", original.read_bytes())
        atomic_write_new(bridge, yaml.safe_dump(profile["p2_bridge"]).encode())
        atomic_write_new(run / "artifacts/p2-replacement-evidence.json",
                         verified_file(spec["qualification"]).read_bytes())


def configure_motion(run, profile, catalog, nav_config):
    maneuver = profile["p2_v1"]["maneuver"]
    if maneuver == "delivery":
        return
    poses = {entry["station_id"]: entry["pose"] for entry in catalog["stations"]}
    target, home = poses["delivery"], poses["home"]
    target["position"] = dict(home["position"])
    yaw = 0.0
    if maneuver == "straight":
        target["position"]["x"] += 1.0
    elif maneuver != "stationary":
        yaw = math.pi / 2 if maneuver == "rotate_left" else -math.pi / 2
    target["orientation"] = dict(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    checker = nav_config["controller_server"]["ros__parameters"]["general_goal_checker"]
    # AMCL heading error and Nav2 goal tolerance both contribute to the
    # independent endpoint error. Leave margin below the 0.10 rad test gate.
    yaw_tolerance = 0.035 if maneuver in ("rotate_left", "rotate_right") else 0.06
    checker.update(xy_goal_tolerance=0.05, yaw_goal_tolerance=yaw_tolerance)
    atomic_write_new(run / "artifacts/p2-maneuver.json",
                     json.dumps({"maneuver": maneuver, "target": target, "start": home}).encode())


def start_estimation(supervisor, run, profile, env, root):
    return supervisor.spawn("p2-estimation", [
        "ros2", "launch", "--noninteractive", "cmr_tracking_ros", "p2_estimation.launch.py",
        "run_id:=" + run.name, "mode:=" + profile["p2_v1"]["mode"], "use_sim_time:=true",
        "model_file:=" + str(run / "artifacts/p2-model.json"),
        "map_version:=" + profile["files"]["map_image"]["sha256"],
        "trace_path:=" + str(run / "runtime/p2-input-time.csv"),
        "runner_artifacts:=" + str(run / "artifacts"),
    ] + (["fault_test:=" + profile["p2_v1"]["fault_test"]] if "fault_test" in profile["p2_v1"] else []), env, root)


def observe_hold(seconds, run, link, gaz, budget):
    """Allow physics to advance with no Mission submit and an untouched Gate."""
    if not seconds:
        return
    def sim_now():
        sec, nsec = (gaz.latest["stats"]["protobuf"]["sim_time"].get(k, 0)
                     for k in ("sec", "nsec"))
        return int(sec) * 1_000_000_000 + int(nsec)

    before = sim_now()
    wall = time.monotonic()
    atomic_write_new(run / "runtime/p2-hold-start.json",
                     json.dumps({"start_ns": before, "steady": wall}).encode())
    link.rpc("mark", {"name": "P2_HOLD_START"}, budget.child(3))
    while sim_now() - before < seconds * 1_000_000_000:
        budget.require()
        link.pump()
    link.rpc("mark", {"name": "P2_HOLD_END"}, budget.child(3))
    atomic_write_new(run / "runtime/p2-hold.json", json.dumps({
        "start_ns": before, "end_ns": sim_now(), "steady_seconds": time.monotonic() - wall,
        "mission_submit_during_hold": False,
    }).encode())


def start_fault_proxy(supervisor, run, profile, env, root):
    import sys
    return supervisor.spawn("p2-fault-proxy", [
        sys.executable, "-B", str(root / "tools/p2/fault_proxy.py"),
        "--run", str(run), "--case", profile["p2_v1"]["fault_test"],
    ], env, root)
