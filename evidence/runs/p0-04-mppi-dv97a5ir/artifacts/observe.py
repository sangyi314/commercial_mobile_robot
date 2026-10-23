from pathlib import Path
import argparse, json, math, time
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from action_msgs.msg import GoalStatusArray
from tf2_ros import Buffer, TransformListener, TransformException

parser = argparse.ArgumentParser()
parser.add_argument("--output", required=True)
parser.add_argument("--seconds", type=float, default=240.0)
args = parser.parse_args()
if not math.isfinite(args.seconds) or not 0 < args.seconds <= 600:
    raise SystemExit("Invalid observation duration")
target = Path(args.output)
stream = target.open("x", encoding="utf-8")
rclpy.init()
node = rclpy.create_node(
    "p03_observer",
    parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
)
buffer = Buffer(node=node)
listener = TransformListener(buffer, node)
subscriptions = []
clock_previous = None
clock_emit_wall = 0.0

def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

def clean(values):
    return [float(v) if math.isfinite(float(v)) else None for v in values]

def emit(kind, data):
    stream.write(json.dumps({
        "kind": kind, "receive_monotonic_s": time.monotonic(),
        "ros_now_s": node.get_clock().now().nanoseconds * 1e-9, "data": data,
    }, allow_nan=False) + "\n")
    stream.flush()

def on_clock(msg):
    global clock_previous, clock_emit_wall
    value = seconds(msg.clock)
    now = time.monotonic()
    if clock_previous is not None and value < clock_previous:
        emit("clock_jump", {"before": clock_previous, "after": value})
    clock_previous = value
    if now - clock_emit_wall >= 0.2:
        emit("clock", {"sim_s": value})
        clock_emit_wall = now

def on_odom(msg):
    p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
    emit("odom", {
        "stamp_s": seconds(msg.header.stamp),
        "frame_id": msg.header.frame_id, "child_frame_id": msg.child_frame_id,
        "position": clean([p.x, p.y, p.z]),
        "quaternion_xyzw": clean([q.x, q.y, q.z, q.w]),
        "linear": clean([v.linear.x, v.linear.y, v.linear.z]),
        "angular": clean([v.angular.x, v.angular.y, v.angular.z]),
    })

def on_cmd(topic, msg):
    emit(topic, {
        "linear": clean([msg.linear.x, msg.linear.y, msg.linear.z]),
        "angular": clean([msg.angular.x, msg.angular.y, msg.angular.z]),
        "message_type": "geometry_msgs/msg/Twist",
    })

def on_status(msg):
    emit("action_status", [{
        "goal_uuid": bytes(s.goal_info.goal_id.uuid).hex(), "status": int(s.status),
        "accepted_stamp_s": seconds(s.goal_info.stamp),
    } for s in msg.status_list])

subscriptions.append(node.create_subscription(
    Clock, "/clock", on_clock, qos_profile_sensor_data))
subscriptions.append(node.create_subscription(
    Odometry, "/odom", on_odom, qos_profile_sensor_data))
for topic in ["/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel"]:
    subscriptions.append(node.create_subscription(
        Twist, topic, lambda msg, name=topic: on_cmd(name, msg), qos_profile_sensor_data))
status_qos = QoSProfile(
    depth=10, reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)
subscriptions.append(node.create_subscription(
    GoalStatusArray, "/navigate_to_pose/_action/status", on_status, status_qos))
end = time.monotonic() + args.seconds
last_tf = 0.0
try:
    emit("observer_started", {"seconds": args.seconds})
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if now - last_tf < 0.2:
            continue
        last_tf = now
        try:
            t = buffer.lookup_transform("map", "base_footprint", Time())
            p, q = t.transform.translation, t.transform.rotation
            emit("map_base", {
                "stamp_s": seconds(t.header.stamp), "frame_id": t.header.frame_id,
                "child_frame_id": t.child_frame_id,
                "position": clean([p.x, p.y, p.z]),
                "quaternion_xyzw": clean([q.x, q.y, q.z, q.w]),
            })
        except TransformException as exc:
            emit("tf_unavailable", {"message": str(exc)})
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    stream.flush()
    stream.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
