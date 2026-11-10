"""Opt-in estimation attachment. Owns only the P2 health adapter and EKF."""
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import re
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def verify_replacement(artifacts, namespace):
    if not os.environ.get("CMR_OWNERSHIP_TOKEN") or not artifacts.is_dir():
        raise ValueError("replacement must be started by the project Runner")
    bridge = yaml.safe_load((artifacts / "execution_bridge.yaml").read_text())
    (artifacts / "p2-replacement-evidence.json").read_text()
    if any(row.get("ros_topic_name") == "/tf" for row in bridge):
        raise ValueError("old ROS TF route present in replacement bridge")

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    context = Context()
    rclpy.init(args=[], context=context)
    probe = rclpy.create_node("p2_replacement_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)
    from tf2_msgs.msg import TFMessage
    seen_edges = set()
    def observe(msg):
        for transform in msg.transforms:
            seen_edges.add((transform.header.frame_id, transform.child_frame_id))
    subscription = probe.create_subscription(TFMessage, "/tf", observe, 100)
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            if ("odom", "base_footprint") in seen_edges:
                raise ValueError("old odom-to-base TF still being published before EKF")
        publishers = probe.get_publishers_info_by_topic("/tf")
        if any(p.node_name != "robot_state_publisher" or p.node_namespace != "/"
               for p in publishers):
            raise ValueError("unexpected dynamic TF publisher before EKF")
        odom = probe.get_publishers_info_by_topic("/odom")
        if not (len(odom) == 1 and odom[0].node_name == "execution_bridge" and
                odom[0].node_namespace == namespace):
            raise ValueError("expected raw odom bridge was not discoverable")
        with (artifacts.parent / "runtime/p2-replacement-preflight.json").open("x") as log:
            json.dump({"raw_odom_owner": namespace + "/execution_bridge",
                       "raw_odom_gid": list(odom[0].endpoint_gid),
                       "observed_edges_before_ekf": sorted(seen_edges),
                       "tf_publishers_before_ekf": [
                           {"name": p.node_name, "namespace": p.node_namespace,
                            "gid": list(p.endpoint_gid)} for p in publishers],
                       "forbidden_edge_observed": False, "observation_seconds": 5}, log)
    finally:
        executor.shutdown()
        probe.destroy_node()
        rclpy.shutdown(context=context)


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    run = value("run_id")
    mode = value("mode")
    authority = value("authority")
    if run == "RUN_ID" or not re.fullmatch(r"[A-Za-z0-9_]{1,71}", run):
        raise ValueError("run_id must identify an existing run or isolated replay")
    if mode not in ("shadow", "replacement") or value("use_sim_time") != "true":
        raise ValueError("invalid P2 estimation mode or simulation time")
    if authority not in ("ATTACHED_GATE", "OFFLINE_LOCAL"):
        raise ValueError("unknown epoch authority")
    fault = value("fault_test")
    if fault and (fault not in ("P201.TFOwnership", "P201.FaultTime") or mode != "shadow" or
                  authority != "ATTACHED_GATE" or not os.environ.get("CMR_OWNERSHIP_TOKEN")):
        raise ValueError("fault instrumentation requires a Runner-owned shadow simulation")
    source = value("source_node")
    source_namespace = value("source_namespace")
    if authority == "ATTACHED_GATE":
        source = "execution_bridge"
        source_namespace = f"/cmr_execution/{run}"
        if fault:
            source, source_namespace = "p2_fault_proxy", f"/cmr_p2/{run}"
    elif not source or not source_namespace.startswith("/"):
        raise ValueError("explicit offline publisher identity is required")
    if mode == "replacement":
        if authority != "ATTACHED_GATE" or not value("runner_artifacts"):
            raise ValueError("replacement requires a qualified Runner attachment")
        verify_replacement(Path(value("runner_artifacts")), source_namespace)
    model_file = Path(value("model_file"))
    model = json.loads(model_file.read_text())
    keys = ("wheel_vx_variance", "imu_wz_variance", "imu_wz_bias")
    if model["schema"] != "cmr.p2/1.1" or any(
            type(model[key]) not in (int, float) or not math.isfinite(model[key]) for key in keys
    ) or min(model[keys[0]], model[keys[1]]) <= 0:
        raise ValueError("invalid explicit sensor covariance model")
    if authority == "ATTACHED_GATE" and model.get("source_level") != "E2_SIM_SENSOR":
        raise ValueError("attached motion requires an E2 sensor model")
    ekf_config = Path(get_package_share_directory("cmr_tracking_ros")) / "config/ekf.yaml"
    config_hash = sha256(ekf_config.read_bytes() + b"\0" + mode.encode() + b"\0" + model_file.read_bytes()).hexdigest()
    namespace = f"/cmr_p2/{run}"
    trace = value("trace_path")
    if trace and (not Path(trace).is_absolute() or Path(trace).exists()):
        raise ValueError("trace_path must be an unused absolute path")
    health = Node(
        package="cmr_tracking_ros", executable="sensor_health_node", namespace=namespace,
        parameters=[{**{key: float(model[key]) for key in keys},
                     "fault_test": fault,
                     "source_level": model.get("source_level", "E1_ORACLE"),
                     "execution_mode": "SIM_SENSOR" if authority == "ATTACHED_GATE" else "NUMERIC_TEST",
                     "source_id": model.get("sensor_sha256", "p201-numeric-fixture"),
                     "model_version": model.get("model_version", "numeric-fixture-v1"),
                     "map_version": value("map_version"),
                     "run_id": run, "authority": authority, "source_node": source,
                     "source_namespace": source_namespace, "config_hash": config_hash,
                     "trace_path": trace, "use_sim_time": True}],
        remappings=[(topic, namespace + "/fault_input" + topic)
                    for topic in ("/clock", "/scan", "/odom", "/imu", "/tf", "/tf_static")] if fault else [],
        output="screen",
    )
    ekf = Node(
        package="robot_localization", executable="ekf_node", name="ekf_filter_node",
        namespace=namespace, parameters=[str(ekf_config), {"publish_tf": mode == "replacement"}],
        remappings=[("/clock", f"{namespace}/filter_clock"),
                    ("odometry/filtered", "filter_output_internal")],
        output="screen",
    )
    return [
        RegisterEventHandler(OnProcessExit(target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason="P2 component exited"))]))
        for node in (health, ekf)
    ] + [health, ekf]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("run_id"),
        DeclareLaunchArgument("fault_test", default_value=""),
        DeclareLaunchArgument("map_version", default_value="UNAVAILABLE"),
        DeclareLaunchArgument("mode", default_value="shadow"),
        DeclareLaunchArgument("authority", default_value="ATTACHED_GATE"),
        DeclareLaunchArgument("use_sim_time"),
        DeclareLaunchArgument("source_node", default_value=""),
        DeclareLaunchArgument("source_namespace", default_value="/"),
        DeclareLaunchArgument("trace_path", default_value=""),
        DeclareLaunchArgument("runner_artifacts", default_value=""),
        DeclareLaunchArgument("model_file"),
        OpaqueFunction(function=setup),
    ])
