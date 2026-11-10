#!/usr/bin/env python3
"""Read an existing P2 EKF's parameter services and graph with no set operations."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.parameter import parameter_value_to_python

EXPECTED = {
    "frequency": 30.0, "sensor_timeout": 0.1,
    "world_frame": "odom", "odom_frame": "odom", "base_link_frame": "base_footprint",
    "two_d_mode": True, "reset_on_time_jump": True, "use_sim_time": True,
    "odom0": "wheel_odom", "imu0": "imu_validated",
    "odom0_config": [i == 6 for i in range(15)],
    "imu0_config": [i == 11 for i in range(15)],
}


def capture(namespace, mode):
    target = namespace + "/ekf_filter_node"
    expected = {**EXPECTED, "publish_tf": mode == "replacement"}
    rclpy.init(args=[])
    node = rclpy.create_node("p201_parameter_capture")
    try:
        def query(kind, endpoint, request):
            client = node.create_client(kind, target + "/" + endpoint)
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError("parameter service not discoverable: " + endpoint)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            if not future.done() or future.result() is None:
                raise RuntimeError("parameter request failed: " + endpoint)
            return future.result()

        declared = sorted(query(ListParameters, "list_parameters",
                                ListParameters.Request(depth=0)).result.names)
        if not set(expected).issubset(declared):
            raise RuntimeError("incomplete declaration set")
        requested = sorted(expected)
        replies = query(GetParameters, "get_parameters", GetParameters.Request(names=requested)).values
        if len(replies) != len(requested):
            raise RuntimeError("incomplete parameter response")
        params = {name: parameter_value_to_python(value) for name, value in zip(requested, replies)}
        bad = {name: dict(expected=want, actual=params.get(name)) for name, want in expected.items()
               if params.get(name) != want}
        subs = dict(node.get_subscriber_names_and_types_by_node("ekf_filter_node", namespace))
        allowed = {namespace + "/" + s for s in ("filter_clock", "wheel_odom", "imu_validated", "set_pose")}
        allowed |= {"/parameter_events", "/tf_static", "/tf"}
        if not {namespace + "/" + s for s in ("filter_clock", "wheel_odom", "imu_validated")} <= subs.keys():
            bad["subscriptions"] = "expected private sensor and clock subscriptions missing"
        if subs.keys() - allowed:
            bad["subscriptions"] = sorted(subs.keys() - allowed)
        return {"read_monotonic_ns": time.monotonic_ns(), "target": target,
                "mode": mode, "parameters": params, "declared_parameter_names": declared,
                "subscriptions": subs, "mismatches": bad,
                "status": "FAIL" if bad else "PASS"}
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    run = args.run.resolve(strict=True)
    profile = json.loads((run / "artifacts/profile.json").read_text())["p2_v1"]
    if profile["run_id"] != run.name or profile["mode"] not in ("shadow", "replacement"):
        parser.error("expected this trial's shadow or replacement artifact")
    if args.output.exists():
        parser.error("output exists")
    result = capture("/cmr_p2/" + run.name, profile["mode"])
    with args.output.open("x") as f:
        json.dump(result, f, indent=2, allow_nan=False)
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
