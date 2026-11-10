#!/usr/bin/env python3
"""Evaluate sensor calibration and estimation against Gazebo's independent pose capture."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from audit_bag import NS, digest, inspect_files, open_reader, stamp_ns


def pose(msg):
    p, q = msg.position, msg.orientation
    return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def reference(run):
    source = run / "runtime/p0-08-gazebo.jsonl"
    times, values = [], []
    with source.open() as stream:
        for entry in map(json.loads, stream):
            if entry["kind"] != "pose" or not entry["protobuf"]["pose"]:
                continue
            data = entry["protobuf"]
            if entry["run_id"] != run.name or entry["clock_epoch"] != f"{run.name}:sim0":
                raise ValueError("GT run or time epoch mismatch")
            stamp = data["header"]["stamp"]
            sample = int(stamp.get("sec", 0)) * NS + int(stamp.get("nsec", 0))
            if times and sample <= times[-1]:
                continue
            robot = data["pose"][0]
            if len(data["pose"]) != 1 or robot["name"] != "turtlebot3_waffle":
                raise ValueError("GT robot identity mismatch")
            q = {axis: robot["orientation"].get(axis, 0) for axis in "wxyz"}
            yaw = math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]), 1 - 2 * (q["y"]**2 + q["z"]**2))
            times.append(sample)
            values.append([robot["position"].get("x", 0), robot["position"].get("y", 0), yaw])
    times, values = np.array(times, dtype=np.int64), np.array(values)
    if len(times) < 2 or not np.isfinite(values).all():
        raise ValueError("no independent trajectory")
    values[:, 2] = np.unwrap(values[:, 2])
    return times, values


def signals(run):
    prefix = f"/cmr_p2/{run.name}"
    topics = ("/odom", "/imu", "/cmr_experiment/events", f"{prefix}/odometry/filtered",
              f"{prefix}/filter_output_internal", f"{prefix}/context")
    reader = open_reader(run / "bag")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    types = {entry.name: get_message(entry.type)
             for entry in reader.get_all_topics_and_types() if entry.name in topics}
    series = {topic: [] for topic in topics}
    while reader.has_next():
        topic, raw, bag_time = reader.read_next()
        msg = deserialize_message(raw, types[topic])
        if topic == "/cmr_experiment/events":
            event = json.loads(msg.data)
            series[topic].append((bag_time, event))
        elif topic.endswith("/context"):
            series[topic].append((bag_time, msg.health, msg.reason, msg.clock_epoch))
        else:
            stamp = stamp_ns(msg.header.stamp)
            if topic == "/imu":
                series[topic].append((stamp, msg.angular_velocity.z))
            else:
                series[topic].append((stamp, *pose(msg.pose.pose), msg.twist.twist.linear.x))
    return series


def interpolate(times, values, query):
    right = np.searchsorted(times, query, side="left").clip(1, len(times) - 1)
    left = right - 1
    width = times[right] - times[left]
    covered = (query >= times[left]) & (query <= times[right]) & (width <= 50_000_000)
    weight = (query - times[left]) / width
    result = values[left] + (values[right] - values[left]) * weight[:, None]
    return result, covered


def calibrate(run, output):
    result = json.loads((run / "runtime/runner-exit.json").read_text())
    if result["exit"] != 0:
        raise ValueError("calibration requires a completed independent motion run")
    profile = json.loads((run / "artifacts/profile.json").read_text())
    if profile["p2_v1"]["mode"] != "calibration":
        raise ValueError("calibration must use a dedicated capture")
    t, gt = reference(run)
    data = signals(run)
    sensor_source = json.loads((run / "artifacts/p2-imu-source.json").read_text())
    records = {}
    residuals = {}
    for topic, index in (("/odom", 4), ("/imu", 1)):
        if len(data[topic]) < 100:
            raise ValueError(f"missing calibration observations on {topic}")
        rows = np.asarray(data[topic])
        timestamps = rows[:, 0].astype(np.int64)
        minus, before = interpolate(t, gt, timestamps - 50_000_000)
        plus, after = interpolate(t, gt, timestamps + 50_000_000)
        midpoint = (minus[:, 2] + plus[:, 2]) / 2
        dx, dy, yaw = ((plus - minus) / 0.1).T
        vx = dx * np.cos(midpoint) + dy * np.sin(midpoint)
        stationary = (np.hypot(dx, dy) < 0.005) & (np.abs(yaw) < 0.01)
        physical = vx if topic == "/odom" else yaw
        static_mask = before & after & stationary
        motion_mask = before & after & ~stationary
        residual = rows[:, index] - physical
        if min(static_mask.sum(), motion_mask.sum()) < 100 or not np.isfinite(residual).all():
            raise ValueError(f"insufficient static or dynamic residuals for {topic}")
        bias = float(np.mean(residual[static_mask])) if topic == "/imu" else 0.0
        error = residual - bias
        variance = float(max(np.mean(error[static_mask] ** 2), np.mean(error[motion_mask] ** 2)))
        zero_reference_mse = float(np.mean(physical[motion_mask] ** 2))
        if topic == "/imu" and np.mean(error[motion_mask] ** 2) >= zero_reference_mse:
            raise ValueError("gyro on independent movement does not improve on a zero-rate signal")
        records[topic] = {"zero_reference_mse": zero_reference_mse, "static_n": int(static_mask.sum()), "dynamic_n": int(motion_mask.sum()),
                          "static_mean": float(np.mean(residual[static_mask])),
                          "static_variance": float(np.var(residual[static_mask], ddof=1)),
                          "dynamic_mean": float(np.mean(residual[motion_mask])),
                          "dynamic_rms": float(np.sqrt(np.mean(error[motion_mask] ** 2))),
                          "variance": variance, "bias_removed": bias}
        residuals[topic] = (timestamps, rows[:, index], physical, error, static_mask, motion_mask)

    output.mkdir(exist_ok=False, parents=True)
    with (output / "residuals.csv").open("x", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["topic", "stamp_ns", "measurement", "independent_reference", "residual", "segment"])
        for topic, columns in residuals.items():
            for stamp, raw, gt_value, error, static, motion in zip(*columns):
                if static or motion:
                    writer.writerow([topic, stamp, raw, gt_value, error, "static" if static else "motion"])
    model = {
        "schema": "cmr.p2/1.1", "source_level": "E2_SIM_SENSOR",
        "wheel_vx_variance": records["/odom"]["variance"],
        "imu_wz_variance": max(records["/imu"]["variance"], 4e-8),
        "imu_wz_bias": records["/imu"]["bias_removed"],
        "model_version": "sim-tb3-mounted-residual-v1",
        "qualification": "PASS", "sensor_sha256": sensor_source["sensor_sha256"],
        "calibration_run": str(run), "source_frames": ["/odom:odom/base_footprint", "/imu:imu_link"],
        "pose_reference_kind": "Gazebo independent model pose",
        "bag_sha256": inspect_files(run / "bag")[1],
        "gt_sha256": digest(run / "runtime/p0-08-gazebo.jsonl"),
        "method": "maximum static and dynamic mean squared residual; source gyro noise lower bound",
        "reference_difference_span_s": 0.1, "results": records,
    }
    if model["wheel_vx_variance"] <= 0:
        raise ValueError("no positive wheel covariance identified")
    with (output / "model.json").open("x") as stream:
        json.dump(model, stream, indent=2, allow_nan=False)
        stream.write("\n")


def displacement(values):
    local = values - values[0]
    yaw = values[0, 2]
    x, y = local[:, :2].T.copy()
    local[:, 0] = x * math.cos(yaw) + y * math.sin(yaw)
    local[:, 1] = -x * math.sin(yaw) + y * math.cos(yaw)
    local[:, 2] = np.arctan2(np.sin(local[:, 2]), np.cos(local[:, 2]))
    return local


def compare(run, output, case):
    t, gt = reference(run)
    data = signals(run)
    namespace = f"/cmr_p2/{run.name}"
    filtered = f"{namespace}/odometry/filtered"
    contexts = data[f"{namespace}/context"]
    runner = json.loads((run / "runtime/runner-exit.json").read_text())
    hold_file = run / "runtime/p2-hold.json"
    hold = json.loads(hold_file.read_text()) if hold_file.exists() else None
    start, end = int(t[0]), int(t[-1])
    target = np.zeros(3)
    if case in ("P201.Static", "P201.Resource"):
        if not hold:
            raise ValueError("missing qualified hold")
        start, end = hold["start_ns"], hold["end_ns"]
    if case in ("P201.Straight", "P201.Rotate"):
        motion = json.loads((run / "artifacts/p2-maneuver.json").read_text())
        goal = motion["target"]
        initial = motion["start"]
        target[0] = goal["position"]["x"] - initial["position"]["x"]
        target[1] = goal["position"]["y"] - initial["position"]["y"]
        target[2] = 2 * math.atan2(goal["orientation"]["z"], goal["orientation"]["w"])
        marker = next((row for row in data["/cmr_experiment/events"]
                       if row[1]["type"] == "DELIVERY_WAIT_HANDOVER"), None)
        if marker is None:
            raise ValueError("outbound leg has no qualified arrival marker")
        end = marker[0]
    comparisons = {}
    output.mkdir(exist_ok=False, parents=True)
    with (output / "pose-error.csv").open("x", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["topic", "sample_index", "stamp_ns", "truth_x", "truth_y", "truth_yaw",
                         "estimate_x", "estimate_y", "estimate_yaw", "translation_error", "yaw_error"])
        for topic in ("/odom", filtered):
            samples = np.asarray(data[topic])
            if samples.ndim != 2:
                comparisons[topic] = {"status": "FAIL", "reason": "no samples"}
                continue
            stamps = samples[:, 0].astype(np.int64)
            ref, mask = interpolate(t, gt, stamps)
            keep = (stamps >= start) & (stamps <= end) & mask
            if keep.sum() < 2:
                comparisons[topic] = {"status": "BLOCKED", "reason": "insufficient matched GT"}
                continue
            estimate, truth = displacement(samples[keep, 1:4]), displacement(ref[keep])
            distance = np.linalg.norm(estimate[:, :2] - truth[:, :2], axis=1)
            yaw = np.abs(np.arctan2(np.sin(estimate[:, 2] - truth[:, 2]),
                                   np.cos(estimate[:, 2] - truth[:, 2])))
            for i, stamp, trusted, estimated, d, y in zip(
                    np.flatnonzero(keep), stamps[keep], truth, estimate, distance, yaw):
                writer.writerow([topic, i, stamp, *trusted, *estimated, d, y])
            start_gap, end_gap = int(stamps[keep][0] - start), int(end - stamps[keep][-1])
            entry = {
                "status": "OBSERVED",
                "samples": int(keep.sum()), "max_estimation_error_m": float(distance.max()),
                "max_estimation_error_rad": float(yaw.max()),
                "endpoint_estimation_error_m": float(distance[-1]),
                "endpoint_estimation_error_rad": float(yaw[-1]),
                "max_displacement_m": float(np.linalg.norm(estimate[:, :2], axis=1).max()),
                "max_rotation_rad": float(np.abs(estimate[:, 2]).max()),
                "truth_terminal_displacement": truth[-1].tolist(),
                "estimate_terminal_displacement": estimate[-1].tolist(),
                "truth_target_error_m": float(np.linalg.norm(truth[-1, :2] - target[:2])),
                "truth_target_error_rad": abs(math.remainder(truth[-1, 2] - target[2], 2 * math.pi)),
                "start_gap_ns": start_gap, "end_gap_ns": end_gap,
            }
            if case == "P201.Static":
                entry["within_limits"] = (entry["max_displacement_m"] <= 0.03 and
                                          entry["max_rotation_rad"] <= 0.03 and
                                          entry["max_estimation_error_m"] <= 0.03 and
                                          entry["max_estimation_error_rad"] <= 0.03 and
                                          start_gap <= 300_000_000 and end_gap <= 300_000_000)
            if case in ("P201.Straight", "P201.Rotate"):
                position = 0.10 if case == "P201.Straight" else 0.05
                entry["within_limits"] = (entry["truth_target_error_m"] <= position and
                                          entry["truth_target_error_rad"] <= 0.10 and
                                          entry["endpoint_estimation_error_m"] <= position and
                                          entry["endpoint_estimation_error_rad"] <= 0.10 and
                                          start_gap <= 5_000_000_000 and end_gap <= 300_000_000)
            comparisons[topic] = entry
    report = {"case": case, "runner_exit": runner["exit"], "hold": hold,
              "gt_kind": "Gazebo transport independent model pose",
              "gt_sha256": digest(run / "runtime/p0-08-gazebo.jsonl"),
              "start_ns": start, "end_ns": end, "comparison": comparisons,
              "context_ok": sum(row[1] == 0 for row in contexts), "context_count": len(contexts),
              "context_reasons": sorted({row[2] for row in contexts})}
    result = comparisons[filtered]
    if case == "P201.Resource":
        from resource_report import resource_report
        report["resources"] = resource_report(run, hold)
        result["within_limits"] = report["resources"]["status"] == "PASS"
    if hold:
        window_contexts = [row for row in contexts if start <= row[0] <= end]
        report["window_context_count"] = len(window_contexts)
        report["window_invalid_contexts"] = sum(row[1] != 0 for row in window_contexts)
        report["window_health_pass"] = bool(window_contexts) and not report["window_invalid_contexts"]
        result["within_limits"] = result.get("within_limits", False) and report["window_health_pass"]
    report["status"] = "PASS" if runner["exit"] == 0 and report["context_ok"] and (
        result.get("within_limits", result["status"] == "OBSERVED")) else "FAIL"
    if result["status"] == "BLOCKED":
        report["status"] = "BLOCKED"
    with (output / "estimation-result.json").open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("calibrate", "compare"))
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--case", default="shadow", choices=["shadow", "P201.Static", "P201.Straight", "P201.Rotate", "P201.Resource"])
    args = parser.parse_args()
    if args.mode == "calibrate":
        calibrate(args.run.resolve(strict=True), args.output)
    else:
        result = compare(args.run.resolve(strict=True), args.output, args.case)
        if result["status"] != "PASS":
            parser.exit(1, result["status"] + "\n")


if __name__ == "__main__":
    main()
