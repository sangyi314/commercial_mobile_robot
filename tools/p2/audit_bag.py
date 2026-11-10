#!/usr/bin/env python3
"""Read-only MCAP audit of P2 sensor stamps, covariance and exact-time TF."""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformException
import yaml

NS = 1_000_000_000
MAX_AGE = {"/scan": 300_000_000, "/odom": 200_000_000, "/imu": 100_000_000}
SENSORS = {
    "/scan": ("sensor_msgs/msg/LaserScan", "base_scan"),
    "/odom": ("nav_msgs/msg/Odometry", "odom"),
    "/imu": ("sensor_msgs/msg/Imu", "imu_link"),
}
TYPES = {
    **{topic: entry[0] for topic, entry in SENSORS.items()},
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/clock": "rosgraph_msgs/msg/Clock",
}
COLUMNS = [
    "index", "topic", "epoch", "bag_ns", "stamp_ns", "clock_ns", "bag_age_ns",
    "frame", "child_frame", "scan_end_ns", "tf_coverage", "covariance", "issues",
]


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < NS:
        raise ValueError("invalid ROS stamp")
    return stamp.sec * NS + stamp.nanosec


def open_reader(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    return reader


def messages(bag):
    reader = open_reader(bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TYPES)))
    types = {entry.name: get_message(entry.type)
             for entry in reader.get_all_topics_and_types()
             if TYPES.get(entry.name) == entry.type}
    index = 0
    while reader.has_next():
        topic, raw, bag_ns = reader.read_next()
        if topic in types:
            yield index, topic, deserialize_message(raw, types[topic]), bag_ns
        index += 1


def inspect_files(bag):
    metadata = bag / "metadata.yaml"
    info = yaml.safe_load(metadata.read_text())["rosbag2_bagfile_information"]
    if info["storage_identifier"] != "mcap":
        raise ValueError("input must be MCAP")
    names = info["relative_file_paths"]
    if not names or len(set(names)) != len(names):
        raise ValueError("invalid MCAP file list")
    files = [metadata]
    for name in names:
        path = bag / name
        if path.resolve().parent != bag or path.suffix != ".mcap" or not path.is_file():
            raise ValueError(f"invalid MCAP input member: {name}")
        files.append(path)
    return info, {file.name: digest(file) for file in files}


def clock_epochs(bag):
    epoch = 0
    clock = None
    for index, topic, msg, bag_ns in messages(bag):
        if topic == "/clock":
            new_clock = stamp_ns(msg.clock)
            if clock is not None and new_clock < clock:
                epoch += 1
            clock = new_clock
        yield index, topic, msg, bag_ns, epoch, clock


def transform_index(bag, duration):
    buffers = {}
    static = {}
    edge_counts = Counter()
    failures = []
    for index, topic, msg, _, epoch, _ in clock_epochs(bag):
        if topic not in ("/tf", "/tf_static"):
            continue
        if epoch not in buffers:
            buffers[epoch] = Buffer(cache_time=duration)
            for tf in static.values():
                buffers[epoch].set_transform_static(tf, "bag")
        for transform in msg.transforms:
            try:
                stamp_ns(transform.header.stamp)
                edge = (transform.header.frame_id, transform.child_frame_id)
                if not edge[0] or not edge[1] or edge[0] == edge[1]:
                    raise ValueError("invalid TF frame")
                edge_counts[(epoch, topic, *edge)] += 1
                if topic == "/tf_static":
                    static[edge] = transform
                    buffers[epoch].set_transform_static(transform, "bag")
                else:
                    buffers[epoch].set_transform(transform, "bag")
            except (ValueError, TransformException) as exc:
                failures.append({"index": index, "reason": str(exc)})
    return buffers, [{"epoch": epoch, "topic": topic, "parent": parent, "child": child,
                      "count": count}
                     for (epoch, topic, parent, child), count in sorted(edge_counts.items())], failures


def covariance(values, size, *, imu=False):
    if imu and values[0] == -1.0:
        return "UNAVAILABLE"
    matrix = np.asarray(values, dtype=np.float64).reshape((size, size))
    if not np.isfinite(matrix).all():
        return "INVALID"
    scale = np.abs(matrix).max()
    if scale == 0:
        return "UNKNOWN"
    scaled = matrix / scale
    eigen = np.linalg.eigvalsh((scaled + scaled.T) / 2)
    tol = 1e-10 * max(1 / max(scale, 1), np.abs(eigen).max())
    if np.abs(scaled - scaled.T).max() > tol or eigen.min() < -tol:
        return "INVALID"
    return "AVAILABLE"


def coverage(buffer, target, source, stamps):
    # TF time zero requests the latest value, which is forbidden for this audit.
    if buffer is None or not source or any(value <= 0 for value in stamps):
        return False
    return all(buffer.can_transform(target, source, Time(nanoseconds=value)) for value in stamps)


def summarize(series):
    if not series:
        return {"count": 0, "first_ns": None, "last_ns": None, "hz": None}
    span = series[-1] - series[0]
    return {"count": len(series), "first_ns": series[0], "last_ns": series[-1],
            "hz": (len(series) - 1) * NS / span if span > 0 else None}


def check_sample(topic, msg, stamp, buffer, timing):
    issues = []
    frame = msg.header.frame_id
    child = msg.child_frame_id if topic == "/odom" else None
    if frame != SENSORS[topic][1] or (topic == "/odom" and child != "base_footprint"):
        issues.append("FRAME_MISMATCH")

    scan_end = None
    cov = {}
    if topic == "/scan":
        if not 0 < len(msg.ranges) <= 4096 or not all(
                math.isfinite(x) for x in (msg.angle_min, msg.angle_max, msg.angle_increment,
                                           msg.range_min, msg.range_max, msg.time_increment, msg.scan_time)
        ) or msg.angle_increment <= 0 or not 0 <= msg.range_min < msg.range_max or any(
                math.isnan(value) for value in msg.ranges):
            issues.append("INVALID_SCAN")
        if msg.time_increment > 0 and msg.ranges and stamp is not None:
            scan_end = stamp + round((len(msg.ranges) - 1) * msg.time_increment * NS)
            if scan_end >= 2**63:
                scan_end = None
                issues.append("STAMP_OVERFLOW")
        elif timing == "instantaneous" and msg.time_increment == 0:
            scan_end = stamp
        else:
            issues.append("TIMING_UNKNOWN")
        checked = [stamp] if scan_end is None else [stamp, scan_end]
        tf = stamp is not None and coverage(buffer, "odom", frame, checked)
    elif topic == "/imu":
        cov = {kind: covariance(getattr(msg, kind + "_covariance"), 3, imu=True)
               for kind in ("orientation", "angular_velocity", "linear_acceleration")}
        if not math.isfinite(msg.angular_velocity.z):
            issues.append("NONFINITE_WZ")
        tf = stamp is not None and coverage(buffer, "base_footprint", frame, [stamp])
    else:
        cov = {field: covariance(getattr(msg, field).covariance, 6)
               for field in ("pose", "twist")}
        if not math.isfinite(msg.twist.twist.linear.x):
            issues.append("NONFINITE_VX")
        tf = stamp is not None and coverage(buffer, frame, child, [stamp])
    if not tf:
        issues.append("TF_UNAVAILABLE")
    if "INVALID" in cov.values():
        issues.append("INVALID_COVARIANCE")
    return frame, child, scan_end, cov, tf, issues


def audit(bag, output, recorded_clock, timing):
    bag = bag.resolve(strict=True)
    output = output.resolve()
    if output == bag or bag in output.parents:
        raise ValueError("output must be outside the raw bag")
    info, hashes = inspect_files(bag)
    output.mkdir(parents=True, exist_ok=False)
    duration_ns = int(info["duration"]["nanoseconds"])
    if duration_ns < 0 or duration_ns >= 2**63 - 2 * NS:
        raise ValueError("unsupported bag duration")
    buffers, edges, tf_errors = transform_index(bag, Duration(nanoseconds=duration_ns + 2 * NS))
    stats = defaultdict(lambda: {"bag": [], "stamp": [], "age": [],
                                 "cov": Counter(), "issues": Counter(), "tf_ok": 0})
    clock_stats = defaultdict(list)
    last = {}
    with (output / "samples.csv").open("x", newline="") as stream:
        rows = csv.DictWriter(stream, fieldnames=COLUMNS)
        rows.writeheader()
        for index, topic, msg, bag_ns, epoch, clock in clock_epochs(bag):
            if topic == "/clock":
                clock_stats[epoch].append(clock)
            if topic not in SENSORS:
                continue
            group = stats[topic, epoch]
            group["bag"].append(bag_ns)
            try:
                stamp = stamp_ns(msg.header.stamp)
            except ValueError:
                stamp = None
            frame, child, scan_end, cov, tf, issues = check_sample(
                topic, msg, stamp, buffers.get(epoch), timing)
            age = None
            if stamp is None:
                issues.append("INVALID_STAMP")
            else:
                group["stamp"].append(stamp)
                previous = last.get((topic, epoch), -1)
                if stamp <= previous:
                    issues.append("DUPLICATE_STAMP" if stamp == previous else "OUT_OF_ORDER")
                else:
                    last[topic, epoch] = stamp
                if recorded_clock == "simulation" and clock is not None:
                    age = bag_ns - stamp
                    group["age"].append(age)
                    if age > MAX_AGE[topic]:
                        issues.append("INPUT_STALE")
                    elif age < -20_000_000:
                        issues.append("FUTURE_STAMP")
            group["tf_ok"] += int(tf)
            group["issues"].update(issues)
            group["cov"].update(f"{field}:{state}" for field, state in cov.items())
            rows.writerow(dict(index=index, topic=topic, epoch=epoch, bag_ns=bag_ns,
                               stamp_ns=stamp, clock_ns=clock, bag_age_ns=age, frame=frame,
                               child_frame=child, scan_end_ns=scan_end, tf_coverage=int(tf),
                               covariance=json.dumps(cov, sort_keys=True), issues="|".join(issues)))
    metadata = {entry["topic_metadata"]["name"]: entry["topic_metadata"]
                for entry in info["topics_with_message_count"]}
    report = {
        "schema": "cmr.p2/1.1", "bag": str(bag), "sha256": hashes,
        "bag_clock": recorded_clock, "scan_timing_assumption": timing,
        "publisher_provenance": "not established by MCAP; bind to capture manifest and graph",
        "topics": metadata, "tf_edges": edges, "tf_errors": tf_errors,
        "missing_or_wrong_type": [topic for topic, expected in TYPES.items()
                                  if metadata.get(topic, {}).get("type") != expected],
        "clock_epochs": {epoch: summarize(values) for epoch, values in clock_stats.items()},
        "sensor_groups": [
            {"topic": topic, "epoch": epoch, "record_time": summarize(group["bag"]),
             "header_time": summarize(group["stamp"]), "tf_covered": group["tf_ok"],
             "covariance": dict(group["cov"]), "issues": dict(group["issues"]),
             "age_p99_ns": float(np.percentile(group["age"], 99)) if group["age"] else None,
             "live_arrival_hz": None}
            for (topic, epoch), group in sorted(stats.items())
        ],
    }
    if inspect_files(bag)[1] != hashes:
        raise ValueError("input MCAP or metadata changed during audit")
    with (output / "audit.json").open("x") as stream:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="Original MCAP bag directory")
    parser.add_argument("--output", required=True, type=Path, help="New output directory")
    parser.add_argument("--recorded-clock", choices=("unknown", "simulation"), default="unknown",
                        help="Use simulation only with capture evidence that bag uses /clock")
    parser.add_argument("--scan-timing", choices=("unknown", "instantaneous"), default="unknown",
                        help="Use instantaneous only with an independently qualified sensor")
    args = parser.parse_args()
    try:
        audit(args.bag, args.output, args.recorded_clock, args.scan_timing)
    except (ValueError, OSError, RuntimeError) as exc:
        parser.exit(2, f"audit failed: {exc}\n")


if __name__ == "__main__":
    main()
