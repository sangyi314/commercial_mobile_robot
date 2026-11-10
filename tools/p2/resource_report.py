"""Compute bounded-queue and resource observations using recorded steady and ROS clocks."""
import csv
import json
from pathlib import Path

import numpy as np

LIMITS = {
    "source_sample_min_hz": {"/scan": 4.75, "/odom": 79.0, "/imu": 190.0},
    "accepted_age_max_s": {"/scan": 0.30, "/odom": 0.20, "/imu": 0.10},
    "adapter_pending_capacity": 5, "tf_edge_capacity": 128, "tf_cache_s": 10.0,
    "p2_rss_growth_limit_bytes": 64 * 1024 * 1024,
    "basis": "Source qualification at 5/83.333/200 Hz; 5% count margin frozen before Resource; contract age/queue limits",
}


def describe(values):
    a = np.asarray(values, dtype=float)
    return ({"n": len(a), "p50": float(np.percentile(a, 50)), "p99": float(np.percentile(a, 99)),
             "max": float(a.max()), "min": float(a.min())} if len(a) else None)


def resource_report(run, hold):
    start, end = hold["start_ns"], hold["end_ns"]
    rows = list(csv.DictReader((run / "runtime/p2-input-time.csv").open()))
    sources = {}
    for topic, minimum in LIMITS["source_sample_min_hz"].items():
        window = [r for r in rows if r["topic"] == topic and start <= int(r["stamp_ns"]) <= end]
        stamps = np.asarray([int(r["stamp_ns"]) for r in window], dtype=np.int64)
        arrivals = np.asarray([int(r["receive_steady_ns"]) for r in window], dtype=np.int64)
        age = [(int(r["receive_ros_ns"]) - int(r["stamp_ns"])) / 1e9 for r in window]
        accepted_age = [(int(r["receive_ros_ns"]) - int(r["stamp_ns"])) / 1e9
                        for r in window if r["reason"] == "NONE"]
        sample_hz = (len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0
        arrival_hz = (len(arrivals) - 1) * 1e9 / (arrivals[-1] - arrivals[0]) if len(arrivals) > 1 else 0
        reasons = {}
        for r in window:
            reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
        sources[topic] = {
            "n": len(window), "sample_hz": sample_hz, "arrival_wall_hz": arrival_hz,
            "age_s": describe(age), "accepted_age_s": describe(accepted_age),
            "reasons": reasons,
            "passed": bool(len(stamps) >= minimum * (end - start) / 1e9 and
                           sample_hz >= minimum and accepted_age and
                           max(accepted_age) <= LIMITS["accepted_age_max_s"][topic])}
    samples = []
    for r in map(json.loads, (run / "runtime/p2-resources.jsonl").open()):
        t = r["simulation_stats"].get("sim_time", {})
        ns = int(t.get("sec", 0)) * 1_000_000_000 + int(t.get("nsec", 0))
        if start <= ns <= end:
            samples.append((ns, r))
    cpu, ram, rtf = [], [], []
    previous = None
    for sim, row in samples:
        group = [p for p in row["processes"] if p["role"] == "p2-estimation"]
        ram.append(sum(p["rss_bytes"] for p in group))
        if previous:
            prior = {(p["pid"], p["start_ticks"]): p for p in previous[1]["processes"]}
            dt = (row["steady_ns"] - previous[1]["steady_ns"]) / 1e9
            if dt > 0:
                cpu.append(100 * sum(p["cpu_seconds"] - prior[(p["pid"], p["start_ticks"])]["cpu_seconds"]
                                    for p in group if (p["pid"], p["start_ticks"]) in prior) / dt)
                rtf.append((sim - previous[0]) / 1e9 / dt)
        previous = (sim, row)
    growth = float(np.median(ram[-30:]) - np.median(ram[:30])) if len(ram) >= 60 else None
    passed = (all(r["passed"] for r in sources.values()) and len(samples) >= 590 and
              cpu and rtf and min(ram) > 0 and growth is not None and
              growth <= LIMITS["p2_rss_growth_limit_bytes"])
    return {"status": "PASS" if passed else "FAIL", "frozen_limits": LIMITS,
            "source_metrics": sources, "steady_samples": len(samples),
            "p2_cpu_percent_one_core_100": describe(cpu), "p2_rss_bytes": describe(ram),
            "p2_rss_growth_bytes": growth, "real_time_factor": describe(rtf),
            "queue_bound_kind": "STATIC source bound plus observed RSS; no inference of infinite-duration proof"}
