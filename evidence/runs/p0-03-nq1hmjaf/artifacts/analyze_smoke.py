"""Offline-only P0-03 evidence calculation. Does not import ROS or send commands."""
from pathlib import Path
import argparse
import bisect
import collections
import hashlib
import json
import math
import re


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def read_json(path):
    return json.loads(path.read_text(), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))


def rows(path):
    result = []
    for number, line in enumerate(path.read_text().splitlines(), 1):
        item = json.loads(line, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
        if not finite(item.get("receive_monotonic_s")):
            raise ValueError(f"Invalid receive time at {path}:{number}")
        result.append(item)
    return result


def valid_pose(data):
    p, q = data.get("position", []), data.get("quaternion_xyzw", [])
    return (len(p) == 3 and len(q) == 4 and all(finite(x) for x in p + q)
            and abs(sum(x*x for x in q) - 1) <= 1e-6)


def yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def nearest_clock(sample, clocks):
    times = [x["receive_monotonic_s"] for x in clocks]
    index = bisect.bisect_left(times, sample["receive_monotonic_s"])
    candidates = clocks[max(0, index-1):min(len(clocks), index+1)]
    return min(candidates, key=lambda x: abs(x["receive_monotonic_s"]-sample["receive_monotonic_s"]))


def window(samples):
    counts = collections.Counter(x["kind"] for x in samples)
    clocks = [x for x in samples if x["kind"] == "clock" and finite(x["data"].get("sim_s"))]
    raw_odom = [x for x in samples if x["kind"] == "odom"]
    odom = [x for x in raw_odom if valid_pose(x["data"]) and finite(x["data"].get("stamp_s"))
            and len(x["data"].get("linear", [])) == 3 and len(x["data"].get("angular", [])) == 3
            and all(finite(y) for y in x["data"]["linear"] + x["data"]["angular"])]
    tf = [x for x in samples if x["kind"] == "map_base" and valid_pose(x["data"])
          and finite(x["data"].get("stamp_s"))]
    result = {"counts": dict(counts), "receive_start_s": samples[0]["receive_monotonic_s"],
              "receive_end_s": samples[-1]["receive_monotonic_s"],
              "clock_jump_events": counts["clock_jump"], "invalid_odom_samples": len(raw_odom)-len(odom),
              "command_samples": {k: counts[k] for k in ("/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel")},
              "last_cmd_vel": None, "odom": None, "last_map_base": tf[-1] if tf else None}
    commands = [x for x in samples if x["kind"] == "/cmd_vel" and
                len(x["data"].get("linear", [])) == 3 and len(x["data"].get("angular", [])) == 3 and
                all(finite(y) for y in x["data"]["linear"]+x["data"]["angular"])]
    if commands:
        result["last_cmd_vel"] = commands[-1]
    if clocks:
        result["clock"] = {"samples": len(clocks), "sim_start_s": clocks[0]["data"]["sim_s"],
                           "sim_end_s": clocks[-1]["data"]["sim_s"],
                           "backsteps": sum(b["data"]["sim_s"] < a["data"]["sim_s"] for a, b in zip(clocks, clocks[1:]))}
    if odom:
        start = odom[0]["data"]
        gaps = [b["data"]["stamp_s"]-a["data"]["stamp_s"] for a, b in zip(odom, odom[1:])]
        drift = max(math.hypot(x["data"]["position"][0]-start["position"][0],
                               x["data"]["position"][1]-start["position"][1]) for x in odom)
        nearest = [(x, nearest_clock(x, clocks)) for x in odom] if clocks else []
        max_sim_age = max(abs(x["data"]["stamp_s"]-c["data"]["sim_s"]) for x, c in nearest) if nearest else None
        max_wall_age = max(abs(x["receive_monotonic_s"]-c["receive_monotonic_s"]) for x, c in nearest) if nearest else None
        details = {"samples": len(odom), "sim_start_s": start["stamp_s"],
                   "sim_end_s": odom[-1]["data"]["stamp_s"],
                   "span_sim_s": odom[-1]["data"]["stamp_s"]-start["stamp_s"],
                   "first_position": start["position"], "last_position": odom[-1]["data"]["position"],
                   "max_gap_sim_s": max(gaps) if gaps else None,
                   "nonforward_stamp_pairs": sum(g <= 0 for g in gaps),
                   "max_linear_m_s": max(math.hypot(x["data"]["linear"][0], x["data"]["linear"][1]) for x in odom),
                   "max_angular_rad_s": max(abs(x["data"]["angular"][2]) for x in odom),
                   "max_drift_m": drift,
                   "net_displacement_m": math.hypot(odom[-1]["data"]["position"][0]-start["position"][0], odom[-1]["data"]["position"][1]-start["position"][1]),
                   "max_odom_vs_nearest_clock_abs_sim_s": max_sim_age,
                   "max_nearest_clock_receive_distance_wall_s": max_wall_age}
        details["stationary_window_criteria"] = (
            len(odom) >= 5 and details["span_sim_s"] >= 3 and len(raw_odom) == len(odom)
            and bool(gaps) and all(0 < g <= 0.5 for g in gaps)
            and details["max_linear_m_s"] <= 0.01 and details["max_angular_rad_s"] <= 0.03
            and drift <= 0.02 and max_sim_age is not None and max_sim_age <= 0.5
            and max_wall_age <= 0.5 and result["clock_jump_events"] == 0
            and result["clock"]["backsteps"] == 0 and result["clock"]["sim_end_s"] > result["clock"]["sim_start_s"])
        result["odom"] = details
    result["tf_unique_stamp_count"] = len({x["data"]["stamp_s"] for x in tf})
    result["valid_tf_samples"] = len(tf)
    return result


def analyze(run):
    dispatch = read_json(run/"runtime/goal-dispatch-before.json")
    returned = read_json(run/"runtime/goal-command-returned.json")
    original = rows(run/"runtime/observations.jsonl")
    supplement = rows(run/"runtime/observations-cleanup-20261022.jsonl")
    original_window, cleanup_window = window(original), window(supplement)
    text = (run/"logs/navigate-goal.log").read_text()
    ids = sorted(set(re.findall(r"Goal accepted with ID:\s*([0-9a-fA-F]{32})", text)))
    action = "REJECTED" if "Goal was rejected." in text and not ids else None
    terminal = re.findall(r"Goal finished with status:\s*([A-Z_]+)", text)
    if terminal:
        action = terminal[-1]
    phase_samples = [x for x in original+supplement if dispatch["monotonic_s"] <= x["receive_monotonic_s"] <= returned["monotonic_s"]]
    phase_odom = [x for x in phase_samples if x["kind"] == "odom" and valid_pose(x["data"])]
    phase_displacement = None
    if len(phase_odom) >= 2:
        a, b = phase_odom[0]["data"]["position"], phase_odom[-1]["data"]["position"]
        phase_displacement = math.hypot(b[0]-a[0], b[1]-a[1])
    scenario = read_json(run/"artifacts/scenario.yaml")  # This frozen YAML uses JSON syntax.
    target = scenario["task"]["goal_pose"]
    position_error, yaw_error = None, None
    final = cleanup_window["last_map_base"]
    if final:
        p, q = final["data"]["position"], final["data"]["quaternion_xyzw"]
        position_error = math.hypot(p[0]-target["x_m"], p[1]-target["y_m"])
        yaw_error = abs(math.atan2(math.sin(yaw(q)-target["yaw_rad"]), math.cos(yaw(q)-target["yaw_rad"])))
        final = {**final, "yaw_rad": yaw(q), "quaternion_norm_squared": sum(x*x for x in q),
                 "goal_position_error_m": position_error, "goal_yaw_error_rad": yaw_error,
                 "after_command_return_wall_s": final["receive_monotonic_s"]-returned["monotonic_s"],
                 "ros_now_minus_tf_stamp_s": final["ros_now_s"]-final["data"]["stamp_s"],
                 "scope": "supplementary pre-cleanup localization estimate; not an immediate navigation terminal pose or physical truth"}
    input_paths = ["logs/navigate-goal.log", "logs/navigate-goal.exit", "runtime/goal-dispatch-before.json",
                   "runtime/goal-command-returned.json", "runtime/observations.jsonl", "runtime/observations-cleanup-20261022.jsonl",
                   "artifacts/scenario.yaml", "runtime/cleanup-evidence-20261022.json", "runtime/session-ended.json"]
    return {"run_id": run.name, "validation_scope": "record_contract", "action": {"terminal": action,
            "accepted_goal_uuids": ids, "accepted_goal_uuid": ids[0] if len(ids) == 1 else None,
            "command_exit": returned["command_exit"], "action_success": action == "SUCCEEDED" if action in ("SUCCEEDED", "REJECTED", "ABORTED", "CANCELED") else None,
            "dispatch_monotonic_s": dispatch["monotonic_s"], "return_monotonic_s": returned["monotonic_s"],
            "command_duration_s": returned["monotonic_s"]-dispatch["monotonic_s"]},
            "original_observation": original_window, "supplementary_cleanup_observation": cleanup_window,
            "observer_ended_before_dispatch_s": dispatch["monotonic_s"]-original_window["receive_end_s"],
            "action_interval_observed_samples": len(phase_samples),
            "action_interval_odom_displacement_m": phase_displacement,
            "final_localization_estimate": final, "last_cmd_vel": cleanup_window["last_cmd_vel"] or original_window["last_cmd_vel"],
            "post_success_stop_observed": None,
            "clock_jump_scope": "No jumps detected in the two recorded windows; the unrecorded gap and whole session are unknown",
            "cleanup": read_json(run/"runtime/cleanup-evidence-20261022.json"),
            "session": read_json(run/"runtime/session-ended.json"),
            "engineering_result": "FAILED: goal rejected by inactive action server; no accepted-goal motion/success/stop evidence",
            "limitations": ["No missing Twist samples were converted to zero.", "The supplementary stationary window is pre-cleanup evidence, not successful post-goal stopping.", "No independent collision evaluation or physical ground truth.", "RViz movement was not independently observed by this analysis."],
            "input_sha256": {name: hashlib.sha256((run/name).read_bytes()).hexdigest() for name in input_paths}}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = analyze(args.run)
    with args.output.open("x") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"output": str(args.output), "engineering_result": report["engineering_result"],
                      "action": report["action"], "final_localization_estimate": report["final_localization_estimate"],
                      "supplementary_stationary_window": report["supplementary_cleanup_observation"]["odom"]}, ensure_ascii=False, indent=2))
