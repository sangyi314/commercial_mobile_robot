#!/usr/bin/env python3
"""Corrupt only private P2 copies of live Gazebo inputs. Never publish control or raw topics."""
import argparse
from collections import deque
from copy import deepcopy
from functools import partial
import csv
import json
import os
from pathlib import Path
import time

NS = 1_000_000_000


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--case", required=True, choices=["P201.FaultTime", "P201.TFOwnership"])
    args = parser.parse_args()
    run = args.run.resolve(strict=True)
    profile = json.loads((run / "artifacts/profile.json").read_text())["p2_v1"]
    if (os.environ.get("ROS_DOMAIN_ID") != "71" or
            not os.environ.get("CMR_OWNERSHIP_TOKEN", "").endswith(":p2-fault-proxy") or
            profile.get("fault_test") != args.case or profile["mode"] != "shadow" or
            profile["run_id"] != run.name):
        parser.error("requires this Runner's explicit shadow fault profile")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from cmr_interfaces.msg import P2Context
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    rclpy.init(args=[])
    namespace = "/cmr_p2/" + run.name
    feed = namespace + "/fault_input"
    node = Node("p2_fault_proxy", namespace=namespace)
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    topics = {"clock": Clock, "imu": Imu, "odom": Odometry, "scan": LaserScan}
    pubs = {name: node.create_publisher(typ, feed + "/" + name, qos_profile_sensor_data)
            for name, typ in topics.items()}
    tf_pubs = {topic: node.create_publisher(TFMessage, feed + topic, qos)
               for topic, qos in (("/tf", 100), ("/tf_static", durable))}
    rogue = node.create_publisher(TFMessage, feed + "/tf", 5)
    raw_edges = set()
    raw_graph = {}
    tf_latest = None
    last_imu = None
    queue = deque(maxlen=128)
    state = {"clock": 0, "context": None, "outputs": 0, "stage": "waiting",
             "job": None, "start": 0, "sent": False, "index": 0, "baseline": 0,
             "last_output": 0, "context_before": None}
    jobs = (["duplicate_tf"] if args.case == "P201.TFOwnership" else
            ["delay50", "delay100", "delay200", "future", "duplicate", "old",
             "tf_missing", "clock_pause", "clock_reset"])
    rows = []
    events = (run / "runtime/p2-fault-events.jsonl").open("x")
    result_path = run / "runtime/p2-fault-result.json"
    trace = run / "runtime/p2-input-time.csv"

    def log(kind, **data):
        events.write(json.dumps({"kind": kind, "steady_ns": time.monotonic_ns(),
                                 "source_ros_ns": state["clock"], **data}) + "\n")
        events.flush()

    def stamp(value):
        return value.sec * NS + value.nanosec

    def set_stamp(value, ns):
        value.sec, value.nanosec = divmod(ns, NS)

    def on_raw(key, msg):
        nonlocal last_imu
        job = state["job"] if state["stage"] == "injecting" else None
        if key == "clock":
            state["clock"] = stamp(msg.clock)
            if job == "clock_pause":
                return
            if job == "clock_reset":
                msg = deepcopy(msg)
                set_stamp(msg.clock, state["clock"] - 2 * NS)
            pubs[key].publish(msg)
            return
        if key == "imu":
            if job and job.startswith("delay"):
                if len(queue) == queue.maxlen:
                    raise RuntimeError("fault fixture capacity exceeded")
                queue.append(deepcopy(msg))
                return
            if job in ("duplicate", "old", "future"):
                if state["sent"] or last_imu is None:
                    return
                msg = deepcopy(last_imu if job != "future" else msg)
                if job == "old":
                    set_stamp(msg.header.stamp, stamp(msg.header.stamp) - 1_000_000)
                elif job == "future":
                    set_stamp(msg.header.stamp, state["clock"] + 100_000_000)
                state["sent"] = True
                log("corrupted_imu", fault=job, stamp_ns=stamp(msg.header.stamp))
                pubs[key].publish(msg)
                return
            last_imu = deepcopy(msg)
        pubs[key].publish(msg)

    def on_tf(msg, static):
        nonlocal tf_latest
        topic = "/tf_static" if static else "/tf"
        for tf in msg.transforms:
            raw_edges.add((tf.header.frame_id, tf.child_frame_id))
            if tf.header.frame_id == "odom" and tf.child_frame_id == "base_footprint":
                tf_latest = TFMessage(transforms=[tf])
        if state["stage"] == "injecting" and state["job"] == "tf_missing" and not static:
            return
        tf_pubs[topic].publish(msg)

    for key, typ in topics.items():
        node.create_subscription(typ, "/" + key, partial(on_raw, key),
                                 qos_profile_sensor_data)
    node.create_subscription(TFMessage, "/tf", lambda msg: on_tf(msg, False), 100)
    node.create_subscription(TFMessage, "/tf_static", lambda msg: on_tf(msg, True), durable)

    def context(msg):
        state["context"] = msg
        log("context", health=msg.health, reason=msg.reason, clock_epoch=msg.clock_epoch,
            odom_epoch=msg.odom_epoch, sequence=msg.sequence)

    def output(_msg):
        state["outputs"] += 1
        state["last_output"] = time.monotonic()

    node.create_subscription(P2Context, namespace + "/context", context, durable)
    node.create_subscription(Odometry, namespace + "/odometry/filtered", output, 5)

    def finish():
        status = "PASS" if len(rows) == len(jobs) and all(r["status"] == "PASS" for r in rows) else "FAIL"
        result = {"case": args.case, "status": status, "evidence_kind": "E2_RUNTIME",
                  "scope": "Private P2 copies of live Gazebo sensors; original Gate/Nav2 inputs unchanged",
                  "expected_faults": len(jobs), "actual_faults": len(rows),
                  "raw_tf_edges": sorted(raw_edges), "raw_tf_graph": raw_graph, "results": rows}
        with result_path.open("x") as stream:
            json.dump(result, stream, indent=2)
        state["stage"] = "finished"
        state["job"] = None
        log("finished", status=status)

    def assess():
        job = state["job"]
        ctx = state["context"]
        with trace.open() as stream:
            observations = [r for r in csv.DictReader(stream)
                            if int(r["receive_steady_ns"]) >= state["start_ns"]]
        imu = [r for r in observations if r["topic"] == "/imu"]
        reasons = sorted({r["reason"] for r in observations})
        ok = False
        if job.startswith("delay"):
            # The boundary is applied to observed age, including actual delivery.
            # A nominal 100 ms hold can arrive >100 ms and must then be rejected.
            checks = []
            for r in imu:
                age = int(r["receive_ros_ns"]) - int(r["stamp_ns"])
                if age < 0:
                    continue
                expected = "NONE" if age <= 100_000_000 else "INPUT_STALE"
                checks.append(r["reason"] == expected)
            ok = len(checks) >= 20 and all(checks)
        elif job in ("future", "duplicate", "old"):
            expected = {"future": "FUTURE_STAMP", "duplicate": "DUPLICATE_STAMP", "old": "OUT_OF_ORDER"}[job]
            ok = state["sent"] and any(r["reason"] == expected for r in imu)
        elif job in ("tf_missing", "clock_pause"):
            expected = "TF_UNAVAILABLE" if job == "tf_missing" else "INPUT_STALE"
            ok = ctx is not None and ctx.health == 2 and ctx.reason == expected
        elif job in ("clock_reset", "duplicate_tf"):
            expected = "CLOCK_RESET" if job == "clock_reset" else "TF_UNAVAILABLE"
            ok = (ctx is not None and ctx.health == 2 and ctx.reason == expected and
                  ctx.odom_epoch != state["context_before"].odom_epoch and
                  time.monotonic() - state["last_output"] > 0.6 and
                  ctx.clock_epoch == state["context_before"].clock_epoch)
            if job == "duplicate_tf":
                required = {("map", "odom"), ("odom", "base_footprint"), ("base_link", "base_scan")}
                ok = ok and required.issubset(raw_edges)
        rows.append({"fault": job, "status": "PASS" if ok else "FAIL",
                     "observations": len(observations), "reasons": reasons,
                     "health_reason": ctx.reason if ctx else None})
        log("assessed", **rows[-1])

    def tick():
        now = time.monotonic()
        ctx = state["context"]
        if state["stage"] == "finished":
            return
        if state["stage"] == "injecting":
            job = state["job"]
            if job.startswith("delay"):
                delay = int(job[5:]) * 1_000_000
                while queue and state["clock"] - stamp(queue[0].header.stamp) >= delay:
                    pubs["imu"].publish(queue.popleft())
            if job == "duplicate_tf" and tf_latest and not state["sent"]:
                rogue.publish(tf_latest)
                state["sent"] = True
                log("duplicate_tf", parent="odom", child="base_footprint")
            if now - state["start"] >= 1.25:
                assess()
                queue.clear()
                state["stage"] = "recovering"
                state["job"] = None
                state["start"] = now
                state["index"] += 1
                if state["index"] == len(jobs):
                    finish()
            return
        if not (run / "runtime/p2-hold-start.json").exists():
            return
        if ctx is not None and ctx.health == 0 and (state["stage"] == "waiting" or now - state["start"] >= 2.1):
            if not raw_graph:
                for topic in ("/tf", "/tf_static"):
                    raw_graph[topic] = [
                        {"node": endpoint.node_name, "namespace": endpoint.node_namespace,
                         "gid": list(endpoint.endpoint_gid)}
                        for endpoint in node.get_publishers_info_by_topic(topic)]
            state.update(stage="injecting", job=jobs[state["index"]], start=now,
                         start_ns=time.monotonic_ns(), sent=False, context_before=deepcopy(ctx))
            log("inject", fault=state["job"])
        elif state["stage"] == "recovering" and now - state["start"] > 10:
            rows.append({"fault": "recovery", "status": "FAIL", "reason": "not healthy within 10 s"})
            finish()

    node.create_timer(0.01, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not result_path.exists():
            rows.append({"fault": "suite", "status": "FAIL", "reason": "stopped before all faults"})
            finish()
        events.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
