#!/usr/bin/env python3
"""Execute the complete P2-01 case through existing CTest, pytest, or Runner entry points."""
if __package__ in (None, ""):
    from pathlib import Path
    from runpy import run_path
    run_path(str(Path(__file__).resolve().parents[1] / "__init__.py"))["bootstrap"]()

import argparse
from hashlib import sha256
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
from uuid import uuid4

from tools import PROJECT_ROOT
from tools.delivery.profile import prepare
from tools.p2.runner_profile import verified_file
from tools.p2.provenance import source_fingerprint, source_inventory
from tools.p2.p1_fresh_ctest import prepare_ctest

UNIT = {"P201." + name for name in (
    "TimeBoundary", "CovarianceAvailability", "FrameTwist", "RandomTimestamp")}
ROS = {"P201.HealthRecovery", "P201.ResetRace"}
SIM = {
    "P201.Static": (["stationary"] * 3, 60, 360),
    "P201.Straight": (["straight"] * 5, 0, 600),
    "P201.Rotate": (["rotate_left"] * 5 + ["rotate_right"] * 5, 0, 900),
    "P201.Resource": (["stationary"] * 3, 600, 2400),
    "P201.TFOwnership": (["delivery"] * 3, 60, 900),
    "P201.FaultTime": (["delivery"] * 3, 60, 900),
}
CASES = UNIT | ROS | set(SIM) | {
    "P201.DeterministicAudit", "P201.P1Regression", "P201.HardwareReadOnly"}


def save(path, data):
    with path.open("x") as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write("\n")


def command(argv, log, env=None):
    with log.open("x") as stream:
        stream.write(json.dumps({"argv": argv, "cwd": str(PROJECT_ROOT)}) + "\n")
        stream.flush()
        return subprocess.run(argv, cwd=PROJECT_ROOT, env=env,
                              stdout=stream, stderr=subprocess.STDOUT).returncode


def run_trial(item, out, p2_spec=None):
    argv = [sys.executable, "tools/run_trial.py", "--scenario", item["scenario"],
            "--method", item["method"], "--profile", item["profile"],
            "--run-id", item["run_id"], "--mission"]
    if p2_spec:
        argv += ["--p2-profile", str(p2_spec)]
    preflight = command(argv + ["--check"], out / "check.log")
    rc = command(argv, out / "runner.log") if preflight == 0 else preflight
    run = PROJECT_ROOT / "evidence/missions" / item["run_id"]
    result = {"run": str(run), "runner_exit": rc, "status": "PASS" if rc == 0 else "FAIL"}
    if run.exists():
        (out / "run").symlink_to(run, target_is_directory=True)
    return result


def simulation(case, manifest, out):
    model = verified_file(manifest["model"])
    variants, hold, limit = SIM[case]
    results = []
    for index, maneuver in enumerate(variants, 1):
        root = out / f"repetition-{index:03}"
        root.mkdir()
        plan = prepare(str(verified_file(manifest["base_profile"])),
                       manifest["install_prefix"], str(root / "plan"), uuid4().hex[:12])
        item = next(v for v in plan if v["case"] == "G01D")
        spec = {"schema": "cmr.p2/1.1", "run_id": item["run_id"],
                "p1_profile_sha256": sha256(Path(item["profile"]).read_bytes()).hexdigest(),
                "mode": manifest.get("estimation_mode", "shadow"), "hold_sim_seconds": hold,
                "maneuver": maneuver, "model": manifest["model"],
                "qualification": manifest.get("qualification")}
        if case in ("P201.TFOwnership", "P201.FaultTime"):
            spec["fault_test"] = case
        save(root / "p2-profile.json", spec)
        entry = run_trial(item, root, root / "p2-profile.json")
        run = Path(entry["run"])
        try:
            if case in ("P201.TFOwnership", "P201.FaultTime"):
                report = json.loads((run / "runtime/p2-fault-result.json").read_text())
                entry["evaluation"] = report
                entry["status"] = report["status"] if entry["runner_exit"] == 0 else "FAIL"
            else:
                rc = command([sys.executable, "tools/p2/evaluate_estimation.py", "compare",
                              "--run", str(run), "--output", str(root / "evaluation"),
                              "--case", case], root / "evaluation.log")
                entry["evaluation_exit"] = rc
                entry["status"] = "PASS" if rc == 0 else "FAIL"
        except Exception as exc:
            entry.update(status="FAIL", error=str(exc))
        results.append(entry)
        save(root / "result.json", entry)
        print(json.dumps({"case": case, "repetition": index, **entry}), flush=True)
        # Runner's lock and graph checks own recovery. Never clean up another run.
    return results, limit


def p1_regression(manifest, out):
    rows = []
    build = Path(manifest["p1_build"])
    for package in ("cmr_core", "cmr_execution", "cmr_mission", "cmr_sim"):
        test_root = out / (package + "-ctest")
        compile_count = prepare_ctest(build / package, test_root)
        code = command(["ctest", "--test-dir", str(test_root), "--output-on-failure",
                        "--no-tests=error"], out / (package + ".log"))
        rows.append({"case": package, "status": "PASS" if code == 0 else "FAIL",
                     "exit": code, "registered_tests": compile_count})
    code = command([sys.executable, "-B", "-m", "pytest", "-p", "no:cacheprovider",
                    "tests", "-q"], out / "pytest.log")
    rows.append({"case": "python-regression", "status": "PASS" if code == 0 else "FAIL", "exit": code})
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID="72", ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    env.pop("ROS_AUTOMATIC_DISCOVERY_RANGE", None)
    suite = str(verified_file(manifest["execution_suite"]))
    for family, count in (("E", 16), ("D", 18)):
        for n in range(1, count + 1):
            case = f"{family}{n:02}"
            script = "run_execution_tests.py" if family == "E" else "run_delivery_tests.py"
            argv = [sys.executable, "tools/mission/" + script, "--install-prefix",
                    manifest["install_prefix"], "--output-dir", str(out / case), "--cases", case]
            if family == "E":
                argv += ["--suite", suite]
            check = command(argv + ["--check"], out / (case + "-check.log"), env)
            code = command(argv, out / (case + ".log"), env) if check == 0 else check
            rows.append({"case": case, "status": "PASS" if code == 0 else "FAIL", "exit": code})
    plan = prepare(str(verified_file(manifest["base_profile"])), manifest["install_prefix"],
                   str(out / "l2-plan"), uuid4().hex[:12])
    for case in ("G01D", "G02", "G03", "G04", "G05", "G06"):
        item = next(v for v in plan if v["case"] == case)
        root = out / case
        root.mkdir()
        rows.append({"case": case, **run_trial(item, root)})
    return rows, 7200


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", required=True, choices=["P2-01"])
    parser.add_argument("--case", required=True, choices=sorted(CASES))
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--check", action="store_true", help="validate only; creates no output")
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    if (manifest.get("stage") != args.stage or manifest.get("schema") != "cmr.p2/1.1" or
            manifest["head"] != subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()):
        parser.error("manifest stage, schema or HEAD mismatch")
    if args.output.exists():
        parser.error("output already exists")
    if args.case in SIM or args.case == "P201.P1Regression":
        if manifest.get("simulation_authorized") is not True or not manifest.get("p1_qualification"):
            parser.error("manifest must record simulation authorization and P1 qualification")
        verified_file(manifest["base_profile"])
        if args.case in SIM:
            verified_file(manifest["model"])
    if args.check:
        print(json.dumps({"status": "CHECKED_ONLY", "case": args.case}))
        return 0
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    save(out / "manifest.json", manifest)
    code_hash = source_fingerprint()
    save(out / "source-inventory.json", source_inventory())
    start = time.monotonic()
    limit = None
    rows = []
    try:
        if args.case in SIM:
            rows, limit = simulation(args.case, manifest, out)
        elif args.case == "P201.P1Regression":
            rows, limit = p1_regression(manifest, out)
        elif args.case == "P201.HardwareReadOnly":
            rows = [{"status": "BLOCKED", "reason": "No connected qualified hardware; no motion requested"}]
        else:
            if args.case in UNIT:
                argv = ["ctest", "--test-dir", manifest["core_build"], "--no-tests=error",
                        "-R", "^" + args.case.replace(".", r"\.") + "$", "--output-on-failure"]
            else:
                file = "test_estimation_live.py" if args.case in ROS else "test_estimation_protocol.py"
                argv = [sys.executable, "-m", "pytest", "-o", "addopts=", "-m", "",
                        "tests/p2/" + file, "--case", args.case, "--manifest", str(args.manifest.resolve())]
            rc = command(argv, out / "test.log")
            rows = [{"status": "PASS" if rc == 0 else "FAIL", "exit": rc}]
    except Exception:
        (out / "exception.log").write_text(traceback.format_exc())
        rows.append({"status": "FAIL", "reason": "dispatcher exception; see exception.log"})
    elapsed = time.monotonic() - start
    if source_fingerprint() != code_hash:
        rows.append({"status": "FAIL", "reason": "SOURCE_CHANGED_DURING_CASE"})
    if not rows:
        rows = [{"status": "FAIL", "reason": "zero tests executed"}]
    if limit and elapsed > limit:
        rows.append({"status": "FAIL", "reason": "TIMEOUT", "limit_s": limit, "actual_s": elapsed})
    status = "PASS" if all(row["status"] == "PASS" for row in rows) else (
        "FAIL" if any(row["status"] == "FAIL" for row in rows) else "BLOCKED")
    report = {"schema": "cmr.p2/1.1", "head": manifest["head"], "case": args.case,
              "status": status, "code_sha256": code_hash, "model": manifest.get("model"),
              "elapsed_s": elapsed, "results": rows}
    save(out / ("p1-regression-result.json" if args.case == "P201.P1Regression" else "result.json"), report)
    print(json.dumps(report), flush=True)
    return 0 if status == "PASS" else 2


if __name__ == "__main__":
    sys.exit(main())
