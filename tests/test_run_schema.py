"""P0-02 synthetic record-contract oracles; no robot experiments are run."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests" / "fixtures"
TOOL = ROOT / "tools" / "validate_run.py"
module_spec = importlib.util.spec_from_file_location("cmr_validate_run", TOOL)
validator = importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(validator)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    # Trusted fixture editing only; deliberately independent of load_strict.
    return (yaml.safe_load(path.read_text()) if path.suffix in (".yaml", ".yml")
            else json.loads(path.read_text()))


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def snapshot(directory):
    return {str(p.relative_to(directory)): sha(p) for p in directory.rglob("*") if p.is_file()}


def copy_run(tmp_path, name="succeeded_unknown"):
    target = tmp_path / name
    shutil.copytree(FIXTURES / "runs" / name, target)
    return target


def edit_result(run, change):
    path = run / "result.json"
    data = read(path)
    change(data)
    write(path, data)


def edit_manifest(run, change):
    path = run / "manifest.json"
    data = read(path)
    change(data)
    write(path, data)
    if (run / "result.json").exists():
        edit_result(run, lambda d: d.update(manifest_sha256=sha(path)))


def scenario_copy(tmp_path):
    path = tmp_path / "scenario.yaml"
    shutil.copyfile(ROOT / "experiments/scenarios/official_static.yaml", path)
    return path


def expect(report, status="VALID", code=None, field=None):
    assert report["status"] == status, report
    assert report["validation_scope"] == "record_contract"
    if code is not None:
        assert any(e["code"] == code for e in report["errors"]), report
    if field is not None:
        assert any(field in e["path"] for e in report["errors"]), report


def run_report(run):
    return validator.validate_run_dir(run, allow_fixtures=True)


def cli(*args, cwd):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    process = subprocess.run([sys.executable, str(TOOL), *map(str, args)], cwd=cwd,
                             env=env, text=True, capture_output=True, check=False)
    assert not process.stderr, process.stderr
    report = json.loads(process.stdout)  # Extra output or tracebacks are rejected.
    assert isinstance(report, dict)
    return process.returncode, report


def test_S01_official_draft():
    path = ROOT / "experiments/scenarios/official_static.yaml"
    assert read(path)["task"]["initial_pose"] is None
    expect(validator.validate_scenario(path))
    expect(validator.validate_scenario(path, require_ready=True), "NOT_READY", "SCENARIO_NOT_READY")


@pytest.mark.parametrize("filename,master", [("crossing_draft.yaml", "S1"), ("observed_occlusion_draft.yaml", "S3")])
def test_S02_dynamic_draft(tmp_path, filename, master):
    path = tmp_path / filename
    shutil.copyfile(FIXTURES / "scenarios" / filename, path)
    assert read(path)["master_scenario_id"] == master
    expect(validator.validate_scenario(path))
    data = read(path)
    data["master_scenario_id"] = "S2"
    write(path, data)
    expect(validator.validate_scenario(path), "INVALID", "SCENARIO_MAPPING", "master_scenario_id")


@pytest.mark.parametrize("change,field", [
    (lambda d: d.pop("schema_version"), "schema_version"),
    (lambda d: d.update(scenario_id=""), "scenario_id"),
    (lambda d: d.update(scenario_id="   "), "scenario_id"),
    (lambda d: d.update(typo=1), "typo"),
    (lambda d: d.update(kind="invented"), "kind"),
    (lambda d: d.update(schema_version=True), "schema_version"),
    (lambda d: d.update(schema_version=1.0), "schema_version"),
    (lambda d: d["task"].update(typo=1), "task.typo"),
])
def test_S03_structure(tmp_path, change, field):
    path = scenario_copy(tmp_path)
    data = read(path)
    change(data)
    write(path, data)
    expect(validator.validate_scenario(path), "INVALID", "SCHEMA_ERROR", field)


def test_S03_resource_uri_not_implemented(tmp_path):
    path = scenario_copy(tmp_path)
    data = read(path)
    data["resources"]["world"] = {"path": "custom://synthetic/world", "sha256": None, "package": None, "package_version": None}
    write(path, data)
    expect(validator.validate_scenario(path), "INVALID", "RESOURCE_PATH", "resources.world.path")


def synthetic_ready(tmp_path):
    """Construct an independent ready record with explicitly synthetic files."""
    path = scenario_copy(tmp_path)
    data = read(path)
    resources = []
    for name in ("world", "map_yaml", "map_image", "robot", "constraints", "review"):
        p = tmp_path / (name + ".txt")
        p.write_text("SYNTHETIC " + name + "; no physical preparation was verified.\n")
        resources.append({"path": str(p), "sha256": sha(p)})
    lock = tmp_path / "dependencies.json"
    write(lock, {"resources": resources[:4], "unresolved": []})
    def resource(p):
        return {"path": str(p), "sha256": sha(p), "package": None, "package_version": None}
    for key in ("world", "map_yaml", "map_image", "robot"):
        data["resources"][key] = resource(tmp_path / (key + ".txt"))
    data["resources"]["dependency_lock"] = resource(lock)
    data["task"]["common_constraints_ref"] = resource(tmp_path / "constraints.txt")
    data["task"]["initial_pose"] = {"frame_id": "synthetic_map", "x_m": 0, "y_m": 0, "yaw_rad": 0}
    data["task"]["goal_pose"] = {"frame_id": "synthetic_map", "x_m": 1, "y_m": 0, "yaw_rad": 0}
    for key in ("resource_dependencies", "world_map_alignment", "poses_free", "interfaces_observed"):
        data["verification"][key] = "checked"
    data["verification"]["evidence_refs"] = [str(tmp_path / "review.txt")]
    data["status"] = "ready_for_manual_smoke"
    write(path, data)
    return path


@pytest.mark.parametrize("change", [
    lambda d: d["task"].update(initial_pose=None),
    lambda d: d["resources"].update(dependency_lock=None),
    lambda d: d["verification"].update(evidence_refs=[]),
    lambda d: d["verification"].update(poses_free="pending"),
])
def test_S04_false_ready(tmp_path, change):
    path = synthetic_ready(tmp_path)
    data = read(path)
    change(data)
    write(path, data)
    expect(validator.validate_scenario(path), "INVALID", "READY_REQUIREMENTS", "status")


def test_S05_synthetic_ready(tmp_path):
    path = synthetic_ready(tmp_path)
    expect(validator.validate_scenario(path, require_ready=True))
    # Removing one primary resource from a correctly hashed lock is not ready.
    lock = tmp_path / "dependencies.json"
    data = read(lock)
    data["resources"].pop()
    write(lock, data)
    spec = read(path)
    spec["resources"]["dependency_lock"]["sha256"] = sha(lock)
    write(path, spec)
    expect(validator.validate_scenario(path), "INVALID", "DEPENDENCY_LOCK", "resources.robot")


@pytest.mark.parametrize("variant,code", [("unresolved", "DEPENDENCY_LOCK"), ("bad_hash", "HASH_MISMATCH"), ("empty_evidence", "ARTIFACT_INVALID")])
def test_S05_ready_evidence_integrity(tmp_path, variant, code):
    path = synthetic_ready(tmp_path)
    if variant == "empty_evidence":
        (tmp_path / "review.txt").write_bytes(b"")
    else:
        lock = tmp_path / "dependencies.json"
        if variant == "unresolved":
            data = read(lock)
            data["unresolved"] = ["synthetic missing model"]
            write(lock, data)
            spec = read(path)
            spec["resources"]["dependency_lock"]["sha256"] = sha(lock)
            write(path, spec)
        else:
            (tmp_path / "world.txt").write_text("SYNTHETIC changed resource\n")
    expect(validator.validate_scenario(path), "INVALID", code)


def test_R01_success_unknown():
    run = FIXTURES / "runs/succeeded_unknown"
    expect(run_report(run))
    result = read(run / "result.json")
    assert result["metrics"]["action_success"] is True
    assert result["evaluation"]["collision"] is None
    assert result["metrics"]["delivery_success"] is None


@pytest.mark.parametrize("action,success", [("CANCELED", False), ("UNKNOWN", None)])
def test_R02_timeout(tmp_path, action, success):
    run = copy_run(tmp_path, "nav_timeout")
    def change(d):
        d["status"]["action"] = action
        d["metrics"]["action_success"] = success
    edit_result(run, change)
    expect(run_report(run))
    assert read(run / "result.json")["status"]["harness"] == "NAV_TIMEOUT"


@pytest.mark.parametrize("runtime", [None, {"actual_plugin": None, "parameters_ref": None}])
def test_R03_startup_failure(tmp_path, runtime):
    run = copy_run(tmp_path, "startup_failed")
    edit_result(run, lambda d: d.update(runtime=runtime))
    expect(run_report(run))
    result = read(run / "result.json")
    assert result["status"]["action"] == "NOT_SENT"
    assert result["metrics"]["action_success"] is None


def test_R04_incomplete_readonly(tmp_path):
    run = copy_run(tmp_path, "incomplete")
    before = snapshot(run)
    expect(run_report(run), "INCOMPLETE", "MISSING_RESULT", "result.json")
    assert snapshot(run) == before
    assert not (run / "result.json").exists()


@pytest.mark.parametrize("variant", ["malformed", "id", "origin", "hash"])
def test_R05_bad_existing_result(tmp_path, variant):
    run = copy_run(tmp_path)
    if variant == "malformed":
        (run / "result.json").write_text('{"broken":')
        code, field = "PARSE_ERROR", "result.json"
    else:
        key, value = {"id": ("run_id", "another_run"), "origin": ("record_origin", "real"), "hash": ("manifest_sha256", "0" * 64)}[variant]
        edit_result(run, lambda d: d.update({key: value}))
        code, field = "IDENTITY_MISMATCH", key
    expect(run_report(run), "INVALID", code, field)


@pytest.mark.parametrize("variant", ["actual", "expected", "method", "parameters"])
def test_R06_runtime_identity(tmp_path, variant):
    run = copy_run(tmp_path)
    if variant == "actual":
        edit_result(run, lambda d: d["runtime"].update(actual_plugin="dwb_core::DWBLocalPlanner"))
    elif variant == "expected":
        edit_manifest(run, lambda d: d["baseline"].update(expected_plugin="wrong_plugin"))
    elif variant == "method":
        edit_manifest(run, lambda d: d["baseline"].update(method="dwb"))
    else:
        edit_result(run, lambda d: d["runtime"].update(parameters_ref=None))
    expect(run_report(run), "INVALID", "ARTIFACT_REFERENCE" if variant == "parameters" else "PLUGIN_MISMATCH")


@pytest.mark.parametrize("action,success", [("SUCCEEDED", True), ("REJECTED", False), ("ABORTED", False), ("CANCELED", False), ("NOT_SENT", None), ("UNKNOWN", None)])
def test_R07_terminal_mapping(tmp_path, action, success):
    run = copy_run(tmp_path)
    def correct(d):
        d["status"]["action"] = action
        d["metrics"]["action_success"] = success
    edit_result(run, correct)
    expect(run_report(run))
    edit_result(run, lambda d: d["metrics"].update(action_success=False if success is not False else True))
    expect(run_report(run), "INVALID", "ACTION_MISMATCH", "action_success")


def test_R07_cleanup_keeps_terminal(tmp_path):
    run = copy_run(tmp_path)
    edit_result(run, lambda d: d["status"].update(cleanup="FAILED", primary_reason="Synthetic cleanup deadline"))
    before = (run / "result.json").read_bytes()
    expect(run_report(run))
    assert (run / "result.json").read_bytes() == before
    assert read(run / "result.json")["status"]["action"] == "SUCCEEDED"
    edit_result(run, lambda d: d["status"].update(primary_reason=None))
    expect(run_report(run), "INVALID", "REASON_REQUIRED", "primary_reason")


def test_R07_success_is_not_nav_timeout(tmp_path):
    run = copy_run(tmp_path)
    edit_result(run, lambda d: d["status"].update(harness="NAV_TIMEOUT", primary_reason="Synthetic timeout"))
    expect(run_report(run), "INVALID", "STATUS_INCONSISTENT", "harness")


@pytest.mark.parametrize("variant,code", [
    ("dirty_missing", "SOURCE_SNAPSHOT_REQUIRED"), ("missing_ref", "ARTIFACT_REFERENCE"),
    ("hash", "HASH_MISMATCH"), ("escape", "ARTIFACT_PATH"), ("absolute", "ARTIFACT_PATH"),
    ("symlink", "ARTIFACT_PATH"), ("duplicate_id", "DUPLICATE_ID"),
    ("wrong_scenario", "IDENTITY_MISMATCH"),
])
def test_R08_artifact_integrity(tmp_path, variant, code):
    run = copy_run(tmp_path)
    if variant == "dirty_missing":
        edit_manifest(run, lambda d: d["source"].update(dirty=True))
    elif variant == "missing_ref":
        edit_manifest(run, lambda d: d["source"].update(snapshot_ref="absent"))
    elif variant == "hash":
        (run / "artifacts/config.txt").write_text("SYNTHETIC changed bytes\n")
    elif variant in ("escape", "absolute"):
        edit_manifest(run, lambda d: d["artifacts"][0].update(path="../outside.yaml" if variant == "escape" else "/tmp/outside.yaml"))
    elif variant == "symlink":
        outside = tmp_path / "outside.yaml"
        outside.write_text("SYNTHETIC outside file\n")
        (run / "artifacts/scenario.yaml").unlink()
        (run / "artifacts/scenario.yaml").symlink_to(outside)
    elif variant == "duplicate_id":
        edit_manifest(run, lambda d: d["artifacts"][1].update(id="scenario"))
    else:
        edit_manifest(run, lambda d: d["scenario"].update(scenario_revision="another_revision"))
    expect(run_report(run), "INVALID", code)


def test_R08_dirty_snapshot_present(tmp_path):
    run = copy_run(tmp_path)
    patch = run / "artifacts/source.patch"
    patch.write_text("SYNTHETIC modification snapshot\n--- a/synthetic.txt\n+++ b/synthetic.txt\n@@ -1 +1 @@\n-before\n+after\n")
    def change(d):
        d["source"].update(dirty=True, snapshot_ref="source")
        d["artifacts"].append({"id": "source", "path": "artifacts/source.patch", "sha256": sha(patch), "kind": "source_snapshot"})
    edit_manifest(run, change)
    expect(run_report(run))


def test_R08_archived_scenario_does_not_access_other_host(tmp_path):
    run = copy_run(tmp_path)
    path = run / "artifacts/scenario.yaml"
    spec = read(path)
    spec["resources"]["world"] = {"path": "/synthetic_other_host/missing.world", "sha256": "0" * 64, "package": None, "package_version": None}
    write(path, spec)
    def change(d):
        d["scenario"]["spec_sha256"] = sha(path)
        d["artifacts"][0]["sha256"] = sha(path)
    edit_manifest(run, change)
    expect(run_report(run))


@pytest.mark.parametrize("record,field,value", [
    ("manifest", "schema_version", True), ("result", "schema_version", True),
    ("manifest", "typo", 1), ("result", "typo", 1),
])
def test_R05_record_structure(tmp_path, record, field, value):
    run = copy_run(tmp_path)
    if record == "manifest":
        edit_manifest(run, lambda d: d.update({field: value}))
    else:
        edit_result(run, lambda d: d.update({field: value}))
    expect(run_report(run), "INVALID", "SCHEMA_ERROR", field)


def test_E01_unknown_preserved(tmp_path):
    run = copy_run(tmp_path)
    before = snapshot(run)
    expect(run_report(run))
    assert read(run / "result.json")["evaluation"]["collision"] is None
    assert snapshot(run) == before


def test_E02_false_without_evidence(tmp_path):
    run = copy_run(tmp_path)
    edit_result(run, lambda d: d["evaluation"].update(collision=False))
    expect(run_report(run), "INVALID", "COLLISION_UNSUPPORTED", "collision")
    expect(run_report(FIXTURES / "runs/invalid_collision_false"), "INVALID", "COLLISION_UNSUPPORTED")


def test_E03_observed_partial():
    run = FIXTURES / "runs/collision_observed_partial"
    expect(run_report(run))
    data = read(run / "result.json")["evaluation"]
    assert data["collision"] is True and data["coverage"] == "partial"
    assert data["evaluation_valid"] is False


def test_E04_full_scoped_false():
    run = FIXTURES / "runs/collision_false_scoped"
    report = run_report(run)
    expect(report)
    assert report["record_origin"] == "fixture"
    assert any("Synthetic" in warning for warning in report["warnings"])
    data = read(run / "result.json")["evaluation"]
    assert data["collision"] is False and data["coverage"] == "full"


@pytest.mark.parametrize("variant,code", [("partial", "COLLISION_UNSUPPORTED"), ("no_objects", "COLLISION_UNSUPPORTED"), ("missing_ref", "ARTIFACT_REFERENCE"), ("outside_interval", "TIME_INVALID")])
def test_E04_false_requires_full_scope(tmp_path, variant, code):
    run = copy_run(tmp_path, "collision_false_scoped")
    def change(d):
        e = d["evaluation"]
        if variant == "partial": e["coverage"] = "partial"
        if variant == "no_objects": e["objects"] = []
        if variant == "missing_ref": e["evidence_refs"] = ["absent"]
        if variant == "outside_interval": e["interval_s"]["end"] = 21
    edit_result(run, change)
    expect(run_report(run), "INVALID", code)


@pytest.mark.parametrize("clearance", [0.2, -0.02])
def test_E05_geometry_does_not_infer_collision(tmp_path, clearance):
    run = copy_run(tmp_path, "collision_false_scoped")
    edit_result(run, lambda d: d["evaluation"].update(collision=None, geometric_contact=False, min_signed_clearance_m=clearance))
    expect(run_report(run))
    data = read(run / "result.json")["evaluation"]
    assert data["collision"] is None and data["geometric_contact"] is False
    assert data["min_signed_clearance_m"] == clearance


@pytest.mark.parametrize("variant,code", [("epoch", "EPOCH_MISMATCH"), ("reversed", "TIME_INVALID"), ("outside", "TIME_INVALID"), ("object", "EVALUATION_INVALID"), ("source", "ARTIFACT_REFERENCE")])
def test_E03_event_boundaries(tmp_path, variant, code):
    run = copy_run(tmp_path, "collision_observed_partial")
    def change(d):
        e = d["evaluation"]
        if variant == "epoch": e["clock_epoch"] = "other_epoch"
        if variant == "reversed": e["interval_s"] = {"start": 15, "end": 11}
        if variant == "outside": e["collision_event"]["at_s"] = 21
        if variant == "object": e["collision_event"]["objects"] = ["outside_scope"]
        if variant == "source": e["collision_event"]["source_ref"] = "missing"
    edit_result(run, change)
    expect(run_report(run), "INVALID", code)


@pytest.mark.parametrize("variant,code,field", [("epoch", "EPOCH_MISMATCH", "clock_epoch"), ("start", "TIME_INVALID", "sim_start_s"), ("end", "TIME_INVALID", "sim_end_s"), ("elapsed", "SCHEMA_ERROR", "elapsed_monotonic_s"), ("utc", "UTC_INVALID", "started_at_utc")])
def test_T01_time_domains(tmp_path, variant, code, field):
    run = copy_run(tmp_path)
    if variant == "utc":
        edit_manifest(run, lambda d: d["clock"].update(started_at_utc="2026-02-30T00:00:00Z"))
    else:
        key, value = {"epoch": ("clock_epoch", "other_epoch"), "start": ("sim_start_s", 11), "end": ("sim_end_s", 9), "elapsed": ("elapsed_monotonic_s", -1)}[variant]
        edit_result(run, lambda d: d["time"].update({key: value}))
    expect(run_report(run), "INVALID", code, field)


@pytest.mark.parametrize("variant", ["null", "valid", "zero", "ratio", "window", "missing_sim"])
def test_T02_rtf_window(tmp_path, variant):
    run = copy_run(tmp_path)
    def change(d):
        if variant == "null": d["time"]["rtf"] = None
        if variant == "zero": d["time"]["rtf"]["monotonic_delta_s"] = 0
        if variant == "ratio": d["time"]["rtf"]["value"] = 0.75
        if variant == "window": d["time"]["rtf"] = {"value": .5, "sim_delta_s": 5, "monotonic_delta_s": 10}
        if variant == "missing_sim": d["time"]["sim_end_s"] = None
    edit_result(run, change)
    if variant in ("null", "valid"):
        expect(run_report(run))
    else:
        expect(run_report(run), "INVALID", "SCHEMA_ERROR" if variant == "zero" else "RTF_INVALID", "rtf")


@pytest.mark.parametrize("token", ["NaN", "Infinity", "-Infinity", "1e309", "-1e309"])
def test_P01_json_nonfinite(tmp_path, token):
    original = scenario_copy(tmp_path)
    data = read(original)
    path = original.with_suffix(".json")
    data["task"]["initial_pose"] = {"frame_id": "map", "x_m": "SENTINEL", "y_m": 0, "yaw_rad": 0}
    path.write_text(json.dumps(data).replace('"SENTINEL"', token))
    expect(validator.validate_scenario(path), "INVALID", "NONFINITE_NUMBER" if "e309" in token else "PARSE_ERROR")


@pytest.mark.parametrize("token", [".nan", ".inf", "-.inf", "1e309", "1.0e+309"])
def test_P01_yaml_nonfinite(tmp_path, token):
    path = scenario_copy(tmp_path)
    data = read(path)
    data["task"]["initial_pose"] = {"frame_id": "map", "x_m": "SENTINEL", "y_m": 0, "yaw_rad": 0}
    path.write_text(yaml.safe_dump(data).replace("SENTINEL", token))
    expect(validator.validate_scenario(path), "INVALID", "NONFINITE_NUMBER", "initial_pose.x_m")


@pytest.mark.parametrize("variant", ["json_duplicate", "yaml_duplicate", "nonstring", "merge", "unsafe_tag", "recursive_alias"])
def test_P02_strict_parsing(tmp_path, variant):
    path = scenario_copy(tmp_path)
    data = read(path)
    if variant == "json_duplicate":
        path = path.with_suffix('.json')
        path.write_text(json.dumps(data).replace('"schema_version": 1', '"schema_version": 1, "schema_version": 1'))
    else:
        text = yaml.safe_dump(data)
        suffix = {"yaml_duplicate": "\nscenario_id: changed\n", "nonstring": "\n1: value\n", "merge": "\nextra: &x {a: 1}\nmerged:\n  <<: *x\n", "unsafe_tag": "\nextra: !!python/object:builtins.object {}\n", "recursive_alias": "\nextra: &x [*x]\n"}[variant]
        path.write_text(text + suffix)
    expect(validator.validate_scenario(path), "INVALID", "PARSE_ERROR")


@pytest.mark.parametrize("field,value", [("elapsed_monotonic_s", True), ("sim_end_s", False)])
def test_P03_bool_is_not_number(tmp_path, field, value):
    run = copy_run(tmp_path)
    edit_result(run, lambda d: d["time"].update({field: value}))
    expect(run_report(run), "INVALID", "SCHEMA_ERROR", field)


def test_P03_string_false_and_null(tmp_path):
    run = copy_run(tmp_path)
    before = snapshot(run)
    expect(run_report(run))
    assert read(run / "result.json")["metrics"]["goal_position_error_m"] is None
    assert snapshot(run) == before
    edit_result(run, lambda d: d["metrics"].update(stop_observed="false"))
    expect(run_report(run), "INVALID", "SCHEMA_ERROR", "stop_observed")


def test_C01_fixture_default_rejected(tmp_path):
    run = copy_run(tmp_path)
    code, report = cli("run", run, cwd=tmp_path)
    assert code == 2
    expect(report, "INVALID", "FIXTURE_NOT_ALLOWED", "record_origin")
    code, report = cli("run", run, "--allow-fixtures", cwd=tmp_path)
    assert code == 0
    expect(report)
    bad = copy_run(tmp_path, "invalid_collision_false")
    code, report = cli("run", bad, "--allow-fixtures", cwd=tmp_path)
    assert code == 2
    expect(report, "INVALID", "COLLISION_UNSUPPORTED")


@pytest.mark.parametrize("name,status,exitcode", [("succeeded_unknown", "VALID", 0), ("nav_timeout", "VALID", 0), ("startup_failed", "VALID", 0), ("collision_observed_partial", "VALID", 0), ("collision_false_scoped", "VALID", 0), ("incomplete", "INCOMPLETE", 3), ("invalid_collision_false", "INVALID", 2)])
def test_C02_external_cwd_readonly(tmp_path, name, status, exitcode):
    run = copy_run(tmp_path, name)
    before = snapshot(run)
    external = tmp_path / "external_cwd"
    external.mkdir()
    code, report = cli("run", run, "--allow-fixtures", cwd=external)
    assert code == exitcode
    expect(report, status)
    assert snapshot(run) == before
    if name == "incomplete":
        assert not (run / "result.json").exists()


def test_C02_scenario_exitcodes(tmp_path):
    path = scenario_copy(tmp_path)
    before = sha(path)
    code, report = cli("scenario", path, cwd=tmp_path)
    assert code == 0
    expect(report)
    code, report = cli("scenario", path, "--require-ready", cwd=tmp_path)
    assert code == 4
    expect(report, "NOT_READY", "SCENARIO_NOT_READY")
    assert sha(path) == before


@pytest.mark.parametrize("variant", ["missing_artifact", "empty_artifact", "missing_input", "bad_json"])
def test_C03_io_errors(tmp_path, variant):
    run = copy_run(tmp_path)
    if variant == "missing_artifact": (run / "artifacts/config.txt").unlink()
    if variant == "empty_artifact": (run / "artifacts/config.txt").write_bytes(b"")
    if variant == "missing_input": (run / "manifest.json").unlink()
    if variant == "bad_json": (run / "result.json").write_text("not json")
    code, report = cli("run", run, "--allow-fixtures", cwd=tmp_path)
    assert code == 2
    expect(report, "INVALID", "INPUT_IO" if variant == "missing_input" else "PARSE_ERROR" if variant == "bad_json" else "ARTIFACT_INVALID")
    assert report["errors"][0]["path"]


def test_C03_argument_and_internal_errors(tmp_path, monkeypatch, capsys):
    code, report = cli("unknown_command", cwd=tmp_path)
    assert code == 2
    expect(report, "INVALID", "CLI_ARGUMENT")
    def unexpected(*args, **kwargs):
        raise RuntimeError("synthetic unexpected tool failure")
    monkeypatch.setattr(validator, "validate_run_dir", unexpected)
    assert validator.main(["run", "unused"]) == 1
    expect(json.loads(capsys.readouterr().out), "INTERNAL_ERROR", "INTERNAL_ERROR")
