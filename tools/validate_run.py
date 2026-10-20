#!/usr/bin/env python3
"""Read-only ScenarioSpec/RunManifest/RunResult v1 record-contract validation."""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import sys

from jsonschema import Draft202012Validator, validators
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PLUGINS = {"mppi": "nav2_mppi_controller::MPPIController", "dwb": "dwb_core::DWBLocalPlanner"}
EXIT_CODES = {"VALID": 0, "INVALID": 2, "INCOMPLETE": 3, "NOT_READY": 4}


class InvalidRecord(ValueError):
    def __init__(self, code, path, message):
        self.error = {"code": code, "path": str(path), "message": str(message)}
        super().__init__(str(message))


def fail(code, path, message):
    raise InvalidRecord(code, path, message)


class StrictSafeLoader(yaml.SafeLoader):
    pass


def unique_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        if key_node.tag == "tag:yaml.org,2002:merge":
            raise ValueError("YAML merge key is forbidden")
        key = loader.construct_object(key_node, deep=True)
        if type(key) is not str:
            raise ValueError("Mapping key must be a string")
        if key in result:
            raise ValueError("Duplicate key: " + key)
        result[key] = loader.construct_object(value_node, deep=True)
    return result


StrictSafeLoader.add_constructor("tag:yaml.org,2002:map", unique_mapping)
# PyYAML 6's default resolver misses JSON-style exponents without a decimal
# point. Recognize them explicitly so 1e309 cannot hide as a string scalar.
StrictSafeLoader.add_implicit_resolver(
    "tag:yaml.org,2002:float",
    re.compile(r"^[-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)[eE][-+]?[0-9]+$"),
    list("-+0123456789."),
)


def unique_json_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate key: " + key)
        result[key] = value
    return result


def reject_constant(value):
    raise ValueError("Nonstandard JSON number: " + value)


def check_json_tree(value, path, active=None):
    """Reject non-JSON YAML types, nonfinite floats, and recursive aliases."""
    if active is None:
        active = set()
    if type(value) is float and not math.isfinite(value):
        fail("NONFINITE_NUMBER", path, "Numeric value must be finite")
    if value is None or type(value) in (str, bool, int, float):
        return
    if type(value) not in (list, dict):
        fail("PARSE_ERROR", path, "Only JSON-expressible types are allowed")
    if id(value) in active:
        fail("PARSE_ERROR", path, "Recursive YAML alias is forbidden")
    active.add(id(value))
    for key, item in (value.items() if type(value) is dict else enumerate(value)):
        if type(value) is dict and type(key) is not str:
            fail("PARSE_ERROR", path, "Mapping key must be a string")
        check_json_tree(item, f"{path}.{key}", active)
    active.remove(id(value))


def load_strict(path):
    path = Path(path)
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        fail("INPUT_IO", path, exc)
    try:
        if path.suffix.lower() in (".yaml", ".yml"):
            value = yaml.load(text, Loader=StrictSafeLoader)
        else:
            value = json.loads(text, object_pairs_hook=unique_json_object, parse_constant=reject_constant)
    except (ValueError, yaml.YAMLError, RecursionError) as exc:
        fail("PARSE_ERROR", path, exc)
    try:
        check_json_tree(value, f"{path}:$")
    except RecursionError:
        fail("PARSE_ERROR", path, "Record nesting exceeds supported parser depth")
    return value


# Draft 2020-12 normally regards 1.0 as integer. This v1 record contract uses
# actual integers, and never treats Python bool as integer or number.
TYPE_CHECKER = Draft202012Validator.TYPE_CHECKER.redefine_many({
    "integer": lambda checker, value: type(value) is int,
    "number": lambda checker, value: type(value) in (int, float),
})
RecordValidator = validators.extend(Draft202012Validator, type_checker=TYPE_CHECKER)


def validate_structure(value, kind, path):
    schema = load_strict(PROJECT_ROOT / "experiments" / "schemas" / f"{kind}.schema.json")

    def local_refs_only(node):
        if isinstance(node, dict):
            if "$ref" in node and not node["$ref"].startswith("#/$defs/"):
                raise RuntimeError("Schema contains nonlocal $ref")
            for item in node.values():
                local_refs_only(item)
        elif isinstance(node, list):
            for item in node:
                local_refs_only(item)

    local_refs_only(schema)  # No resolver or network access is needed.
    error = next(RecordValidator(schema).iter_errors(value), None)
    if error is not None:
        parts = list(error.absolute_path)
        if error.validator == "required" and type(error.instance) is dict:
            parts += [next(k for k in error.validator_value if k not in error.instance)]
        if error.validator == "additionalProperties" and type(error.instance) is dict:
            extras = set(error.instance) - set(error.schema.get("properties", {}))
            if extras:
                parts += [sorted(extras)[0]]
        fail("SCHEMA_ERROR", f"{path}:$." + ".".join(map(str, parts)), error.message)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def nonempty_file(path, location, expected=None):
    try:
        if not path.is_file() or path.stat().st_size == 0:
            fail("ARTIFACT_INVALID", location, "Referenced file must exist and be nonempty")
        if expected is not None and digest(path) != expected.lower():
            fail("HASH_MISMATCH", location, "SHA256 does not match file bytes")
    except InvalidRecord:
        raise
    except (OSError, ValueError) as exc:
        fail("INPUT_IO", location, exc)


def host_path(name):
    path = Path(name)
    return path if path.is_absolute() else PROJECT_ROOT / path


def check_resource(resource, location, check_host=True):
    if resource is None:
        return
    if "://" in resource["path"] or "\x00" in resource["path"]:
        fail("RESOURCE_PATH", location + ".path", "Resource path is a host file path, not a URI")
    if check_host and resource["sha256"] is not None:
        nonempty_file(host_path(resource["path"]), location, resource["sha256"])


def scenario_rules(spec, path, check_host_resources=True):
    where = f"{path}:$"
    mapping = {"static_smoke": "S0", "crossing": "S1", "head_on": "S2", "observed_occlusion": "S3"}
    if spec["master_scenario_id"] != mapping[spec["kind"]]:
        fail("SCENARIO_MAPPING", where + ".master_scenario_id", "kind does not match Master scenario")
    if spec["kind"] == "static_smoke" and spec["dynamic_events"]:
        fail("SCENARIO_MAPPING", where + ".dynamic_events", "Static scenario has no dynamic events")
    ids = [event["event_id"] for event in spec["dynamic_events"]]
    if len(ids) != len(set(ids)):
        fail("DUPLICATE_ID", where + ".dynamic_events", "event_id must be unique")
    for key, resource in spec["resources"].items():
        check_resource(resource, where + ".resources." + key, check_host_resources)
    check_resource(spec["task"]["common_constraints_ref"], where + ".task.common_constraints_ref", check_host_resources)
    for i, evidence in enumerate(spec["verification"]["evidence_refs"]):
        if "://" in evidence or "\x00" in evidence:
            fail("RESOURCE_PATH", where + f".verification.evidence_refs.{i}", "Preparation evidence must be a host file path")
        if check_host_resources:
            nonempty_file(host_path(evidence), where + f".verification.evidence_refs.{i}")
    if spec["status"] == "ready_for_manual_smoke":
        required_resources = list(spec["resources"].values()) + [spec["task"]["common_constraints_ref"]]
        verification = spec["verification"]
        if (spec["kind"] != "static_smoke" or
                any(x is None or x["sha256"] is None for x in required_resources) or
                any(spec["task"][x] is None for x in ("initial_pose", "goal_pose")) or
                any(verification[x] != "checked" for x in ("resource_dependencies", "world_map_alignment", "poses_free", "interfaces_observed")) or
                not verification["evidence_refs"]):
            fail("READY_REQUIREMENTS", where + ".status", "Claimed ready lacks resources, poses, constraints or checked evidence")
        if check_host_resources:
            # Explicit minimal dependency-lock encoding, documented in schema.md.
            lock = load_strict(host_path(spec["resources"]["dependency_lock"]["path"]))
            if (type(lock) is not dict or set(lock) != {"resources", "unresolved"} or
                    type(lock["resources"]) is not list or not lock["resources"] or lock["unresolved"] != []):
                fail("DEPENDENCY_LOCK", where + ".resources.dependency_lock", "Lock requires nonempty resources and empty unresolved list")
            locked = {}
            for item in lock["resources"]:
                if (type(item) is not dict or set(item) != {"path", "sha256"} or
                        type(item["path"]) is not str or not item["path"].strip() or
                        type(item["sha256"]) is not str or not re.fullmatch(r"[0-9a-fA-F]{64}", item["sha256"])):
                    fail("DEPENDENCY_LOCK", where + ".resources.dependency_lock", "Bad lock resource entry")
                resolved = host_path(item["path"]).resolve()
                if resolved in locked:
                    fail("DEPENDENCY_LOCK", where + ".resources.dependency_lock", "Duplicate resolved resource path")
                nonempty_file(resolved, where + ".resources.dependency_lock", item["sha256"])
                locked[resolved] = item["sha256"].lower()
            for key in ("world", "map_yaml", "map_image", "robot"):
                resource = spec["resources"][key]
                if locked.get(host_path(resource["path"]).resolve()) != resource["sha256"].lower():
                    fail("DEPENDENCY_LOCK", where + ".resources." + key, "Primary resource not covered by dependency lock")


def artifacts(record, directory, path):
    root = directory.resolve()
    refs = {}
    for i, artifact in enumerate(record["artifacts"]):
        where = f"{path}:$.artifacts.{i}"
        if artifact["id"] in refs:
            fail("DUPLICATE_ID", where + ".id", "Artifact id must be unique within its record")
        name = artifact["path"]
        posix = PurePosixPath(name)
        windows = PureWindowsPath(name)
        if posix.is_absolute() or windows.is_absolute() or windows.drive or ".." in posix.parts or "\\" in name or "\x00" in name:
            fail("ARTIFACT_PATH", where + ".path", "Only contained relative paths without traversal are allowed")
        try:
            target = (root / name).resolve()
        except (OSError, RuntimeError) as exc:
            fail("ARTIFACT_PATH", where + ".path", exc)
        if not target.is_relative_to(root):
            fail("ARTIFACT_PATH", where + ".path", "Resolved artifact escapes the run directory")
        nonempty_file(target, where + ".path", artifact["sha256"])
        refs[artifact["id"]] = (artifact, target)
    return refs


def require_ref(refs, ref_id, where, kind=None):
    if ref_id not in refs or (kind is not None and refs[ref_id][0]["kind"] != kind):
        fail("ARTIFACT_REFERENCE", where, "Missing artifact id or incompatible artifact kind")
    return refs[ref_id]


def manifest_rules(manifest, directory, path):
    where = f"{path}:$"
    stamp = manifest["clock"]["started_at_utc"]
    if not re.fullmatch(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?Z", stamp):
        fail("UTC_INVALID", where + ".clock.started_at_utc", "Expected actual UTC timestamp ending in Z")
    try:
        datetime.fromisoformat(stamp[:-1] + "+00:00")
    except ValueError as exc:
        fail("UTC_INVALID", where + ".clock.started_at_utc", exc)
    if manifest["baseline"]["expected_plugin"] != PLUGINS[manifest["baseline"]["method"]]:
        fail("PLUGIN_MISMATCH", where + ".baseline.expected_plugin", "method/expected_plugin mismatch")
    refs = artifacts(manifest, directory, path)
    for kind, expected in [("scenario", manifest["scenario"]["spec_sha256"]),
                           ("config", manifest["baseline"]["config_sha256"]),
                           ("common_constraints", manifest["baseline"]["common_constraints_sha256"])]:
        matches = [(a, p) for a, p in refs.values() if a["kind"] == kind]
        if len(matches) != 1:
            fail("ARTIFACT_REFERENCE", where + ".artifacts", "Exactly one " + kind + " artifact is required")
        artifact, target = matches[0]
        if artifact["sha256"].lower() != expected.lower():
            fail("IDENTITY_MISMATCH", where + ".artifacts", kind + " hash differs from manifest identity")
        if kind == "scenario":
            spec = load_strict(target)
            validate_structure(spec, "scenario", target)
            scenario_rules(spec, target, check_host_resources=False)
            for field in ("scenario_id", "scenario_revision"):
                if spec[field] != manifest["scenario"][field]:
                    fail("IDENTITY_MISMATCH", where + ".scenario." + field, "Scenario snapshot identity differs")
    snapshot = manifest["source"]["snapshot_ref"]
    if manifest["source"]["dirty"] and snapshot is None:
        fail("SOURCE_SNAPSHOT_REQUIRED", where + ".source.snapshot_ref", "Dirty source needs an actual modification snapshot")
    if snapshot is not None:
        require_ref(refs, snapshot, where + ".source.snapshot_ref", "source_snapshot")


def close(a, b):
    try:
        return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-12)
    except OverflowError:
        return False


def evaluation_rules(evaluation, time, refs, path):
    where = f"{path}:$.evaluation"
    e = evaluation
    measured = [e[k] for k in ("collision", "min_signed_clearance_m", "geometric_contact", "contact_sensor_event")]
    # This check precedes other evaluation conditions so the single false
    # mutation of an unevaluated run produces COLLISION_UNSUPPORTED.
    if e["collision"] is False and (not e["evaluation_valid"] or e["coverage"] != "full"):
        fail("COLLISION_UNSUPPORTED", where + ".collision", "false requires full valid scoped evidence")
    if not e["evaluation_valid"] and not e["invalid_reason"]:
        fail("EVALUATION_INVALID", where + ".invalid_reason", "Invalid evaluation needs a reason")
    if e["evaluation_valid"] and e["invalid_reason"] is not None:
        fail("EVALUATION_INVALID", where + ".invalid_reason", "Valid evaluation has no invalid reason")
    if e["coverage"] == "none":
        if e["evaluation_valid"] or any(x is not None for x in measured) or e["collision_event"] is not None:
            fail("EVALUATION_INVALID", where + ".coverage", "No evaluation cannot contain measurements")
    needs_context = e["evaluation_valid"] or any(x is not None for x in measured) or e["collision_event"] is not None
    if needs_context:
        for field in ("evaluator_version", "scope_id", "objects", "geometry_model", "frame_id", "clock_epoch", "interval_s", "evidence_refs"):
            if not e[field]:
                code = "COLLISION_UNSUPPORTED" if e["collision"] is False else "EVALUATION_INVALID"
                fail(code, where + "." + field, "Measurement/validity needs scope, interval and evidence context")
    for i, ref_id in enumerate(e["evidence_refs"]):
        require_ref(refs, ref_id, where + f".evidence_refs.{i}")
    if e["clock_epoch"] is not None and e["clock_epoch"] != time["clock_epoch"]:
        fail("EPOCH_MISMATCH", where + ".clock_epoch", "Evaluation and run epochs differ")
    interval = e["interval_s"]
    if interval is not None:
        if interval["start"] > interval["end"]:
            fail("TIME_INVALID", where + ".interval_s", "Evaluation interval is reversed")
        if ((time["sim_start_s"] is not None and interval["start"] < time["sim_start_s"]) or
                (time["sim_end_s"] is not None and interval["end"] > time["sim_end_s"])):
            fail("TIME_INVALID", where + ".interval_s", "Evaluation interval exceeds known run interval")
        if e["clock_epoch"] != time["clock_epoch"]:
            fail("EPOCH_MISMATCH", where + ".clock_epoch", "Evaluation interval requires the run epoch")
    event = e["collision_event"]
    if e["collision"] is True:
        if event is None:
            fail("COLLISION_UNSUPPORTED", where + ".collision_event", "true requires a traceable event")
        if not interval or not interval["start"] <= event["at_s"] <= interval["end"]:
            fail("TIME_INVALID", where + ".collision_event.at_s", "Event is outside evaluation interval")
        if not set(event["objects"]).issubset(e["objects"]):
            fail("EVALUATION_INVALID", where + ".collision_event.objects", "Event object is outside declared scope")
        require_ref(refs, event["source_ref"], where + ".collision_event.source_ref")
    elif event is not None:
        fail("EVALUATION_INVALID", where + ".collision_event", "Event requires collision=true")


def result_rules(manifest, result, directory, path, manifest_path):
    where = f"{path}:$"
    for field in ("run_id", "record_origin"):
        if result[field] != manifest[field]:
            fail("IDENTITY_MISMATCH", where + "." + field, "Manifest and result identity differ")
    if result["manifest_sha256"].lower() != digest(manifest_path):
        fail("IDENTITY_MISMATCH", where + ".manifest_sha256", "Result is not bound to actual manifest bytes")
    refs = artifacts(result, directory, path)
    status = result["status"]
    success = {"SUCCEEDED": True, "REJECTED": False, "ABORTED": False, "CANCELED": False, "NOT_SENT": None, "UNKNOWN": None}[status["action"]]
    if result["metrics"]["action_success"] is not success:
        fail("ACTION_MISMATCH", where + ".metrics.action_success", "action_success must follow terminal action mapping")
    if status["startup"] == "FAILED" and status["action"] != "NOT_SENT":
        fail("STATUS_INCONSISTENT", where + ".status.action", "Startup failure cannot have a sent action")
    if status["harness"] == "NAV_TIMEOUT" and status["action"] == "SUCCEEDED":
        fail("STATUS_INCONSISTENT", where + ".status.harness", "Confirmed success cannot be navigation timeout")
    if (status["harness"] != "COMPLETED" or status["startup"] == "FAILED" or status["cleanup"] == "FAILED") and not status["primary_reason"]:
        fail("REASON_REQUIRED", where + ".status.primary_reason", "Failure/timeout must keep its primary reason")
    runtime = result["runtime"]
    if status["startup"] == "READY":
        if runtime is None or runtime["actual_plugin"] != manifest["baseline"]["expected_plugin"]:
            fail("PLUGIN_MISMATCH", where + ".runtime.actual_plugin", "READY needs the actual expected plugin")
        require_ref(refs, runtime["parameters_ref"], where + ".runtime.parameters_ref", "runtime_parameters")
    elif runtime is not None and runtime["parameters_ref"] is not None:
        require_ref(refs, runtime["parameters_ref"], where + ".runtime.parameters_ref", "runtime_parameters")
    time = result["time"]
    if time["clock_epoch"] != manifest["clock"]["clock_epoch"]:
        fail("EPOCH_MISMATCH", where + ".time.clock_epoch", "Manifest/result epochs differ")
    start, end = time["sim_start_s"], time["sim_end_s"]
    if start is not None and manifest["clock"]["sim_start_s"] is not None and start != manifest["clock"]["sim_start_s"]:
        fail("TIME_INVALID", where + ".time.sim_start_s", "Manifest/result simulation starts differ")
    if start is not None and end is not None and end < start:
        fail("TIME_INVALID", where + ".time.sim_end_s", "End precedes start")
    rtf = time["rtf"]
    if rtf is not None:
        try:
            matches = (start is not None and end is not None and
                       close(rtf["sim_delta_s"], end - start) and
                       close(rtf["monotonic_delta_s"], time["elapsed_monotonic_s"]) and
                       close(rtf["value"], rtf["sim_delta_s"] / rtf["monotonic_delta_s"]))
        except (OverflowError, ZeroDivisionError):
            matches = False
        if not matches:
            fail("RTF_INVALID", where + ".time.rtf", "RTF must use the recorded run window and ratio")
    evaluation_rules(result["evaluation"], time, refs, path)


def diagnostic(status, origin=None, errors=None, warnings=None):
    return {"status": status, "validation_scope": "record_contract", "record_origin": origin,
            "errors": errors or [], "warnings": warnings or []}


def validate_scenario(path, require_ready=False):
    warnings = ["Record references do not prove physical or runtime validity."]
    try:
        spec = load_strict(path)
        validate_structure(spec, "scenario", path)
        scenario_rules(spec, path)
        if spec["status"] == "draft":
            warnings.append("Draft scenario; runtime readiness is not confirmed.")
            if require_ready:
                return diagnostic("NOT_READY", errors=[{"code": "SCENARIO_NOT_READY", "path": str(path) + ":$.status", "message": "Valid draft is not ready"}], warnings=warnings)
        return diagnostic("VALID", warnings=warnings)
    except InvalidRecord as exc:
        return diagnostic("INVALID", errors=[exc.error], warnings=warnings)


def validate_run_dir(path, allow_fixtures=False):
    directory = Path(path)
    manifest_path, result_path = directory / "manifest.json", directory / "result.json"
    origin = None
    warnings = ["Reference/hash consistency does not verify log, bag or evaluation content semantics."]
    try:
        manifest = load_strict(manifest_path)
        # Minimal identity check precedes fixture policy; full structure follows it.
        if (type(manifest) is not dict or type(manifest.get("schema_version")) is not int or
                manifest.get("schema_version") != 1 or type(manifest.get("run_id")) is not str or
                not manifest["run_id"].strip() or manifest.get("record_origin") not in ("real", "fixture")):
            validate_structure(manifest, "manifest", manifest_path)
        origin = manifest["record_origin"]
        if origin == "fixture":
            warnings.append("Synthetic fixture; not a robot experiment.")
            if not allow_fixtures:
                fail("FIXTURE_NOT_ALLOWED", str(manifest_path) + ":$.record_origin", "Use --allow-fixtures for synthetic contract tests")
        validate_structure(manifest, "manifest", manifest_path)
        manifest_rules(manifest, directory, manifest_path)
        # lexists semantics: a dangling result link is an invalid existing input.
        if not result_path.exists() and not result_path.is_symlink():
            return diagnostic("INCOMPLETE", origin, [{"code": "MISSING_RESULT", "path": str(result_path), "message": "Valid manifest has no result; nothing was written"}], warnings)
        result = load_strict(result_path)
        validate_structure(result, "result", result_path)
        result_rules(manifest, result, directory, result_path, manifest_path)
        return diagnostic("VALID", origin, warnings=warnings)
    except InvalidRecord as exc:
        return diagnostic("INVALID", origin, [exc.error], warnings)


class JsonArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        fail("CLI_ARGUMENT", "$argv", message)


def main(argv=None):
    parser = JsonArgumentParser(description=__doc__, add_help=False)
    sub = parser.add_subparsers(dest="command", required=True, parser_class=JsonArgumentParser)
    scenario = sub.add_parser("scenario", add_help=False)
    scenario.add_argument("path")
    scenario.add_argument("--require-ready", action="store_true")
    run = sub.add_parser("run", add_help=False)
    run.add_argument("path")
    run.add_argument("--allow-fixtures", action="store_true")
    try:
        args = parser.parse_args(argv)
        report = (validate_scenario(args.path, args.require_ready) if args.command == "scenario"
                  else validate_run_dir(args.path, args.allow_fixtures))
        code = EXIT_CODES[report["status"]]
    except InvalidRecord as exc:
        report, code = diagnostic("INVALID", errors=[exc.error]), 2
    except Exception as exc:
        # An unexpected tool failure is never VALID or downgraded to INCOMPLETE.
        report = diagnostic("INTERNAL_ERROR", errors=[{"code": "INTERNAL_ERROR", "path": "$tool", "message": f"{type(exc).__name__}: {exc}"}])
        code = 1
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return code


if __name__ == "__main__":
    sys.exit(main())
