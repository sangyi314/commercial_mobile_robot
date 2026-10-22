"""Offline final assessment: recompute raw metrics, then include closed-log cleanup errors."""
from pathlib import Path
import sys, importlib.util, json, hashlib, argparse
sys.dont_write_bytecode = True
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("run", type=Path)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
source = Path(__file__).with_name("analyze_smoke.py")
spec = importlib.util.spec_from_file_location("offline_smoke_analysis", source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
report = module.analyze(args.run)
report["initial_resource_release_assessment"] = report["cleanup"]
final = args.run / "runtime/cleanup-final-assessment-20261022.json"
report["cleanup"] = json.loads(final.read_text())
report["input_sha256"][str(final.relative_to(args.run))] = hashlib.sha256(final.read_bytes()).hexdigest()
report["analysis_sources_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in [source, Path(__file__)]}
with args.output.open("x") as f:
    json.dump(report, f, ensure_ascii=False, indent=2, allow_nan=False)
    f.write("\n")
print(json.dumps({"output": str(args.output), "action": report["action"]["terminal"], "cleanup": report["cleanup"]["cleanup_status"], "resources_released": report["cleanup"]["resources_released"]}))
