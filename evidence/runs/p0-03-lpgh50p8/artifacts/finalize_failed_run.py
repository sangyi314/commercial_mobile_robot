"""Finalize the already stopped real run; no ROS commands, no simulation."""
from pathlib import Path
import json,hashlib,datetime,subprocess,platform,xml.etree.ElementTree as ET
root=Path("/home/robot/commercial_mobile_robot");run=root/"evidence/runs/p0-03-lpgh50p8"
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(name):return json.loads((run/name).read_text())
def put(name,data):
 with (run/name).open("x") as f:json.dump(data,f,indent=2,ensure_ascii=False,allow_nan=False);f.write("\n")
start=read("repo-at-start.json");ended=read("runtime/session-ended.json");cleanup=read("runtime/cleanup-evidence.json");pre=read("runtime/pre-summary.json");failure=read("runtime/failure-readonly.json")
assert not (run/"runtime/goal-dispatch.json").exists()
assert not (run/"runtime/nav2-process.json").exists()
assert not (run/"runtime/initialpose-request.yaml").exists()
assert cleanup["cleanup_status"]=="SUCCEEDED" and cleanup["resources_released"]
protected=read("protected-before.json");prior=read("runtime/old-evidence-before.json")
changed=[n for n,h in protected.items() if not(root/n).is_file() or sha(root/n)!=h]
oldchanged=[n for n,h in prior.items() if not(root/n).is_file() or sha(root/n)!=h]
assert not changed and not oldchanged,(changed,oldchanged)
now=datetime.datetime.now(datetime.timezone.utc).isoformat()
put("runtime/final-integrity.json",{"checked_at_utc":now,"protected_files_unchanged":True,"old_evidence_unchanged":True,"protected_count":len(protected),"prior_evidence_count":len(prior),"navigation_config_unchanged":sha(run/"artifacts/nav2_mppi.yaml")==sha(root/"evidence/runs/p0-03-chnntxl6/artifacts/nav2_mppi.yaml"),"scope":"Files in pre-run snapshots only; later unrelated runs are excluded."})
versions={}
for name in ("nav2_bringup","nav2_lifecycle_manager","nav2_map_server","nav2_amcl","rclcpp","rmw_fastrtps_cpp","ros_gz_bridge"):
 p=Path("/opt/ros/jazzy/share")/name/"package.xml"
 if p.exists():versions[name]=ET.parse(p).getroot().findtext("version")
put("runtime/finalization-context.json",{"finalized_at_utc":now,"run_ended_at_utc":ended["ended_at_utc"],"manifest_written_after_failed_run":True,"goal_input_freeze_performed":False,"reason_no_freeze":"Startup failed before AMCL initial pose, navigation creation or goal selection.","current_platform":platform.platform(),"current_installed_versions":versions,"version_scope":"Read at finalization, not a run-time package-version snapshot. Manifest package_versions is empty.","current_git_status":subprocess.check_output(["git","status","--short"],cwd=root,text=True)})
scenario=json.loads((root/"evidence/runs/p0-03-5u34h8ya/artifacts/scenario.yaml").read_text())
scenario["scenario_revision"]="r2+"+run.name
scenario["status"]="draft"
scenario["task"]["initial_pose"]=None
scenario["task"]["goal_pose"]=None
scenario["task"]["common_constraints_ref"]={"path":str((run/"artifacts/common_constraints.json").relative_to(root)),"sha256":sha(run/"artifacts/common_constraints.json"),"package":None,"package_version":None}
scenario["resources"]["world"]={"path":str((run/"artifacts/world-ogre1.sdf").relative_to(root)),"sha256":sha(run/"artifacts/world-ogre1.sdf"),"package":None,"package_version":None}
for key in ("map_yaml","map_image","robot"):
 item=scenario["resources"][key]
 assert sha(Path(item["path"]))==item["sha256"],"Installed resource changed since reference: "+key
 item["package_version"]=None
scenario["verification"]={"resource_dependencies":"pending","world_map_alignment":"pending","poses_free":"pending","interfaces_observed":"pending","evidence_refs":[str((run/"runtime/pre-summary.json").relative_to(root)),str((run/"runtime/failure-readonly.json").relative_to(root))]}
put("artifacts/scenario.yaml",scenario)
manifest_paths=[("scenario","artifacts/scenario.yaml","scenario"),("config","artifacts/nav2_mppi.yaml","config"),("common_constraints","artifacts/common_constraints.json","common_constraints"),("source_at_start","artifacts/source-at-start.tgz","source_snapshot"),("world","artifacts/world-ogre1.sdf","other"),("world_variant","artifacts/world-variant.json","other"),("launch","artifacts/tb3_simulation.launch.py","other"),("session","session.bash","other"),("repair_rationale","artifacts/repair-rationale.json","other"),("finalization_context","runtime/finalization-context.json","other")]
def art(i,p,k):return {"id":i,"path":p,"sha256":sha(run/p),"kind":k}
manifest={"schema_version":1,"run_id":run.name,"record_origin":"real","scenario":{"scenario_id":scenario["scenario_id"],"scenario_revision":scenario["scenario_revision"],"spec_sha256":sha(run/"artifacts/scenario.yaml")},"source":{"git_commit":start["git_commit"],"dirty":bool(start["git_status"].strip()),"snapshot_ref":"source_at_start"},"baseline":{"method":"mppi","config_sha256":sha(run/"artifacts/nav2_mppi.yaml"),"expected_plugin":"nav2_mppi_controller::MPPIController","common_constraints_sha256":sha(run/"artifacts/common_constraints.json")},"environment":{"ros_distro":"jazzy","platform":"Ubuntu ARM64; exact kernel/platform version not captured during this run","package_versions":{},"external_seed":None,"seed_scope":[],"controller_seed_controlled":False},"clock":{"clock_epoch":run.name+":sim0","started_at_utc":start["started_at_utc"],"sim_start_s":None},"artifacts":[art(*x) for x in manifest_paths]}
put("manifest.json",manifest)
evaluation={"evaluator_version":None,"scope_id":None,"objects":[],"geometry_model":None,"frame_id":None,"clock_epoch":None,"interval_s":None,"coverage":"none","evaluation_valid":False,"invalid_reason":"Startup failed before navigation; no independent collision evaluation.","evidence_refs":[],"collision":None,"collision_event":None,"min_signed_clearance_m":None,"geometric_contact":None,"contact_sensor_event":None}
paths=["runtime/preflight.json","runtime/pre-summary.json","runtime/pre-samples.json","runtime/pre-scan.yaml","runtime/failure-readonly.json","runtime/cleanup-evidence.json","runtime/session-ended.json","runtime/final-integrity.json","runtime/finalization-context.json","runtime/gazebo-process.json","runtime/robot-process.json","logs/control-events.jsonl","logs/localization-startup.out","logs/localization-startup.exit","logs/failure-readonly.out","logs/robot-launch.log","logs/gazebo.log","logs/cleanup.jsonl","logs/cleanup-summary.out","logs/cleanup-summary.exit","logs/shutdown-localization.out","logs/shutdown-localization.exit","logs/nodes-after.exit","artifacts/launch.patch","artifacts/cleanup.patch","artifacts/repair-rationale.json","artifacts/control.py","artifacts/scoped_cleanup.py","artifacts/failure_readonly.py"]
result={"schema_version":1,"run_id":run.name,"record_origin":"real","manifest_sha256":sha(run/"manifest.json"),"status":{"startup":"FAILED","action":"NOT_SENT","harness":"STARTUP_FAILED","cleanup":cleanup["cleanup_status"],"primary_reason":"The single localization STARTUP service request did not return within 30 wall seconds. Subsequent read-only GetState responses showed map_server and amcl unconfigured; shutdown later responded successfully. Navigation processes were never created and no initial pose or goal was sent. Service transport/callback cause remains unresolved."},"runtime":None,"time":{"clock_epoch":manifest["clock"]["clock_epoch"],"sim_start_s":None,"sim_end_s":None,"elapsed_monotonic_s":ended["elapsed_monotonic_s"],"rtf":None},"metrics":{"action_success":None,"goal_position_error_m":None,"goal_yaw_error_rad":None,"stop_observed":None,"intervention":None,"delivery_success":None},"evaluation":evaluation,"artifacts":[art("evidence_"+str(i),p,"log" if p.startswith("logs/") else "other") for i,p in enumerate(paths)]}
put("result.json",result)
r=subprocess.run(["python3","-B",str(root/"tools/validate_run.py"),"run",str(run)],cwd=root,capture_output=True,text=True)
for suffix,s in (("json",r.stdout),("err",r.stderr),("exit",str(r.returncode)+"\n")):
 with (run/"logs"/("run-final-check."+suffix)).open("x") as f:f.write(s)
print(r.stdout);print("exit",r.returncode);print("result_sha256",sha(run/"result.json"))
raise SystemExit(r.returncode)
