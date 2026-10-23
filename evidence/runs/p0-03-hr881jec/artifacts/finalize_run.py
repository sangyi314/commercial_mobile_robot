"""Archive the actual run after logs are closed, using existing v1 contract."""
from pathlib import Path
import json,hashlib,sys,subprocess
run=Path(sys.argv[1]);root=Path('/home/robot/commercial_mobile_robot')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(n):return json.loads((run/n).read_text())
manifest=read('manifest.json');freeze=read('artifacts/input-freeze.json');summary=read('runtime/navigation-evidence.json');cleanup=read('runtime/cleanup-evidence.json');ended=read('runtime/session-ended.json')
assert sha(run/'manifest.json')==freeze['manifest_sha256']
assert all(sha(run/name)==h for name,h in freeze['inputs'].items())
protected=read('protected-before.json')
changed=[name for name,h in protected.items() if sha(root/name)!=h]
old=read('runtime/old-evidence-before.json')
oldchanged=[name for name,h in old.items() if not (root/name).exists() or sha(root/name)!=h]
assert not changed and not oldchanged,(changed,oldchanged)
guard={'protected_files_unchanged':not changed,'old_evidence_unchanged':not oldchanged,'frozen_inputs_unchanged':True,'protected_count':len(protected),'prior_evidence_count':len(old)}
(run/'runtime/final-integrity.json').write_text(json.dumps(guard,indent=2)+'\n')
evaluation={'evaluator_version':None,'scope_id':None,'objects':[],'geometry_model':None,'frame_id':None,'clock_epoch':None,'interval_s':None,'coverage':'none','evaluation_valid':False,'invalid_reason':'Independent collision evaluation was not performed; software costmaps and successful navigation do not establish physical collision truth.','evidence_refs':[],'collision':None,'collision_event':None,'min_signed_clearance_m':None,'geometric_contact':None,'contact_sensor_event':None}
artifacts=[]
paths=['runtime/controller_server.yaml','runtime/navigation-evidence.json','runtime/cleanup-evidence.json','runtime/session-ended.json','runtime/final-integrity.json','runtime/pre-goal-audit-v2.json','runtime/goal-dispatch.json','runtime/goal-terminal.json','runtime/observer-shutdown.json','runtime/observations-goal.jsonl','runtime/observations.jsonl','runtime/initialpose-request.yaml','runtime/initialize-initialpose-echo.yaml','runtime/post-summary.json','runtime/post-samples.json','runtime/post-amcl-pose.yaml','logs/action-client.jsonl','logs/nav2-launch.log','logs/gazebo.log','logs/cleanup.jsonl','logs/navigation-analysis.out','logs/navigation-analysis.exit','logs/nodes-after.exit','logs/shutdown-navigation.out','logs/shutdown-localization.out','logs/run-before-goal.json','logs/run-before-goal.exit','artifacts/input-freeze.json','artifacts/analyze_navigation.py','artifacts/repair-rationale.json','artifacts/launch.patch']
for i,path in enumerate(paths):
 p=run/path
 assert p.is_file() and p.stat().st_size,path
 artifacts.append({'id':'runtime_parameters' if path=='runtime/controller_server.yaml' else f'evidence_{i:02d}','path':path,'sha256':sha(p),'kind':'runtime_parameters' if path=='runtime/controller_server.yaml' else 'log' if path.startswith('logs/') else 'other'})
failedcleanup=cleanup['cleanup_status']!='SUCCEEDED'
result={'schema_version':1,'run_id':run.name,'record_origin':'real','manifest_sha256':sha(run/'manifest.json'),'status':{'startup':'READY','action':summary['action'],'harness':'COMPLETED','cleanup':cleanup['cleanup_status'],'primary_reason':'Navigation succeeded, but shutdown produced abnormal process exits/forced termination; cleanup failed. See closed launch and cleanup logs.' if failedcleanup else None},'runtime':{'actual_plugin':manifest['baseline']['expected_plugin'],'parameters_ref':'runtime_parameters'},'time':{'clock_epoch':manifest['clock']['clock_epoch'],'sim_start_s':None,'sim_end_s':None,'elapsed_monotonic_s':ended['elapsed_monotonic_s'],'rtf':None},'metrics':{'action_success':summary['action']=='SUCCEEDED','goal_position_error_m':summary['arrival']['last_error']['position_m'],'goal_yaw_error_rad':summary['arrival']['last_error']['yaw_rad'],'stop_observed':summary['checks']['stop'],'intervention':False,'delivery_success':None},'evaluation':evaluation,'artifacts':artifacts}
with (run/'result.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
r=subprocess.run(['python3','-B',str(root/'tools/validate_run.py'),'run',str(run)],capture_output=True,text=True)
(run/'logs/run-final-check.json').write_text(r.stdout);(run/'logs/run-final-check.err').write_text(r.stderr);(run/'logs/run-final-check.exit').write_text(str(r.returncode)+'\n')
print(r.stdout, 'exit',r.returncode,'INTEGRITY',json.dumps(guard),'RESULT',sha(run/'result.json'))
raise SystemExit(r.returncode)
