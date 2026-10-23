"""Finalize new P0-04 records from closed raw files only; never overwrite."""
from pathlib import Path
import hashlib,json,re,subprocess,sys
root=Path('/home/robot/commercial_mobile_robot');run=Path(sys.argv[1])
def read(name):return json.loads((run/name).read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert not (run/'result.json').exists()
manifest=read('manifest.json');cleanup=read('runtime/cleanup-evidence.json');ended=read('runtime/session-ended.json')
failure=read('runtime/session-failure.json') if (run/'runtime/session-failure.json').exists() else None
evidence=read('runtime/independent-offline-evidence.json') if (run/'runtime/independent-offline-evidence.json').exists() else None
term=read('runtime/goal-terminal.json')['data'] if (run/'runtime/goal-terminal.json').exists() else {'action':'NOT_SENT'}
actual=read('runtime/all-parameters.json')['controller_server']['FollowPath.plugin'] if (run/'runtime/all-parameters.json').exists() else None
badproc=[p for p in cleanup['processes'] if p['exit']!=0 or p.get('forced') or (p['label'] in ['navigation','localization'] and p['shutdown_service'] is not True)]
abnormal_logs=[]
for name in ['navigation.log','localization.log','bridge.log','robot_state_publisher.log','gazebo.log']:
 p=run/'logs'/name
 if p.exists():
  for i,line in enumerate(p.read_text(errors='replace').splitlines(),1):
   if re.search(r'process has died|SIGKILL|SIGTERM|Segmentation fault|segmentation fault|terminate called|Aborted \(core',line):abnormal_logs.append({'file':str(p.relative_to(run)),'line':i,'text':line})
clean_ok=not badproc and not cleanup['remaining_owned_groups'] and not abnormal_logs
checks=evidence['checks'] if evidence else {}
reason=failure['message'] if failure else None
if not reason and evidence and not all(checks.values()):reason='Offline behavioral evidence checks failed: '+','.join(k for k,v in checks.items() if not v)
if not reason and not clean_ok:reason='Navigation action '+term['action']+'; cleanup abnormal: '+json.dumps(badproc,ensure_ascii=False)
action=term['action'];mapping={'SUCCEEDED':True,'ABORTED':False,'CANCELED':False,'REJECTED':False,'NOT_SENT':None,'UNKNOWN':None}
artifact=[]
for p in sorted(run.rglob('*')):
 if not p.is_file() or p.stat().st_size==0 or p.name in ['result.json','manifest.json']:continue
 artifact.append({'id':'runtime_parameters' if p==run/'runtime/controller_server.yaml' else 'evidence_'+str(len(artifact)),'path':str(p.relative_to(run)),'sha256':sha(p),'kind':'runtime_parameters' if p==run/'runtime/controller_server.yaml' else ('log' if p.parent==run/'logs' else 'other')})
result={'schema_version':1,'run_id':run.name,'record_origin':'real','manifest_sha256':sha(run/'manifest.json'),'status':{'startup':'READY' if actual==manifest['baseline']['expected_plugin'] else 'FAILED','action':action,'harness':'INTERNAL_ERROR' if failure else 'COMPLETED','cleanup':'SUCCEEDED' if clean_ok else 'FAILED','primary_reason':reason},'runtime':{'actual_plugin':actual,'parameters_ref':'runtime_parameters'} if actual else None,'time':{'clock_epoch':manifest['clock']['clock_epoch'],'sim_start_s':None,'sim_end_s':None,'elapsed_monotonic_s':ended['elapsed_monotonic_s'],'rtf':None},'metrics':{'action_success':mapping[action],'goal_position_error_m':evidence['arrival']['last_error']['position_m'] if evidence else None,'goal_yaw_error_rad':evidence['arrival']['last_error']['yaw_rad'] if evidence else None,'stop_observed':checks.get('stop'),'intervention':None,'delivery_success':None},'evaluation':{'evaluator_version':None,'scope_id':None,'objects':[],'geometry_model':None,'frame_id':None,'clock_epoch':None,'interval_s':None,'coverage':'none','evaluation_valid':False,'invalid_reason':'Independent collision evaluation not performed. Localization estimates and absence of monitor triggers are not collision truth.','evidence_refs':[],'collision':None,'collision_event':None,'min_signed_clearance_m':None,'geometric_contact':None,'contact_sensor_event':None},'artifacts':artifact}
with (run/'result.json').open('x') as f:json.dump(result,f,indent=2,ensure_ascii=False,allow_nan=False)
check=subprocess.run(['python3','-B',str(root/'tools/validate_run.py'),'run',str(run)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
for name,value in [('run-final-check.json',check.stdout),('run-final-check.err',check.stderr),('run-final-check.exit',str(check.returncode)+'\n')]:
 with (run/'logs'/name).open('x') as f:f.write(value)
print(json.dumps({'run':str(run),'status':result['status'],'metrics':result['metrics'],'abnormal_log_excerpts':abnormal_logs,'validator_exit':check.returncode,'validator':check.stdout},ensure_ascii=False))
