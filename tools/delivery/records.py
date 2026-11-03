# SPDX-License-Identifier: MIT
"""Read-only delivery evidence and SQLite invariants; no ROS, migration or playback."""
import json,sqlite3,hashlib,shlex,fcntl
from contextlib import ExitStack,closing
from pathlib import Path
STATES=['ACCEPTED','NAVIGATING','WAIT_HANDOVER','RETURNING','PAUSED','RECOVERING','COMPLETED','FAILED','CANCELED']
PHASES=['NONE','SEND_PENDING','ACTIVE','STOP_PENDING','UNCERTAIN']
TERMINAL={'COMPLETED','FAILED','CANCELED'}
def decode_record(text):
 t=shlex.split(text)
 if len(t)<31 or t[0] not in ['1','2','3']:raise ValueError('unsupported/corrupt Record')
 version=int(t[0]);expected=31 if version==1 else (41 if version==3 else len(t))
 if len(t)!=expected or version==2 and len(t) not in [39,40]:raise ValueError('Record field count')
 state=int(t[8]);phase=int(t[9])
 if state not in range(9) or phase not in range(5):raise ValueError('Record enum')
 r=dict(zip(['mission_id','delivery_station','return_station','map_version','station_version'],t[2:7]));r.update(state=STATES[state],phase=PHASES[phase],epoch=t[10],operation=t[11],attempt=t[12],uuid=t[13],pending=t[14],reason=t[15],gate_epoch=t[16],clock_epoch=t[17],version=int(t[18]),event_seq=int(t[19]),retries=int(t[20]),stop_until=int(t[28]),deadline=int(t[29]),last_now=int(t[30]))
 for key,value in zip(['returning','handed_over','goal_terminal','revoked','stopped','arrived','ever_sent'],t[21:28]):
  if value not in ['0','1']:raise ValueError('Record bool')
  r[key]=value=='1'
 if version>1:
  offset=31
  if version==3 or len(t)==40:r['retry_eligible']=t[offset]=='1';offset+=1
  r.update(goal_succeeded=t[offset]=='1',arrival_until=int(t[offset+1]),terminal_ros_ns=int(t[offset+2]),handover_until=int(t[offset+3]),handover_budget=int(t[offset+4]),handover_version=int(t[offset+5]),handover_id=t[offset+6],handover_source=t[offset+7],restart_suspended=version==3 and t[-1]=='1')
 if r['retries'] not in [0,1] or r['version']<0:raise ValueError('Record budget/version')
 return r

def read_database(path):
 path=Path(path).resolve()
 if not path.is_file():raise ValueError('missing mission DB')
 # This is a CLOSED-session reader, not a live SQLite observer. SQLite's
 # ordinary mode=ro can create/delete WAL/SHM sidecars. immutable=1 is safe
 # only while the CMR owner lock excludes a writer and no WAL is pending.
 with ExitStack() as stack:
  lock=Path(str(path)+'.owner.lock')
  if lock.exists():
   handle=stack.enter_context(lock.open('rb'))
   try:fcntl.flock(handle.fileno(),fcntl.LOCK_SH|fcntl.LOCK_NB)
   except BlockingIOError as exc:raise ValueError('live Mission owner; closed DB required') from exc
  for suffix in ['-wal','-journal']:
   sidecar=Path(str(path)+suffix)
   if sidecar.exists() and sidecar.stat().st_size:raise ValueError('pending SQLite journal; audited closed snapshot required')
  db=stack.enter_context(closing(sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1',uri=True)))
  db.execute('PRAGMA query_only=ON')
  if db.execute('PRAGMA user_version').fetchone()[0]!=2:raise ValueError('delivery DB schema')
  if db.execute('PRAGMA integrity_check').fetchall()!=[('ok',)] or db.execute('PRAGMA foreign_key_check').fetchall():raise ValueError('DB integrity')
  return {name:db.execute('SELECT * FROM '+name).fetchall() for name in ['missions','manager','events','operations','registrations','queue_events','command_decisions','delivery_policy']}

def audit_database(path):
 db=read_database(path);errors=[];latest={};uuids={};max_active=0;max_unresolved=0;handover=[];event_lines=[]
 for row in db['events']:
  seq,mission,event_id,kind,identity,reason,before,encoded=row;r=decode_record(encoded);previous=latest.get(mission)
  if r['mission_id']!=mission:errors.append('event/Record identity '+str(seq))
  if previous and int(before)!=previous['version']:errors.append('before version '+str(seq))
  if reason=='APPLIED' and r['version']!=int(before)+1:errors.append('applied version '+str(seq))
  if reason=='APPLIED' and int(kind)==1:
   if not r['uuid']:errors.append('accepted UUID missing')
   if r['uuid'] in uuids and uuids[r['uuid']]!=r['attempt']:errors.append('UUID reused by attempts')
   uuids[r['uuid']]=r['attempt']
  if reason=='APPLIED' and int(kind)==15 and r['state'] in ['WAIT_HANDOVER','COMPLETED']:
   if not all(r[k] for k in ['goal_terminal','revoked','stopped','arrived','goal_succeeded']) or r['last_now']>=min(r['arrival_until'],r['stop_until']):errors.append('resolved without independent facts '+str(seq))
  if reason=='APPLIED' and int(kind)==10:
   if not previous or previous['state']!='WAIT_HANDOVER' or not r['handed_over'] or r['operation']==previous['operation']:errors.append('handover/return guard '+str(seq))
   handover.append({'mission_id':mission,'event_seq':seq,'source':r['handover_source'],'handover_id':r['handover_id'],'entry_version':r['handover_version']})
  if previous and previous['handed_over'] and not r['handed_over']:errors.append('handover fact erased')
  if previous and previous['state'] in TERMINAL and r['state']!=previous['state']:errors.append('terminal resurrection')
  latest[mission]=r;max_active=max(max_active,sum(x['state'] not in TERMINAL for x in latest.values()));max_unresolved=max(max_unresolved,sum(x['ever_sent'] and not x['goal_terminal'] for x in latest.values()))
  event_lines.append({'record_type':'mission_event','persistent_seq':seq,'mission_id':mission,'event_id':event_id,'kind':int(kind),'identity':identity,'reason':reason,'before_version':before,'record':r})
 if max_active>1 or max_unresolved>1:errors.append('more than one active Mission/Goal')
 final={x[0]:decode_record(x[1]) for x in db['missions']}
 for row in db['missions']:
  if (row[2]==1)!=(final[row[0]]['state'] in TERMINAL):errors.append('terminal column mismatch')
 for key,value in final.items():
  if latest.get(key)!=value:errors.append('final event/Record mismatch '+key)
 slot=db['manager'][0][1]
 active_ids=[k for k,r in final.items() if r['state'] not in TERMINAL]
 if active_ids!=([slot] if slot else []):errors.append('active slot/Record mismatch')
 registrations=db['registrations'];seen=[];waiting=[];queue_order=[]
 for seq,mission,request,snapshot,state in registrations:
  req=decode_record(request);snapshot=json.loads(snapshot)
  if req['mission_id']!=mission or seq in seen or seen and seq<=seen[-1]:errors.append('registration identity/order')
  seen.append(seq)
  if state=='WAITING':
   waiting.append(mission)
   if mission in final:errors.append('waiting incorrectly creates active Mission')
  if state=='ACTIVE' and (mission!=slot or mission not in final):errors.append('active registration mismatch')
  if state=='FINISHED' and (mission not in final or final[mission]['state'] not in TERMINAL):errors.append('finished registration mismatch')
  queue_order.append({'mission_id':mission,'queue_seq':seq,'queue_state':state,'request':req,'snapshot':snapshot})
 capacity=db['delivery_policy'][0][1]
 if len(waiting)>capacity:errors.append('waiting exceeds capacity')
 sends=[{'effect_id':r[0],'mission_id':r[1],'operation':r[2],'attempt':r[3],'uuid':r[4],'kind':r[5],'status':r[6]} for r in db['operations'] if r[5]==3]
 for seq,mission,kind,detail in db['queue_events']:event_lines.append({'record_type':'queue_event','persistent_seq':seq,'mission_id':mission,'kind':kind,'detail':detail,'order_domain':'queue_events; not a cross-table timestamp'})
 return {'valid':not errors,'errors':errors,'max_active_missions':max_active,'max_unresolved_goals':max_unresolved,'accepted_uuids':uuids,'records':final,'active_slot':slot,'waiting':waiting,'queue':queue_order,'handover':handover,'authorized_send_intents':sends,'events':event_lines,'unique_admitted':len(registrations),'submit_calls':sum(x[2]=='SUBMIT' for x in db['queue_events']),'capacity':capacity,'schedule_paused':bool(db['manager'][0][-1])}

def validate_references(root,refs):
 root=Path(root).resolve();errors=[]
 for ref in refs:
  path=(root/ref['path']).resolve()
  if not path.is_relative_to(root):errors.append('reference escapes session');continue
  if not path.is_file() or not path.stat().st_size:errors.append('missing '+ref['path']);continue
  actual=hashlib.sha256(path.read_bytes()).hexdigest()
  if actual!=ref['sha256']:errors.append('wrong hash '+ref['path'])
 return errors

def audit_protocol_case(root):
 root=Path(root);r=audit_database(root/'mission.db');trace=json.loads((root/'trace.json').read_text());cleanup=json.loads((root/'cleanup.json').read_text())
 if any('escalation' in x or x.get('exit')!=x.get('expected_exit') for x in cleanup):r['errors'].append('cleanup failure')
 dispatch=[];native_accepted=[]
 for log in root.glob('mission*.log'):
  for line in log.read_text().splitlines():
   if line.startswith('NAV_DISPATCH '):dispatch.append(json.loads(line[len('NAV_DISPATCH '):]))
 if len({x['attempt'] for x in dispatch})!=len(dispatch):r['errors'].append('duplicate native Goal dispatch')
 if len(dispatch)>len(r['authorized_send_intents']):r['errors'].append('send without persisted authorized intent')
 if len(r['accepted_uuids'])>len(dispatch):r['errors'].append('accepted UUID without native send')
 r.update(native_dispatches=dispatch,synthetic=True,valid=not r['errors']);return r

def expected_business(case,db,reliability=None):
 if reliability:
  expected=len(reliability["policy"]["submit_at_s"]);r=list(db["records"].values())
  return len(r)==expected and all(x["state"]=="COMPLETED" and x["handed_over"] and x["handover_source"]=="synthetic" for x in r) and not db["waiting"] and not db["active_slot"]
 records=list(db['records'].values());sends=len(db['authorized_send_intents'])
 if case in ['G01','G01D','G02','G05']:
  count=2 if case=='G05' else 1
  source='human' if case=='G01' else 'synthetic'
  return len(records)==count and all(r['state']=='COMPLETED' and r['handed_over'] and r['handover_source']==source for r in records) and sends>=2*count and sends<=3*count and not db['waiting'] and not db['active_slot']
 if case=='G03':return len(records)==1 and records[0]['state']=='CANCELED' and not records[0]['handed_over'] and db['schedule_paused'] and sends==1
 if case=='G04':return len(records)==1 and records[0]['state']=='FAILED' and not records[0]['handed_over'] and db['schedule_paused'] and sends==1
 if case=='G06':return len(records)==1 and records[0]['state']=='PAUSED' and records[0]['restart_suspended'] and not records[0]['goal_terminal'] and db['schedule_paused'] and len(db['waiting'])==1 and sends==1
 return False

def audit_session(root,recompute_bag=True):
 root=Path(root);manifest=json.loads((root/'mission-manifest.v1.json').read_text());result=json.loads((root/'mission-result.v1.json').read_text());errors=[]
 if manifest.get('record_family')!='cmr.delivery-session' or manifest.get('schema_version')!=1 or result.get('record_family')!='cmr.delivery-session' or result.get('schema_version')!=1 or manifest.get('session_id')!=root.name or result.get('session_id')!=root.name:errors.append('family/version/session')
 refs=result.get('artifacts',[]);names=[r['path'] for r in refs]
 required={'mission-manifest.v1.json','mission-events.jsonl','queue-ledger.v1.json','artifacts/profile.json','artifacts/stations.v1.json','artifacts/delivery.bt.xml','runtime/mission.db','runtime/cleanup.json','runtime/worker-mode.json','runtime/execution-graph.json','runtime/bag-check.json','runtime/delivery-case.json','runtime/observations.jsonl','runtime/p0-08-gazebo.jsonl','bag/metadata.yaml','logs/mission.log'}
 if not required.issubset(set(names)) or len(names)!=len(set(names)) or not any(n.endswith('.mcap') for n in names):errors.append('missing or duplicate required raw artifact')
 errors+=validate_references(root,refs)
 if manifest.get('record_origin')!='gazebo' or result.get('synthetic_observations') is not False:errors.append('evidence layer')
 worker=json.loads((root/'runtime/worker-mode.json').read_text())
 if worker.get('action_client_created') is not False or worker.get('mode')!='mission_observer':errors.append('second worker Goal owner')
 if (root/'logs/action-client.jsonl').read_text().strip():errors.append('worker sent/canceled a Goal')
 db=audit_database(root/'runtime/mission.db');errors+=db['errors']
 if db['records']!=result.get('missions'):errors.append('result does not match persisted final records')
 if any(r['state']=='COMPLETED' and not r['handed_over'] for r in db['records'].values()):errors.append('completion without handover')
 profile=json.loads((root/'artifacts/profile.json').read_text());case=profile['delivery_v1']['case']
 if case!=manifest['case'] or case!=result['case']:errors.append('case identity')
 if profile['delivery_v1'].get('controller_profile')=='p104-mppi-short-leg-v4' and manifest['method']=='mppi':
  actual=json.loads((root/'runtime/all-parameters.json').read_text())['controller_server']
  if manifest.get('controller')!={'plugin':'nav2_rotation_shim_controller::RotationShimController','primary':'nav2_mppi_controller::MPPIController'} or actual.get('FollowPath.plugin')!='nav2_rotation_shim_controller::RotationShimController' or actual.get('FollowPath.primary_controller')!='nav2_mppi_controller::MPPIController' or actual.get('FollowPath.rotate_to_goal_heading') is not True or actual.get('FollowPath.GoalAngleCritic.enabled') is not False or actual.get('FollowPath.rotate_to_heading_once') is not True or actual.get('FollowPath.GoalCritic.cost_weight')!=20.:errors.append('actual MPPI/Rotation Shim backend evidence mismatch')
 if json.loads((root/'queue-ledger.v1.json').read_text())!=db['queue']:errors.append('queue ledger mismatch')
 frozen=json.loads((root/'artifacts/stations.v1.json').read_text())
 if hashlib.sha256((root/'artifacts/delivery.bt.xml').read_bytes()).hexdigest()!=frozen['bt']['sha256']:errors.append('BT identity')
 for row in db['queue']:
  if row['snapshot']['catalog_sha256']!=hashlib.sha256((root/'artifacts/stations.v1.json').read_bytes()).hexdigest():errors.append('persisted catalog identity')
 cleanup=json.loads((root/'runtime/cleanup.json').read_text())
 clean=cleanup['ok'] and not cleanup['remaining'] and bool(cleanup['processes']) and all(p['ok'] and not p['forced'] and not p['remaining'] and not p['errors'] for p in cleanup['processes'])
 if clean!=result.get('cleanup'):errors.append('cleanup claim mismatch')
 business=expected_business(case,db,profile.get("reliability_v1"))
 if profile.get("reliability_v1"):
  from tools.reliability.audit import audit_data
  errors += ["reliability: "+e for e in audit_data(root,db,json.loads((root/"runtime/bag-check.json").read_text()))["errors"]
 if business!=result.get('expected_business_result'):errors.append('business claim mismatch')
 bag=json.loads((root/'runtime/bag-check.json').read_text())
 if recompute_bag:
  from tools.delivery.inspect_delivery_bag import inspect
  independent=inspect(root)
  if independent!=bag:errors.append('bag report does not match independent recomputation')
 errors+=['bag: '+x for x in bag.get('errors',[])]
 if not bag.get('valid') or bag.get('synthetic') is not False:errors.append('bag raw proof absent')
 graph=json.loads((root/'runtime/execution-graph.json').read_text())
 if profile['delivery_v1'].get('stop_feedback',{}).get('required') and not all(x.get('oracle_stop_assisted') is True for x in [manifest,result,bag,graph]):errors.append('Oracle stop assistance label missing or inconsistent')
 if graph.get('old_cmd_vel_endpoints')!=0 or graph.get('docking_absent') is not True or 'Mission' not in graph.get('goal_owner',''):errors.append('graph owner/route evidence')
 fault=json.loads((root/'runtime/delivery-case.json').read_text())
 if case in ['G03','G06'] and fault.get('fault_activation')!='CONFIRMED':errors.append('NOT_EXERCISED fault')
 if case=='G01' and not all(r['handover_source']=='human' for r in db['records'].values()):errors.append('human handover missing')
 harness=result.get('harness')=='COMPLETED'
 return {'valid':not errors,'errors':errors,'oracle_stop_assisted':bool(profile['delivery_v1'].get('stop_feedback',{}).get('oracle_stop_assisted')),'business_ok':business,'cleanup_ok':clean,'bag_ok':bag.get('valid') is True,'session_pass':not errors and business and clean and harness,'database':{k:v for k,v in db.items() if k!='events'}}

def finalize_session(root,manifest,session,cleanup,bag,state,entry):
 import time
 from tools.runner.runner_core import write_json,digest
 root=Path(root)
 try:
  db=audit_database(root/'runtime/mission.db')
  with (root/'mission-events.jsonl').open('x') as f:
   for event in db['events']:f.write(json.dumps(event,allow_nan=False)+'\n')
  write_json(root/'queue-ledger.v1.json',db['queue'])
  refs=[{'path':str(p.relative_to(root)),'sha256':digest(p)} for p in sorted(root.rglob('*')) if p.is_file() and p.stat().st_size and p.name not in ['mission-result.v1.json','mission.db-shm','mission.db-wal']]
  result={'schema_version':1,'record_family':'cmr.delivery-session','session_id':root.name,'case':manifest['case'],'oracle_stop_assisted':manifest.get('oracle_stop_assisted',False),'synthetic_observations':False,'missions':db['records'],'unique_admitted':db['unique_admitted'],'submit_calls':db['submit_calls'],'expected_business_result':expected_business(manifest['case'],db,session.p.get('reliability_v1') if session else None),'harness':state.harness,'cleanup':cleanup,'elapsed_monotonic_s':time.monotonic()-entry,'artifacts':refs,'C':'CONSUMER_ACK_UNKNOWN','collision':None,'failure':session.failure if session else 'session not created'}
  write_json(root/'mission-result.v1.json',result)
  report=audit_session(root,recompute_bag=False);write_json(root/'runtime/delivery-validation.json',report)
  return {'status':'VALID' if report['session_pass'] else 'INVALID','audit':report}
 except Exception as exc:
  import traceback
  failure={'status':'INVALID','error':str(exc),'traceback':traceback.format_exc()};write_json(root/'runtime/delivery-finalization-error.json',failure);return failure
