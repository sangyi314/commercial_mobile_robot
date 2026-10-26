"""Strict serial batch contracts, append-only recovery and read-only reconciliation."""
from pathlib import Path
import hashlib,json,os,time
from tools.runner.runner_core import strict_json, encode, digest, write_json, run_id


def journal_read(path):
 p=Path(path)
 if not p.exists():return []
 raw=p.read_bytes()
 if raw and not raw.endswith(b'\n'):raise ValueError('truncated journal tail; no automatic replay')
 rows=[];previous='0'*64
 for line in raw.splitlines():
  row=strict_json(line.decode());hashed=row.pop('sha256')
  if row.get('seq')!=len(rows)+1 or row.get('previous')!=previous or hashlib.sha256(encode(row)).hexdigest()!=hashed:raise ValueError('journal sequence/hash mismatch')
  row['sha256']=hashed;rows.append(row);previous=hashed
 return rows


class Journal:
 def __init__(self,path):self.path=Path(path);self.rows=journal_read(path)
 def append(self,kind,**data):
  row={'seq':len(self.rows)+1,'previous':self.rows[-1]['sha256'] if self.rows else '0'*64,'kind':kind,'wall_unix_s':time.time(),'monotonic_s':time.monotonic(),**data}
  row['sha256']=hashlib.sha256(encode(row)).hexdigest()
  with self.path.open('ab') as f:f.write(encode(row));f.flush();os.fsync(f.fileno())
  self.rows.append(row);return row


def attempts(rows):
 result={}
 for e in rows:
  aid=e.get('attempt_id')
  if not aid:continue
  if e['kind']=='INTENT':
   if aid in result:raise ValueError('duplicate intent')
   result[aid]={'intent':e,'started':None,'finished':None}
  else:
   if aid not in result:raise ValueError('event without intent')
   key={'CALL_STARTED':'started','CALL_FINISHED':'finished','RECOVERED':'finished'}.get(e['kind'])
   if key:
    if result[aid][key] is not None:raise ValueError('duplicate '+key)
    result[aid][key]=e
 return result


def cause_key(value):
 if isinstance(value,str):
  try:value=strict_json(value)
  except ValueError:return value
 if isinstance(value,dict):return tuple((k,str(value.get(k))) for k in ['code','phase','detail'])
 return value

def choose_slot(matrix,rows,resume_reason=None):
 records=list(attempts(rows).values())
 if any(a['finished'] is None for a in records):raise ValueError('unresolved attempt; reconcile before dispatch')
 last=records[-1] if records else None
 if last and not last['finished']['outcome']['pass']:
  if not last['finished']['outcome']['safe_to_resume']:raise ValueError('unsafe prior cleanup or identity; no resume')
  if not resume_reason:raise ValueError('failed attempt requires explicit review reason')
  if len(records)>1:
   a,b=[x['finished']['outcome'] for x in records[-2:]]
   if not a['pass'] and cause_key(a.get('first_reason'))==cause_key(b.get('first_reason')):raise ValueError('same first cause twice; diagnose and new cohort after fix')
 done={x['intent']['slot_id'] for x in records if x['finished']['outcome']['pass']}
 return next((s for s in matrix['slots'] if s['slot_id'] not in done),None)


def source_inventory(root,matrix):
 root=Path(root);paths=set()
 for directory in ['tools','experiments','src/cmr_bringup/config']:
  paths.update(p.resolve() for p in (root/directory).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
 paths.update(p.resolve() for p in (root/'schemas').rglob('*') if p.is_file())
 # Profile/resource closure and external files are frozen, not just the constraints file.
 import yaml
 for slot in matrix['slots']:
  paths.update(Path(slot[k]['path']) for k in ['profile','scenario'])
  p=yaml.safe_load(Path(slot['profile']['path']).read_text())
  paths.update(Path(v['path']) for v in p['files'].values())
  paths.update(Path(v['config']) for v in p['methods'].values())
  paths.add(Path(p['recording']['qos']));paths.add(Path(p['gazebo_environment']['LD_PRELOAD']))
  paths.update(Path(n) for n in strict_json(Path(p['files']['dependencies']['path']).read_text())['files'])
 return {str(p):digest(p) for p in sorted(paths)}


def validate_matrix(matrix,root,check_inputs=True):
 if set(matrix)!={'matrix_version','purpose','slots'} or matrix['matrix_version']!=1:raise ValueError('matrix keys/version')
 purpose=matrix['purpose'];slots=matrix['slots']
 if purpose not in ['normal','fault'] or len(slots)!=(20 if purpose=='normal' else 6):raise ValueError('fixed matrix size')
 expected=[]
 if purpose=='normal':
  for pair in range(1,11):
   expected.extend([(m,None,pair) for m in (['mppi','dwb'] if pair%2 else ['dwb','mppi'])])
 else:
  for f in ['controller-deactivate','short-deadline','pause-gazebo']:expected.extend((m,f,None) for m in ['mppi','dwb'])
 seen=set()
 for slot,(method,fault,pair) in zip(slots,expected):
  if set(slot)!={'slot_id','pair_index','method','purpose','cold_start','scenario','profile','fault'}:raise ValueError('slot keys')
  run_id(slot['slot_id'])
  if slot['slot_id'] in seen:raise ValueError('duplicate slot')
  seen.add(slot['slot_id'])
  if (slot['method'],slot['fault'],slot['pair_index'])!=(method,fault,pair) or slot['purpose']!=purpose or slot['cold_start'] is not True:raise ValueError('unreviewed order/purpose/cold start')
  for key in ['scenario','profile']:
   ref=slot[key]
   if set(ref)!={'path','sha256'} or not Path(ref['path']).is_absolute() or digest(ref['path'])!=ref['sha256']:raise ValueError(key+' identity')
  if check_inputs:
   from tools.runner.trial import load_inputs
   from tools.runner.runner_faults import validate_policy
   p,_=load_inputs(slot['scenario']['path'],slot['profile']['path'],method,'p008-readonly-check')
   validate_policy(p,fault)
 return matrix


def live_owned(run):
 """A free lock is not evidence of absent descendants; match durable ownership tokens."""
 path=Path(run)/'logs/events.jsonl'
 # Actual runner event file is runtime/events.jsonl (fallback retained for diagnostics).
 if not path.exists():path=Path(run)/'runtime/events.jsonl'
 tokens=set()
 if path.exists():
  for line in path.read_text().splitlines():
   e=strict_json(line)
   if e.get('type')=='PROCESS_STARTED':tokens.add(e['data']['identity']['ownership_token'])
 result=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   from tools.runner.runner_processes import identity
   info=identity(p.name)
   if not info or info['state']=='Z':continue
   env=(p/'environ').read_bytes().split(b'\0')
   if any(('CMR_OWNERSHIP_TOKEN='+t).encode() in env for t in tokens):result.append(info)
  except (PermissionError,FileNotFoundError,ProcessLookupError):pass
 return result


def read_rows(path):return [strict_json(l) for l in Path(path).read_text().splitlines()]


def audit_run(root,intent,exit_code):
 """Gate from original validator plus independent reconstruction of behavior observations."""
 run=Path(root)/'evidence/runs'/intent['run_id']
 out={'run_id':intent['run_id'],'method':intent['method'],'purpose':intent['purpose'],'exit_code':exit_code,'pass':False,'safe_to_resume':False,'actual_start':False,'cold_start':False,'ready':False,'accepted':False,'action':None,'first_reason':None,'contract':'MISSING','bag_valid':False,'cleanup':False,'geometry':'not_evaluated: static-world object geometry scope not validated','collision':None}
 if not run.exists():
  out.update(first_reason='PRESTART_REJECTION',safe_to_resume=exit_code is not None);return out
 try:
  association=strict_json((run/'runtime/batch-association.json').read_text())
  if association!={k:intent[k] for k in ['batch_id','attempt_id','slot_id','run_id']}:raise ValueError('batch association mismatch')
  events=read_rows(run/'logs/events.jsonl')
  starts=[e['data'] for e in events if e['type']=='PROCESS_STARTED' and e['data']['role']=='gazebo']
  out['actual_start']=len(starts)==1
  import tools.contracts.validation as validate_run
  diag=validate_run.validate_run_dir(run);out['contract']=diag['status'];out['diagnostic']=diag
  result=strict_json((run/'result.json').read_text());out['origin']=result.get('record_origin')
  manifest=strict_json((run/'manifest.json').read_text());out['origin']=manifest['record_origin']
  status=result['status'];out['harness']=status['harness'];out['cleanup_status']=status['cleanup'];out.update(ready=status['startup']=='READY',action=status['action'],first_reason=status['primary_reason'])
  cleanup=strict_json((run/'runtime/cleanup.json').read_text());out['remaining_owned']=live_owned(run)
  out['cleanup']=status['cleanup']=='SUCCEEDED' and cleanup['ok'] is True and not out['remaining_owned']
  out['safe_to_resume']=out['cleanup'] and out['contract']=='VALID' and out['origin']=='real'
  bag=strict_json((run/'runtime/bag-check.json').read_text());out['bag_valid']=bag['valid'] is True
  cold=strict_json((run/'runtime/cold-start.json').read_text());out['cold_start']=out['actual_start'] and out['ready'] and cold['valid'] is True and cold['gazebo_identity']==starts[0]['identity'] and cold['partition']=='cmr_'+run.name and cold['clock_epoch']==run.name+':sim0'
  acts=read_rows(run/'logs/action-client.jsonl');accepted=[e for e in acts if e['kind']=='response' and e['data']['accepted']];dispatch=[e for e in acts if e['kind']=='dispatch']
  out['accepted']=len(accepted)==len(dispatch)==1 and accepted[0]['data']['uuid']==dispatch[0]['data']['uuid']
  phases={e['data']:e['parent_monotonic_s'] for e in events if e['type']=='PHASE'}
  terminal=[e for e in acts if e['kind']=='terminal'];timing=strict_json((run/'runtime/timing.json').read_text())
  out['timing']={'startup_wall_s':phases.get('SEND_ONCE',phases.get('SHUTDOWN',0))-timing['entry_monotonic_s'],'accepted_to_terminal_wall_s':terminal[0]['monotonic_s']-accepted[0]['monotonic_s'] if len(terminal)==len(accepted)==1 else None,'accepted_to_terminal_sim_s':terminal[0]['ros_s']-accepted[0]['ros_s'] if len(terminal)==len(accepted)==1 else None,'cleanup_wall_s':phases['INSPECT_BAG']-phases['SHUTDOWN'],'session_wall_s':timing['elapsed_monotonic_s'],'unknown_durations_are_not_zero':True}

  observations=read_rows(run/'runtime/observations.jsonl');profile=strict_json((run/'artifacts/profile.json').read_text())
  if intent['purpose']=='normal':
   from tools.runner.runner_analysis import analyze
   # The runner's own behavior report is not used as the reconstruction input.
   original=analyze(observations,acts,strict_json((run/'artifacts/goal.json').read_text()),strict_json((run/'runtime/all-parameters.json').read_text())['controller_server'],arrival_policy=profile.get('p0_08',{}).get('arrival_policy','last_fresh'))
   out['behavior']=original
   out['pass']=all([out['safe_to_resume'],out['cold_start'],out['bag_valid'],out['accepted'],status['action']=='SUCCEEDED',status['harness']=='COMPLETED',all(original['checks'].values()),exit_code==0])
  else:
   from tools.runner.failure_stop import observe_stop
   stop=strict_json((run/'runtime/failure-stop.json').read_text());req=stop['request']
   recalculated=observe_stop(observations,req['after_monotonic_s'],req['command_after_monotonic_s'],profile['stop'])
   fault=strict_json((run/'runtime/fault-check.json').read_text());out['fault']=fault;out['stop_recomputed']=recalculated
   out['pass']=all([out['safe_to_resume'],out['cold_start'],out['bag_valid'],out['accepted'],fault['case_pass'] is True,all(recalculated['checks'].values()),exit_code==3])
  out['evidence']={str(p.relative_to(run)):digest(p) for p in [run/'manifest.json',run/'result.json',run/'runtime/cleanup.json',run/'runtime/bag-check.json',run/'runtime/cold-start.json']}
 except Exception as exc:out['audit_error']=str(exc);out['first_reason']=out['first_reason'] or type(exc).__name__+': '+str(exc)
 return out


def batch_summary(batch,root):
 batch=Path(batch);manifest=strict_json((batch/'batch-manifest.json').read_text());matrix=strict_json((batch/'matrix-frozen.yaml').read_text());rows=journal_read(batch/'journal.jsonl');records=attempts(rows);runs=[];issues=[]
 for aid,a in records.items():
  row={'attempt_id':aid,**{k:a['intent'][k] for k in ['slot_id','run_id','method','purpose']},'called':a['started'] is not None,'finished':a['finished'] is not None}
  if a['finished']:row['outcome']=audit_run(root,a['intent'],a['finished'].get('exit_code'))
  else:row['outcome']={'pass':False,'actual_start':False,'cold_start':False,'first_reason':'UNRESOLVED'};issues.append('unresolved '+aid)
  runs.append(row)
 claimed={r['run_id'] for r in runs}
 for p in (Path(root)/'evidence/runs').glob('*/runtime/batch-association.json'):
  try:
   v=strict_json(p.read_text())
   if v['batch_id']==manifest['batch_id'] and v['run_id'] not in claimed:issues.append('orphan '+str(p.parent.parent))
  except Exception as exc:issues.append('invalid association '+str(p)+': '+str(exc))
 counts={}
 for method in ['mppi','dwb']:
  subset=[r for r in runs if r['method']==method];outs=[r['outcome'] for r in subset]
  counts[method]={'planned_slots':sum(s['method']==method for s in matrix['slots']),'intents':len(subset),'calls':sum(r['called'] for r in subset),'prestart_rejected':sum(r['finished'] and not r['outcome'].get('actual_start') for r in subset),'actual_starts':sum(o.get('actual_start',False) for o in outs),'valid_cold_starts':sum(o.get('cold_start',False) for o in outs),'ready':sum(o.get('ready',False) for o in outs),'accepted':sum(o.get('accepted',False) for o in outs),'complete_pass':sum(o['pass'] for o in outs),'unresolved':sum(not r['finished'] for r in subset),'actions':{a:sum(o.get('action')==a for o in outs) for a in ['NOT_SENT','UNKNOWN','REJECTED','ABORTED','CANCELED','SUCCEEDED',None]}}
 return {'batch_id':manifest['batch_id'],'purpose':matrix['purpose'],'cohort_sha256':manifest['cohort_sha256'],'counts':counts,'issues':issues,'runs':runs,'collision':None,'personal_review':None}
