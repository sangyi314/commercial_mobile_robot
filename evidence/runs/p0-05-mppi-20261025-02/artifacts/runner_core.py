"""Pure trial state, strict wire protocol, deadlines and durable no-clobber files."""
from dataclasses import dataclass, field, replace
from pathlib import Path
import copy, hashlib, json, math, os, re, tempfile, time
PLUGINS={'mppi':'nav2_mppi_controller::MPPIController','dwb':'dwb_core::DWBLocalPlanner'}
PHASES=['PRECHECK','SNAPSHOT','START_SIM','SENSOR_READY','START_RECORDER','LOCALIZE','NAV_READY','SEND_ONCE','WAIT_RESULT','OBSERVE_STOP','SHUTDOWN','INSPECT_BAG','FINALIZE']
def strict_json(text):
 def pairs(items):
  d={}
  for k,v in items:
   if k in d:raise ValueError('duplicate JSON key')
   d[k]=v
  return d
 return json.loads(text,object_pairs_hook=pairs,parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))
def encode(value):return (json.dumps(value,ensure_ascii=False,allow_nan=False)+'\n').encode()
def run_id(value):
 if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,95}',value):raise ValueError('unsafe run_id')
 return value

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def within(root,path):
 root=Path(root).resolve();p=Path(path).resolve()
 if p==root or root not in p.parents:raise ValueError('path escapes root or names directory root')
 return p

def atomic_write_new(path,data):
 path=Path(path)
 if not isinstance(data,bytes):raise TypeError('bytes required')
 fd,name=tempfile.mkstemp(prefix='.'+path.name+'.',dir=path.parent)
 try:
  with os.fdopen(fd,'wb') as f:f.write(data);f.flush();os.fsync(f.fileno())
  os.link(name,path) # Atomic publication with EEXIST, never replace.
  dfd=os.open(path.parent,os.O_DIRECTORY)
  try:os.fsync(dfd)
  finally:os.close(dfd)
 finally:
  try:os.unlink(name)
  except FileNotFoundError:pass

def write_json(path,value):atomic_write_new(path,encode(value))
@dataclass
class DeadlineBudget:
 end:float
 clock:object=time.monotonic
 @classmethod
 def after(cls,seconds,clock=time.monotonic,parent=None):
  if type(seconds) not in (int,float) or not math.isfinite(seconds) or seconds<0:raise ValueError('bad budget')
  return cls(min(clock()+seconds,parent.end) if parent else clock()+seconds,clock)
 def remaining(self):return max(0.,self.end-self.clock())
 def child(self,seconds):return self.after(seconds,self.clock,self)
 def require(self):
  if self.remaining()<=0:raise TimeoutError('deadline expired')
  return self.remaining()
@dataclass
class FailureLedger:
 first:dict|None=None
 later:list=field(default_factory=list)
 def add(self,code,phase,detail,now):
  item={'code':code,'phase':phase,'detail':str(detail),'monotonic_s':now}
  if self.first is None:self.first=item
  else:self.later.append(item)
@dataclass
class TrialState:
 phase:str='PRECHECK'
 startup:str='FAILED'
 action:str='NOT_SENT'
 harness:str='STARTUP_FAILED'
 cleanup:str='NOT_ATTEMPTED'
 sends:int=0
 uuid:str|None=None
 timeout_decided:bool=False
 ledger:FailureLedger=field(default_factory=FailureLedger)
def reduce_event(state,event,now):
 s=copy.deepcopy(state);kind=event['type'];intents=[]
 if kind=='PHASE':
  new=event['phase']
  if new not in PHASES or PHASES.index(new)<PHASES.index(s.phase):raise ValueError('backward phase')
  s.phase=new
 elif kind=='READY':s.startup='READY';s.harness='COMPLETED'
 elif kind=='DISPATCH':
  if s.startup!='READY' or s.sends:raise ValueError('send not permitted')
  uid=event['uuid']
  if not re.fullmatch('[0-9a-f]{32}',uid) or int(uid,16)==0:raise ValueError('bad UUID')
  s.sends=1;s.uuid=uid;s.action='UNKNOWN';intents=['SEND_ONCE']
 elif kind=='TIMEOUT':
  s.timeout_decided=True;s.harness='NAV_TIMEOUT';s.ledger.add('GOAL_DEADLINE',s.phase,'Terminal not received before deadline',now);intents=['CANCEL_OWN_UUID']
 elif kind=='TERMINAL':
  action=event['action']
  if action not in ['REJECTED','ABORTED','CANCELED','SUCCEEDED','UNKNOWN']:raise ValueError('bad terminal')
  if event.get('uuid')!=s.uuid:raise ValueError('UUID mismatch')
  s.action=action
  late=event.get('deadline') is not None and not now<event['deadline']
  if action=='SUCCEEDED' and (late or s.timeout_decided):
   s.harness='INTERRUPTED';s.ledger.add('LATE_SUCCESS',s.phase,'Success after deadline; real terminal retained',now)
  elif action!='SUCCEEDED':s.ledger.add('ACTION_'+action,s.phase,action,now)
 elif kind=='FAIL':
  s.ledger.add(event['code'],s.phase,event.get('detail',''),now)
  if not s.timeout_decided:s.harness=event.get('harness','INTERNAL_ERROR' if s.startup=='READY' else 'STARTUP_FAILED')
 elif kind=='CLEANUP':
  s.cleanup='SUCCEEDED' if event['ok'] else 'FAILED'
  if not event['ok']:s.ledger.add('CLEANUP_FAILED',s.phase,event.get('detail',''),now)
 elif kind=='INTERRUPT':s.harness='INTERRUPTED';s.ledger.add('INTERRUPTED',s.phase,'User interrupt',now);intents=['CANCEL_OWN_UUID']
 else:raise ValueError('unknown reducer event')
 return s,intents

def validate_envelope(value,identity,last_seq):
 if not isinstance(value,dict) or value.get('protocol_version')!=1 or value.get('run_id')!=identity:raise ValueError('protocol identity')
 if type(value.get('seq')) is not int or value['seq']<=last_seq:raise ValueError('protocol sequence')
 if not isinstance(value.get('request_id'),str) or not isinstance(value.get('type'),str):raise ValueError('protocol fields')
 return value['seq']

def finalize_record(manifest,manifest_hash,state,elapsed,artifacts,plugin=None,metrics=None):
 metrics=metrics or {};refs={a['id']:a for a in artifacts}
 if state.startup=='READY' and (plugin!=PLUGINS[manifest['baseline']['method']] or refs.get('runtime_parameters',{}).get('kind')!='runtime_parameters'):raise ValueError('READY lacks runtime evidence')
 success={'NOT_SENT':None,'UNKNOWN':None,'SUCCEEDED':True,'REJECTED':False,'ABORTED':False,'CANCELED':False}[state.action]
 result={'schema_version':1,'run_id':manifest['run_id'],'record_origin':manifest['record_origin'],'manifest_sha256':manifest_hash,'status':{'startup':state.startup,'action':state.action,'harness':state.harness,'cleanup':state.cleanup,'primary_reason':json.dumps(state.ledger.first,ensure_ascii=False) if state.ledger.first else None},'runtime':{'actual_plugin':plugin,'parameters_ref':'runtime_parameters'} if plugin and 'runtime_parameters' in refs else None,'time':{'clock_epoch':manifest['clock']['clock_epoch'],'sim_start_s':None,'sim_end_s':None,'elapsed_monotonic_s':elapsed,'rtf':None},'metrics':{'action_success':success,'goal_position_error_m':metrics.get('position_error'),'goal_yaw_error_rad':metrics.get('yaw_error'),'stop_observed':metrics.get('stop'),'intervention':None,'delivery_success':None},'evaluation':{'evaluator_version':None,'scope_id':None,'objects':[],'geometry_model':None,'frame_id':None,'clock_epoch':None,'interval_s':None,'coverage':'none','evaluation_valid':False,'invalid_reason':'Independent collision evaluation not performed.','evidence_refs':[],'collision':None,'collision_event':None,'min_signed_clearance_m':None,'geometric_contact':None,'contact_sensor_event':None},'artifacts':artifacts}
 encode(result)
 if state.startup=='FAILED' and state.action not in ['NOT_SENT','REJECTED']:raise ValueError('startup/action invariant')
 if state.harness=='NAV_TIMEOUT' and state.action=='SUCCEEDED':raise ValueError('late success must be INTERRUPTED')
 return result
