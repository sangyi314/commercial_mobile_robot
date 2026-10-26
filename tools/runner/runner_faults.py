"""P0-08 additions hosted by the existing trial parent, not a second runner."""
import json,math,queue,time,os
from pathlib import Path
from tools.runner.runner_core import write_json, encode

FAULTS=('controller-deactivate','short-deadline','pause-gazebo')
POLICY={'qualification_wall_s':20.,'minimum_motion_m':.05,'minimum_samples':3,'natural_response_wall_s':2.,'short_deadline_wall_s':.5,'pause_wall_s':4.,'detect_wall_s':3.,'service_wall_s':1.5,'recovery_wall_s':8.,'stop_wall_s':30.}
def validate_policy(profile,fault):
 cfg=profile.get('p0_08')
 if cfg is None:
  if fault in FAULTS:raise ValueError('P0-08 profile required')
  return
 if cfg!={'version':1,'fault':fault,'policy':POLICY,'raw_gazebo':True,'arrival_policy':'first_fresh_post_terminal_0.5s'}:raise ValueError('unreviewed P0-08 fault/profile policy')
 if fault is not None and fault not in FAULTS:raise ValueError('fault not whitelisted')
 if profile['faults']!=([fault] if fault else []):raise ValueError('fault whitelist mismatch')

class GazeboEvidence:
 def __init__(self,run,partition,log):
  from gz.transport13 import Node,NodeOptions
  from gz.msgs10.world_stats_pb2 import WorldStatistics
  from gz.msgs10.pose_v_pb2 import Pose_V
  from google.protobuf.json_format import MessageToDict
  options=NodeOptions();options.partition=partition;self.node=Node(options);self.run=Path(run);self.log=log;self.queue=queue.Queue();self.seq=0;self.latest={};self.records=[];self.file=(self.run/'runtime/p0-08-gazebo.jsonl').open('x');self.resume_due=None;self.owned_pause=False;self.resume_error=None;self.resume_wall=None
  def callback(kind,msg):self.queue.put((kind,time.monotonic(),MessageToDict(msg,preserving_proto_field_name=True)))
  self.node.subscribe(WorldStatistics,'/world/default/stats',lambda m:callback('stats',m));self.node.subscribe(Pose_V,'/world/default/pose/info',lambda m:callback('pose',m))
 def tick(self):
  while not self.queue.empty():
   kind,wall,msg=self.queue.get_nowait();self.seq+=1;r={'seq':self.seq,'kind':kind,'run_id':self.run.name,'clock_epoch':self.run.name+':sim0','receive_monotonic_s':wall,'protobuf':msg};self.records.append(r);self.latest[kind]=r;self.file.write(encode(r).decode())
  self.file.flush()
  if self.resume_due is not None and time.monotonic()>=self.resume_due:
   self.resume_due=None
   try:self.control(False);self.resume_wall=time.monotonic()
   except Exception as exc:self.resume_error=str(exc);self.log('PAUSE_RECOVERY_FAILED',self.resume_error)
 def control(self,pause):
  from gz.msgs10.world_control_pb2 import WorldControl
  from gz.msgs10.boolean_pb2 import Boolean
  req=WorldControl();req.pause=pause
  self.log('WORLD_CONTROL_REQUEST',{'pause':pause,'monotonic_s':time.monotonic()})
  ok,reply=self.node.request('/world/default/control',req,WorldControl,Boolean,1500)
  self.log('WORLD_CONTROL_RESPONSE',{'pause':pause,'transport':ok,'response':reply.data if ok else None})
  if not ok or not reply.data:raise RuntimeError('world control failed')
  self.owned_pause=pause
 def paused(self,value,after):
  r=self.latest.get('stats');return bool(r and r['receive_monotonic_s']>=after and time.monotonic()-r['receive_monotonic_s']<.5 and r['protobuf'].get('paused',False)==value)
 def cold(self,spawn,process,environment):
  self.tick();r=self.latest.get('pose')
  if not r or time.monotonic()-r['receive_monotonic_s']>.5:raise RuntimeError('cold actual pose missing/stale')
  p=next((p for p in r['protobuf'].get('pose',[]) if p.get('name')=='turtlebot3_waffle'),None)
  if p is None:raise RuntimeError('cold robot identity missing')
  xyz=[p['position'].get(k,0.) for k in 'xyz'];q=p['orientation'];yaw=math.atan2(2*(q.get('w',0)*q.get('z',0)+q.get('x',0)*q.get('y',0)),1-2*(q.get('y',0)**2+q.get('z',0)**2))
  ok=math.dist(xyz,[spawn[k] for k in 'xyz'])<.05 and abs(yaw-spawn['yaw'])<.1
  result={'valid':ok,'gazebo_identity':process.birth,'partition':environment['GZ_PARTITION'],'clock_epoch':self.run.name+':sim0','actual_pose':p,'source_seq':r['seq'],'source_stamp':r['protobuf']['header']['stamp'],'receive_monotonic_s':r['receive_monotonic_s'],'configured_spawn':spawn,'scope':'actual initial Gazebo model pose before goal; new process, not reset'}
  write_json(self.run/'runtime/cold-start.json',result)
  if not ok:raise RuntimeError('cold initial pose mismatch')
  return result
 def close(self):
  if self.owned_pause:
   self.resume_due=None;self.control(False)
  self.tick();self.node.unsubscribe('/world/default/stats');self.node.unsubscribe('/world/default/pose/info');self.tick();self.file.close()

class FaultExercise:
 def __init__(self,run,kind,uid,link,gazebo,work,state,update,set_deadline):
  self.run=Path(run);self.kind=kind;self.uid=uid;self.link=link;self.gz=gazebo;self.work=work;self.state=state;self.update=update;self.set_deadline=set_deadline;self.events=[];self.activated=False;self.gate=None;self.stop=None;self.reason=None;self.stale=None;self.cancel_response=None
  self.file=(self.run/'runtime/fault-events.jsonl').open('x');self.event('PLANNED',{})
 def event(self,name,data):
  e={'seq':len(self.events)+1,'run_id':self.run.name,'event_id':self.run.name+':fault-once','uuid':self.uid,'fault':self.kind,'stage':name,'monotonic_s':time.monotonic(),'sim_time':self.gz.latest.get('stats',{}).get('protobuf',{}).get('sim_time'),'data':data};self.events.append(e);self.file.write(encode(e).decode());self.file.flush();os.fsync(self.file.fileno())
  self.link.send('mark',{'name':'FAULT_'+name,'data':e},time.monotonic()+2)
  return e
 def wait(self,predicate,seconds):
  budget=self.work.child(seconds)
  while not predicate():budget.require();self.link.pump()
 def cancel(self):
  self.event('CANCEL_REQUESTED',{'uuid':self.uid})
  try:self.cancel_response=self.link.rpc('cancel',{},self.work.child(10));self.event('CANCEL_RESPONSE',self.cancel_response)
  except Exception as exc:self.event('CANCEL_UNKNOWN',{'error':str(exc)})
 def execute(self):
  try:
   self.gate=self.link.rpc('fault_ready',{'uuid':self.uid},self.work.child(POLICY['qualification_wall_s']))
   self.event('ARMED',self.gate)
   if self.state().action!='UNKNOWN':raise RuntimeError('NOT_EXERCISED: terminal before injection')
   self.event('REQUESTED',{})
   if self.kind=='controller-deactivate':
    result=self.link.rpc('fault_deactivate',{'uuid':self.uid},self.work.child(5))
    self.activated=result['activated'];self.event('ACTIVATED' if self.activated else 'NOT_EXERCISED',result)
    if not self.activated:raise RuntimeError('NOT_EXERCISED: deactivate')
    self.update({'type':'FAIL','code':'CONTROLLED_CONTROLLER_DEACTIVATE','detail':'actual GetState confirms controller deactivation','harness':'INTERRUPTED'})
    until=time.monotonic()+POLICY['natural_response_wall_s']
    self.wait(lambda:self.state().action!='UNKNOWN' or time.monotonic()>=until,POLICY['natural_response_wall_s']+1)
   elif self.kind=='short-deadline':
    due=time.monotonic()+POLICY['short_deadline_wall_s'];self.set_deadline(due);self.event('DEADLINE_ARMED',{'deadline_monotonic_s':due})
    self.wait(lambda:self.state().action!='UNKNOWN' or time.monotonic()>=due,2)
    if self.state().action!='UNKNOWN':raise RuntimeError('NOT_EXERCISED: terminal before short deadline')
    self.activated=True;self.event('ACTIVATED',{'deadline_monotonic_s':due});self.update({'type':'TIMEOUT'})
   elif self.kind=='pause-gazebo':
    started=time.monotonic();self.gz.resume_due=started+POLICY['pause_wall_s'];self.gz.control(True)
    self.wait(lambda:self.gz.paused(True,started),1.5)
    first=self.gz.latest['stats'];first_stamp=first['protobuf']['sim_time'];self.wait(lambda:self.gz.latest['stats']['receive_monotonic_s']>=first['receive_monotonic_s']+.3,1)
    second=self.gz.latest['stats']
    if second['protobuf']['sim_time']!=first_stamp or not second['protobuf'].get('paused'):raise RuntimeError('pause actual stats did not hold')
    self.activated=True;self.event('ACTIVATED',{'stats_seqs':[first['seq'],second['seq']],'sim_time':first_stamp,'resume_due_monotonic_s':self.gz.resume_due})
    self.wait(lambda:self.stale is not None,POLICY['detect_wall_s'])
    if not started<=self.stale['source_monotonic_s']<=started+POLICY['detect_wall_s']:raise RuntimeError('stale event outside injection detection interval')
    self.event('DETECTED',self.stale)
   if self.state().action=='UNKNOWN':self.cancel()
   if self.kind=='pause-gazebo':
    self.wait(lambda:self.gz.resume_wall is not None or self.gz.resume_error is not None,8)
    if self.gz.resume_error:raise RuntimeError(self.gz.resume_error)
    self.wait(lambda:self.gz.paused(False,self.gz.resume_wall),3)
    self.link.rpc('sensors',{},self.work.child(POLICY['recovery_wall_s']));self.event('RESUMED',{'resume_monotonic_s':self.gz.resume_wall,'stats_seq':self.gz.latest['stats']['seq']})
   until=time.monotonic()+5;self.wait(lambda:self.state().action!='UNKNOWN' or time.monotonic()>=until,6)
   self.event('OBSERVED',{'terminal':self.state().action,'cancel_response_is_not_terminal':True})
  except Exception as exc:self.reason=str(exc);self.event('ERROR',{'error':str(exc)});raise
  finally:
   # Even a detection/cancel failure must not strand a pause owned by this run.
   if self.gz.owned_pause:
    self.wait(lambda:not self.gz.owned_pause or self.gz.resume_error is not None,6)
 def observe(self):
  if not self.gate:return None
  self.stop=self.link.rpc('failure_stop',{'command_after_monotonic_s':self.gate['monotonic_s'],'command_after_sim_s':self.gate['sim_s']},self.work.child(POLICY['stop_wall_s']))
  self.event('FINISHED',{'stop_checks':self.stop['checks'],'terminal':self.state().action})
  return self.stop
 def report(self,cleanup,bag):
  self.file.close();known=self.state().action in ['SUCCEEDED','ABORTED','CANCELED','REJECTED']
  return {'version':1,'run_id':self.run.name,'fault':self.kind,'uuid':self.uid,'activated':self.activated,'trigger':self.gate,'events':self.events,'terminal':self.state().action,'stop':self.stop,'cleanup':cleanup,'bag_valid':bool(bag and bag['valid']),'not_exercised_reason':self.reason if not self.activated else None,'case_pass':bool(self.activated and self.reason is None and self.stop and all(self.stop['checks'].values()) and known and cleanup and bag and bag['valid']),'collision':None}
