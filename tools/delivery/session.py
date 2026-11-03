# SPDX-License-Identifier: MIT
"""Business choreography only; the original Runner owns all processes and budgets."""
import json,signal,time,shlex
from pathlib import Path
from tools.runner.runner_core import write_json
from tools.execution.runner_gate import GateSession
class DeliverySession:
 def __init__(self,supervisor,run,p,env,log,link,Link):
  self.supervisor=supervisor;self.run=Path(run);self.p=p;self.e=p['execution_v1'];self.d=p['delivery_v1'];self.env=env;self.log=log;self.link=link
  self.failure=None;self.stop=None;self.allow_clock_pause=False;self.closed=False;self.owner=None;self.case_done=False;self.activated=False;self.seq=0;self.statuses={};self.proofs=[];self.trigger=None
  self.catalog=json.loads((run/'artifacts/stations.v1.json').read_text());self.ids=[run.name+'_m1',run.name+'_m2']
  base=['--ros-args','-r','__ns:='+self.e['namespace'],'-r','clock:=/clock','-r','odom:=/odom','-p','use_sim_time:=false']
  binary=lambda n:str(run/'artifacts'/(n+'.binary'))
  self.gate=supervisor.spawn('execution-gate',[binary('execution_gate'),*base,'-p','mode:=simulation','-p','run_id:='+run.name,'-p','owner_epoch:='+self.e['owner_epoch'],'-p','source_node:=collision_monitor','-p','source_namespace:=/','-p','frame:=base_link'],env,run.parents[2])
  self.observer=supervisor.spawn('execution-observer',[binary('execution_observer'),*base,'-p','mode:=simulation'],env,run.parents[2])
 def command(self,op,id='',budget=None,**values):
  if budget is None:raise ValueError('explicit owner budget required')
  self.seq+=1;q={'operation':op,'mission_id':id,'command_id':self.run.name+'_c'+str(self.seq),**values}
  result=self.link.rpc('mission_command',q,budget.child(5))
  if result['code'] in ['STORAGE_ERROR','CLOSED','OVERLOADED']:raise RuntimeError('Mission API '+str(result))
  self.log('DELIVERY_API',{'request':q,'response':result})
  if id:self.statuses[id]=result
  return result
 def mark(self,name,data,budget):return self.link.rpc('mark',{'name':name,'data':data},budget.child(3))
 def start_owner(self,budget,restart=False):
  epoch=self.e['owner_epoch']+('_restart' if restart else '')
  params={'workflow_mode':'delivery','test_mode':False,'use_sim_time':False,'execution_mode':'gate','execution_run_id':self.run.name,'execution_namespace':self.e['namespace'],'process_epoch':epoch,'action_name':'/navigate_to_pose','station_catalog':str(self.run/'artifacts/stations.v1.json'),'database':str(self.run/'runtime/mission.db'),'queue_capacity':4,'task_ms':self.d['policy']['task_ms'],'handover_ms':self.d['policy']['handover_ms'],'execute_ms':150000}
  argv=[str(self.run/'artifacts/mission_node.binary'),'--ros-args','-r','__ns:='+self.e['namespace']]
  for k,v in params.items():argv+=['-p',k+':='+(str(v).lower() if isinstance(v,bool) else str(v))]
  self.owner=self.supervisor.spawn('mission-restarted' if restart else 'mission',argv,self.env,self.run.parents[2])
  # A fresh response, not graph discovery alone, establishes replacement readiness.
  readiness=budget.child(12)
  while True:
   try:self.command('query','__readiness__',readiness.child(1));break
   except (RuntimeError,TimeoutError) as exc:
    self.log('RESTART_READINESS_QUERY_ONLY',{'error':str(exc),'no_business_retry':True})
    if self.owner.process.poll() is not None:raise
    readiness.require()
 def prepare(self,budget):
  graph=self.link.rpc('execution_graph',{},budget.child(12));write_json(self.run/'runtime/execution-graph.json',graph)
  item=self.supervisor.spawn('drive-graph',['gz','topic','-i','-t',self.e['namespace']+'/drive'],self.env,self.run.parents[2],critical=False)
  item.process.wait(timeout=budget.child(5).require());self.supervisor.close(item,budget,initial_signal=False,grace=.1)
  if item.process.returncode or not GateSession.route_qualified((self.run/'logs/drive-graph.log').read_text()):raise RuntimeError('private Gazebo drive boundary unqualified')
  self.start_owner(budget)
  write_json(self.run/'runtime/execution-processes.json',[{'role':x.role,'identity':x.birth} for x in self.supervisor.owned])
 def query(self,id,budget):return self.command('query',id,budget)
 def wait(self,id,predicate,budget,description):
  while True:
   budget.require();q=self.query(id,budget)
   if predicate(q):return q
   if q['state'] in ['FAILED','CANCELED','PAUSED'] and not self.d['case'] in ['G03','G04','G06']:raise RuntimeError(description+': unexpected '+str(q))
   until=min(time.monotonic()+.1,budget.end)
   while time.monotonic()<until:self.link.pump()
 def submit(self,id,budget):
  until=budget.child(8)
  while True:
   q=self.command('submit',id,until,schema_version=1,delivery_station='delivery',return_station='home',map_version=self.catalog['map_version'],station_version=self.catalog['station_version'],created_at_unix_ms=1)
   if q['code'] in ['ACCEPTED','QUEUED']:return q
   if q['code']!='INPUT_UNQUALIFIED':raise RuntimeError('submit rejected '+str(q))
   until.require()
   for _ in range(5):self.link.pump()
 def proof(self,label,budget):
  self.mark(label,{},budget)
  proof=self.link.rpc('execution_stop',{'marker':label},budget.child(15))
  self.proofs.append({'marker':label,'proof':proof});write_json(self.run/'runtime'/('delivery-stop-proof-%02d.json'%len(self.proofs)),self.proofs[-1])
  if not all(proof['checks'].values()):raise RuntimeError('independent raw A/B/D missing '+label)
  self.stop=proof;self.mark('DELIVERY_STOP_PROVED',{'label':label,'proof':proof},budget)
  return proof
 def handover(self,id,q,budget):
  if self.d['handover_policy']=='human':
   command=['python3','-B','tools/mission/delivery_command.py','--service',self.e['namespace']+'/delivery','handover','--mission-id',id,'--command-id',self.run.name+'_human','--handover-id',q['handover_id'],'--entry-version',str(q['handover_entry_version']),'--source','human']
   message={'session_id':self.run.name,'state':q,'requires_actual_user':True,'command_after_sourcing_install_and_domain71':shlex.join(command)}
   write_json(self.run/'runtime/human-handover-required.json',message);print('HUMAN_HANDOVER_REQUIRED '+json.dumps(message,ensure_ascii=False),flush=True);self.log('HUMAN_HANDOVER_REQUIRED',message)
   # There is deliberately no RPC here. Only an external user may confirm.
   returned=self.wait(id,lambda x:x['handed_over'] or x['state'] in ['FAILED','PAUSED','CANCELED'],budget,'human handover')
   if not returned['handed_over'] or returned['handover_source']!='human':raise RuntimeError('required human handover not obtained')
  else:
   r=self.command('handover',id,budget,handover_id=q['handover_id'],handover_entry_version=q['handover_entry_version'],source='synthetic')
   if r['code']!='CONFIRMED':raise RuntimeError('handover rejected '+str(r))
 def execute(self,work,gz):
  case=self.d['case'];one,two=self.ids;self.submit(one,work)
  if case in ['G05','G06']:
   q=self.submit(two,work)
   if q['code']!='QUEUED':raise RuntimeError('second order not queued')
  if case in ['G03','G06']:
   q=self.wait(one,lambda x:bool(x['goal_uuid']) and x['state']=='NAVIGATING',work,'accepted moving Goal')
   self.trigger=self.link.rpc('mission_motion_ready',{'uuid':q['goal_uuid']},work.child(25));self.mark('DELIVERY_FAULT_BEFORE',self.trigger,work)
   if case=='G03':
    r=self.command('cancel',one,work)
    if r['code']!='CANCEL_PENDING':raise RuntimeError('cancel not accepted')
    self.activated=True;q=self.wait(one,lambda x:x['state']=='CANCELED',work.child(20),'cancel resolved');self.proof('DELIVERY_CANCEL_RESOLVED',work)
    if not q['schedule_paused']:raise RuntimeError('cancel did not pause scheduling')
   else:
    self.supervisor.expect_mission_fault_exit(self.owner,'G06:actual qualified moving Mission owner exit')
    self.owner.critical=False;self.supervisor.signal_identity(self.owner.birth,signal.SIGKILL);self.activated=True
    self.supervisor.close(self.owner,work.child(4),initial_signal=False,grace=1)
    proof=self.proof('DELIVERY_OWNER_LOST',work)
    if proof['gate_reason']!='LEASE_EXPIRED':raise RuntimeError('owner-loss expiry was not exercised')
    self.start_owner(work,restart=True);q=self.wait(one,lambda x:x['state']=='PAUSED',work.child(8),'restart unresolved');waiting=self.query(two,work)
    if not q['schedule_paused'] or waiting['queue_state']!='WAITING' or waiting['goal_uuid']:raise RuntimeError('restart advanced queue')
    self.mark('DELIVERY_RESTART_OBSERVE',{'record':q,'waiting':waiting},work)
    end=work.child(2)
    while end.remaining()>0:self.link.pump()
    latest=self.query(one,work)
    if latest['goal_uuid']!=q['goal_uuid'] or latest['attempt_id']!=q['attempt_id']:raise RuntimeError('restart auto resend')
   self.case_done=True
  elif case=='G04':
   # The short handover budget starts only at the qualified FSM entry.
   self.wait(one,lambda x:x['state']=='WAIT_HANDOVER',work,'outbound arrival')
   q=self.wait(one,lambda x:x['state']=='FAILED',work.child(15),'handover deadline')
   self.proof('DELIVERY_HANDOVER_TIMEOUT',work)
   if q['handed_over'] or not q['schedule_paused']:raise RuntimeError('timeout confirmed or resumed')
   self.case_done=True
  else:
   for id in [one,two] if case=='G05' else [one]:
    q=self.wait(id,lambda x:x['state']=='WAIT_HANDOVER',work,'outbound arrival and stop')
    self.mark('DELIVERY_WAIT_HANDOVER',{'mission_id':id,'record':q},work)
    self.proof('DELIVERY_OUTBOUND_IDLE_'+id,work)
    q=self.query(id,work)
    # For queued runs, the owner may promote immediately after COMPLETED.
    # Per-leg raw proof is reconstructed from the bag, not a delayed query.
    self.handover(id,q,work);q=self.wait(id,lambda x:x['state']=='COMPLETED',work,'return arrival and stop')
    self.mark('DELIVERY_COMPLETED',{'mission_id':id,'record':q},work)
   self.proof('DELIVERY_FINAL_IDLE',work);self.case_done=True
  write_json(self.run/'runtime/delivery-stop-proofs.json',self.proofs)
  write_json(self.run/'runtime/delivery-case.json',{'case':case,'oracle_stop_assisted':bool(self.d.get('stop_feedback',{}).get('oracle_stop_assisted')),'expected_business_result':self.case_done,'fault_activation':'CONFIRMED' if self.activated else ('NOT_EXERCISED' if case in ['G03','G06'] else 'NOT_APPLICABLE'),'trigger':self.trigger,'statuses':self.statuses,'synthetic_observations':False,'handover_policy':self.d['handover_policy']})
 def safe_finish(self,budget):
  if self.case_done:return
  if not self.owner or self.owner.process.poll() is not None:return
  self.command('pause_schedule',budget=budget)
  for id in self.ids:
   q=self.query(id,budget)
   if q['state'] and q['state'] not in ['COMPLETED','FAILED','CANCELED']:self.command('cancel',id,budget)
  self.proof('DELIVERY_ABNORMAL_FINISH',budget)
 def close(self,budget):
  self.closed=True
  if self.owner and not self.owner.closed:self.supervisor.close(self.owner,budget,grace=5)
  for item in [self.gate,self.observer]:self.supervisor.close(item,budget,grace=5)
 def report(self,cleanup,bag,contract):
  return {'version':1,'case':self.d['case'],'real_mission_integration':True,'oracle_stop_assisted':bool(self.d.get('stop_feedback',{}).get('oracle_stop_assisted')),'synthetic':False,'error':self.failure,'case_pass':bool(self.case_done and not self.failure and cleanup and bag and bag['valid'] and contract=='VALID'),'C':'CONSUMER_ACK_UNKNOWN'}
