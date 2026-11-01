# SPDX-License-Identifier: MIT
"""Additive P1-03 lifecycle hosted by the original Runner parent."""
import json,os,signal,sys,time
from pathlib import Path
from tools.runner.runner_core import DeadlineBudget,write_json
class GateSession:
 def __init__(self,supervisor,run,p,env,log,link,Link):
  self.supervisor=supervisor;self.run=Path(run);self.e=p['execution_v1'];self.env=env;self.link=link;self.log=log;self.owner=None;self.frozen=False;self.allow_clock_pause=False;self.closed=False;self.trigger=None;self.stop=None;self.failure=None;self.activated=False;self.paused_report=None;self.loads=[]
  ns=self.e['namespace'];bins={k:{**v,'path':str(run/'artifacts'/(k+'.binary'))} for k,v in self.e['binaries'].items()};base=['--ros-args','-r','__ns:='+ns,'-r','clock:=/clock','-r','odom:=/odom','-p','use_sim_time:=false']
  self.gate=supervisor.spawn('execution-gate',[bins['execution_gate']['path'],*base,'-p','mode:=simulation','-p','run_id:='+run.name,'-p','owner_epoch:='+self.e['owner_epoch'],'-p','source_node:=collision_monitor','-p','source_namespace:=/','-p','frame:=base_link'],env,run.parents[2])
  self.observer=supervisor.spawn('execution-observer',[bins['execution_observer']['path'],*base,'-p','mode:=simulation'],env,run.parents[2])
  self.relay=None;self.frozen_source=None
  self.owner=supervisor.spawn('authorization-fixture',[sys.executable,'-B',str(run/'artifacts/tools/execution/owner_fixture.py'),'--run-dir',str(run)],env,run.parents[2],ipc=True)
  self.owner_link=Link(self.owner,run.name,log,supervisor);self.link.hooks.append(self.pump)
 def pump(self):
  if not self.closed and not self.frozen:
   self.owner_link.pump(monitor=False)
   if not self.e['fault'] and any(x['type']=='LEASE_LOST' for x in self.owner_link.events) and any(x['type']=='GOAL_SEND' for x in self.link.events) and not any(x['type']=='ACTION_TERMINAL' for x in self.link.events):raise RuntimeError('normal-run lease invalidated; inspect raw gate reason')
 def rpc(self,op,data,seconds=5):return self.link.rpc(op,data,DeadlineBudget.after(seconds))
 def mark(self,name,data=None):return self.rpc('mark',{'name':name,'data':data or {}},3)
 @staticmethod
 def route_qualified(text):
  if 'Publishers' not in text or 'Subscribers' not in text:return False
  pub,sub=text.split('Subscribers',1)
  entries=lambda part:[line.strip() for line in part.splitlines() if line.strip().startswith(('tcp://','ipc://'))]
  publishers=entries(pub);subscribers=entries(sub)
  return len(publishers)==len(subscribers)==1 and all(line.endswith(', gz.msgs.Twist') for line in publishers+subscribers)
 def prepare(self,budget):
  graph=self.link.rpc('execution_graph',{},budget.child(12));write_json(self.run/'runtime/execution-graph.json',graph)
  # Query actual Gazebo bus before authority. Presence is not consumer ACK.
  item=self.supervisor.spawn('drive-graph',['gz','topic','-i','-t',self.e['namespace']+'/drive'],self.env,self.run.parents[2],critical=False)
  item.process.wait(timeout=budget.child(5).require());self.supervisor.close(item,budget,initial_signal=False,grace=.1)
  text=(self.run/'logs/drive-graph.log').read_text()
  if item.process.returncode or not self.route_qualified(text):raise RuntimeError('Gazebo private drive graph unavailable')
  write_json(self.run/'runtime/execution-processes.json',[{'role':i.role,'leader':i.birth,'members':[dict(m,cmdline=(Path('/proc')/str(m['pid'])/'cmdline').read_bytes().replace(b'\0',b' ').decode()) for m in self.supervisor.members(i)]} for i in self.supervisor.owned if not i.closed])
  grant=self.owner_link.rpc('begin',{},budget.child(12));write_json(self.run/'runtime/lease-grant.json',grant)
  if grant['code']!='GRANTED':raise RuntimeError('lease not granted')
  self.mark('EXECUTION_AUTHORIZED',{'generation':grant['generation'],'lease_id':grant['lease_id']})
 def revoke(self,budget):return self.owner_link.rpc('revoke',{},budget.child(3))
 def exercise(self,uid,gz,work):
  kind=self.e['fault']
  try:
   self.trigger=self.link.rpc('fault_ready',{'uuid':uid},work.child(20));self.mark('EXECUTION_FAULT_BEFORE',self.trigger)
   if kind=='revoke':
    response=self.revoke(work);write_json(self.run/'runtime/fault-response.json',response)
    if response['code']!='REVOKED':raise RuntimeError('revoke rejected')
   elif kind in ['owner-loss','owner-loss-load']:
    if kind=='owner-loss-load':
     # Finite, owned, bounded CPU load; no unknown system processes.
     for i in range(2):self.loads.append(self.supervisor.spawn('gate-load-'+str(i),[sys.executable,'-c','import time,math; e=time.monotonic()+12; x=1.0\nwhile time.monotonic()<e: x=math.sin(x)+1.1'],self.env,self.run.parents[2],critical=False))
    self.supervisor.signal_identity(self.owner.birth,signal.SIGSTOP);self.frozen=True
   elif kind=='candidate-stop':
    navigation=next(p for p in self.supervisor.owned if p.role=='navigation')
    matches=[]
    for member in self.supervisor.members(navigation):
     argv=(Path('/proc')/str(member['pid'])/'cmdline').read_bytes().split(b'\0')
     if argv and Path(argv[0].decode()).name=='collision_monitor':matches.append(member)
    if len(matches)!=1:raise RuntimeError('NOT_EXERCISED: unique owned Collision Monitor PID missing')
    self.frozen_source=matches[0];self.supervisor.signal_identity(self.frozen_source,signal.SIGSTOP)
    until=time.monotonic()+.35
    while time.monotonic()<until:self.link.pump()
    self.supervisor.signal_identity(self.frozen_source,signal.SIGCONT);self.frozen_source=None
    self.mark('EXECUTION_CANDIDATE_RESUMED',{'meaning':'resume before lifecycle bond timeout; Gate must remain forbidden'})
   elif kind=='clock-pause':
    self.allow_clock_pause=True;before=time.monotonic();gz.control(True)
    while not gz.paused(True,before):work.require();self.link.pump()
    first_stats=gz.latest['stats']
    # Clock remains paused until A/B and independent D=UNKNOWN are observed.
    until=time.monotonic()+1.0
    while time.monotonic()<until:self.link.pump()
    last_stats=gz.latest['stats']
    if not gz.paused(True,before) or last_stats['receive_monotonic_s']-first_stats['receive_monotonic_s']<.3 or last_stats['protobuf']['sim_time']!=first_stats['protobuf']['sim_time']:raise RuntimeError('pause fault not confirmed by advancing wall stats and held sim time')
    write_json(self.run/'runtime/pause-qualification.json',{'first':first_stats,'last':last_stats})
    self.paused_report=self.rpc('execution_status',{},3);write_json(self.run/'runtime/paused-gate.json',self.paused_report)
    if self.paused_report['state']['allowed'] or self.paused_report['stop']['verdict']=='STOPPED' or self.paused_report['final_nonzero']:raise RuntimeError('paused clock did not fail closed / D reused')
    gz.control(False);resumed=time.monotonic()
    while not gz.paused(False,resumed):work.require();self.link.pump()
    self.link.rpc('sensors',{},work.child(8));self.mark('EXECUTION_RESUMED');self.allow_clock_pause=False
   self.activated=True;self.mark('EXECUTION_FAULT_AFTER',{'fault':kind,'active_goal_uuid':uid})
   # No cancellation or Gazebo stop may cause the measured stop.
   self.stop=self.link.rpc('execution_stop',{'marker':'EXECUTION_RESUMED' if kind=='clock-pause' else 'EXECUTION_FAULT_BEFORE'},work.child(30))
   write_json(self.run/'runtime/execution-fault-stop.json',self.stop)
   if not all(self.stop['checks'].values()):raise RuntimeError('A/B/D not independently established')
   expected={'revoke':'REVOKED','owner-loss':'LEASE_EXPIRED','owner-loss-load':'LEASE_EXPIRED','candidate-stop':'CANDIDATE_EXPIRED'}
   if kind in expected and self.stop.get('gate_reason')!=expected[kind]:raise RuntimeError('fault cause not exercised: '+str(self.stop.get('gate_reason')))
   if kind!='clock-pause':
    latency=self.stop['first_forbidden_receive_s']-self.stop['after_observer_monotonic_s']
    if not 0<=latency<.6:raise RuntimeError('fault detection outside frozen 600ms observation budget')
   if kind=='candidate-stop':
    replies=[json.loads(x)['data'] for x in (self.run/'runtime/authorization-fixture.jsonl').read_text().splitlines() if json.loads(x)['kind']=='response']
    deadlines=[x['deadline_gate_ms'] for x in replies if x['code'] in ['GRANTED','RENEWED']]
    if not deadlines or not self.stop['first_forbidden_gate_ms']<max(deadlines):raise RuntimeError('candidate expiry not independent of valid lease')
    self.stop['last_valid_lease_deadline_gate_ms']=max(deadlines)
   if kind=='owner-loss-load':
    cpus=[{'pid':p.process.pid,'cpu_seconds':sum(int(x) for x in (Path('/proc')/str(p.process.pid)/'stat').read_text().split(') ')[1].split()[11:13])/os.sysconf('SC_CLK_TCK'),'identity':p.birth} for p in self.loads]
    if not all(x['cpu_seconds']>.2 for x in cpus):raise RuntimeError('CPU load not exercised')
    self.stop['owned_cpu_load']=cpus
   write_json(self.run/'runtime/execution-fault-qualification.json',{'fault':kind,'gate_reason':self.stop.get('gate_reason'),'independent_stop_before_cancel':True,'proof':self.stop})
   self.mark('EXECUTION_STOP_PROVED',self.stop)
   self.link.rpc('cancel',{},work.child(8))
  except Exception as ex:self.failure=str(ex);raise
 def normal_stop(self,work):
  self.revoke(work);self.mark('EXECUTION_NORMAL_REVOKED')
  self.stop=self.link.rpc('execution_stop',{'marker':'EXECUTION_NORMAL_REVOKED'},work.child(30));write_json(self.run/'runtime/execution-normal-stop.json',self.stop)
  if not all(self.stop['checks'].values()):raise RuntimeError('normal A/B/D stop unknown')
  self.mark('EXECUTION_STOP_PROVED',self.stop)
 def fail_close(self,gz,work):
  # Preserve bounded stop observation before cancellation/physics shutdown.
  self.closed=True
  if self.frozen_source:self.supervisor.signal_identity(self.frozen_source,signal.SIGCONT);self.frozen_source=None
  if self.pump in self.link.hooks:self.link.hooks.remove(self.pump)
  if self.frozen_source:self.supervisor.signal_identity(self.frozen_source,signal.SIGCONT);self.frozen_source=None
  if gz and gz.owned_pause:gz.control(False)
  if self.frozen:self.supervisor.signal_identity(self.owner.birth,signal.SIGCONT);self.frozen=False;self.owner_link.last_heartbeat=time.monotonic()
  self.revoke(work);self.mark('EXECUTION_FAILURE_REVOKED')
  self.stop=self.link.rpc('execution_stop',{'marker':'EXECUTION_FAILURE_REVOKED'},work.child(30));write_json(self.run/'runtime/execution-failed-stop.json',self.stop)
  self.link.rpc('cancel',{},work.child(8));return self.stop
 def close(self,budget):
  self.closed=True
  if self.frozen_source:self.supervisor.signal_identity(self.frozen_source,signal.SIGCONT);self.frozen_source=None
  if self.pump in self.link.hooks:self.link.hooks.remove(self.pump)
  if self.frozen:self.supervisor.signal_identity(self.owner.birth,signal.SIGCONT);self.frozen=False;self.owner_link.last_heartbeat=time.monotonic()
  if self.owner:
   self.owner.critical=False;self.owner_link.rpc('exit',{},budget.child(4));self.supervisor.close(self.owner,budget,initial_signal=False,grace=3)
  for item in self.loads:self.supervisor.close(item,budget,initial_signal=False,grace=min(12,budget.remaining()))
  for item in [self.gate,self.observer]:self.supervisor.close(item,budget,grace=5)
 def report(self,cleanup,bag,contract):
  return {'version':1,'case':self.e['case'],'run_id':self.run.name,'goal_owner':self.e['goal_owner'],'lease_owner':self.e['lease_owner'],'real_mission_integration':False,'synthetic':False,'fault':self.e['fault'],'fault_activation':'CONFIRMED' if self.activated else ('NOT_EXERCISED' if self.e['fault'] else 'NOT_APPLICABLE'),'trigger':self.trigger,'stop':self.stop,'C':'CONSUMER_ACK_UNKNOWN','cleanup':cleanup,'bag':bool(bag and bag['valid']),'record_contract':contract,'error':self.failure,'case_pass':bool(self.stop and all(self.stop['checks'].values()) and (not self.e['fault'] or self.activated) and not self.failure and cleanup and bag and bag['valid'] and contract=='VALID')}
