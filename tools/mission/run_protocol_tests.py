#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Isolated ROS Action protocol supervisor; --check has no ROS/filesystem writes."""
import argparse,json,os,signal,sqlite3,subprocess,sys,time,uuid,shlex,hashlib
from pathlib import Path

def check_args(args):
 suite=json.loads(Path(args.suite).read_text())
 if suite.get('domain')!=72 or not suite.get('synthetic') or suite.get('version')!=1:raise ValueError('invalid suite contract')
 expected={f'A{i:02}' for i in range(1,21)}
 if {c['id'] for c in suite['cases']}!=expected:raise ValueError('incomplete matrix')
 prefix=Path(args.install_prefix).resolve();bin_dir=prefix/'cmr_mission/lib/cmr_mission'
 for name in ['mission_node','fake_nav2_server']:
  if not (bin_dir/name).is_file():raise ValueError('missing executable '+name)
 if Path(args.output_dir).exists():raise ValueError('output exists; refuse overwrite')
 return suite,prefix,bin_dir

class Case:
 def __init__(self,root,bin_dir,name,config=None,start_server=True,start_mission=True):
  self.root=root;root.mkdir(parents=True,exist_ok=False);self.bin=bin_dir;self.name=name;self.ns='/cmr_test/'+name;self.action=self.ns+'/navigate_to_pose';self.config=config or {};self.procs=[];self.seq=0;self.records=[];self.node=rclpy.create_node('supervisor_'+name);self.db=root/'mission.db';self.server=None;self.mission=None
  self.api=self.node.create_client(MissionCommand,self.ns+'/command');self.fixture=self.node.create_client(SyntheticEvent,self.ns+'/synthetic_event');self.control=self.node.create_client(FakeControl,self.ns+'/control')
  self.scan_graph()
  if start_server:self.start_server()
  if start_mission:self.start_mission()
 def scan_graph(self):
  end=time.monotonic()+0.35
  while time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=0.03)
  for n,ns in self.node.get_node_names_and_namespaces():
   full=(ns.rstrip('/')+'/'+n)
   if n.startswith('supervisor_'):continue
   if ns!=self.ns:raise RuntimeError('foreign node on isolated domain: '+full)
  for topic,types in self.node.get_topic_names_and_types():
   if topic in ['/cmd_vel','/cmd_vel_nav','/cmd_vel_smoothed'] and self.node.count_subscribers(topic):raise RuntimeError('unexpected motion consumer')
 def validate_action_graph(self,action=None):
  target=(action or self.action)+'/_action/send_goal';providers=[]
  for node_name,node_ns in self.node.get_node_names_and_namespaces():
   for service,types in self.node.get_service_names_and_types_by_node(node_name,node_ns):
    if service==target:
     if types!=['nav2_msgs/action/NavigateToPose_SendGoal']:raise RuntimeError('action service type mismatch: '+str(types))
     providers.append((node_name,node_ns))
  if len(providers)>1:raise RuntimeError('extra Action Server: '+str(providers))
  return providers
 def start(self,name,params,tag):
  log=(self.root/(tag+'.log')).open('w');cmd=[str(self.bin/name),'--ros-args','-r','__ns:='+self.ns]
  for k,v in params.items():cmd+=['-p',k+':='+str(v).lower() if isinstance(v,bool) else k+':='+str(v)]
  env=os.environ.copy();env.update(ROS_DOMAIN_ID='72',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
  p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=env);identity=Path('/proc/'+str(p.pid)+'/stat').read_text().split(') ')[1].split()[19];p.expected_exit=0;self.procs.append([p,log,tag,identity]);self.records.append({'spawn':cmd,'pid':p.pid,'start_ticks':identity});return p
 def start_server(self,tag='server'):
  self.server=self.start('fake_nav2_server',{'action_name':self.action},tag);self.wait_service(self.control)
  if tag!='server-extra':self.validate_action_graph()
 def start_mission(self,tag='mission',epoch=None):
  params={'action_name':self.action,'database':str(self.db),'test_mode':True,'process_epoch':epoch or self.name,'readiness_ms':1000,'accept_ms':1000,'execute_ms':7000,'cancel_ms':1000};params.update(self.config)
  self.mission=self.start('mission_node',params,tag)
  # A graph entry can outlive a killed service. Establish a new response path;
  # do not equate wait_for_service(old endpoint) with new process readiness.
  if tag!='mission':
   self.node.destroy_client(self.api);self.node.destroy_client(self.fixture)
   self.api=self.node.create_client(MissionCommand,self.ns+'/command');self.fixture=self.node.create_client(SyntheticEvent,self.ns+'/synthetic_event')
  end=time.monotonic()+10
  while time.monotonic()<end:
   if not self.api.wait_for_service(timeout_sec=0.05):
    rclpy.spin_once(self.node,timeout_sec=0.02);continue
   q=MissionCommand.Request();q.operation='query';q.mission_id='__readiness__'
   f=self.api.call_async(q);until=time.monotonic()+0.6
   while not f.done() and time.monotonic()<until:rclpy.spin_once(self.node,timeout_sec=0.02)
   if f.done() and f.result() is not None:return
   self.api.remove_pending_request(f)
  raise RuntimeError('new Mission service did not respond')
 def wait_service(self,client,seconds=10):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   if client.wait_for_service(timeout_sec=0.03):return
   rclpy.spin_once(self.node,timeout_sec=0.01)
  raise RuntimeError('service not ready: '+client.srv_name)
 def wait(self,predicate,seconds=7):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   if predicate():return
   rclpy.spin_once(self.node,timeout_sec=0.02)
  raise RuntimeError('condition timeout')
 def call(self,client,request,seconds=8):
  f=client.call_async(request);self.wait(f.done,seconds);result=f.result()
  if result is None:raise RuntimeError('service returned no response')
  self.records.append({'request':message_to_ordereddict(request),'response':message_to_ordereddict(result),'time_monotonic':time.monotonic(),'service':client.srv_name});return result
 def request(self,op='query',id='m',command_id=None,**fields):
  q=MissionCommand.Request();q.operation=op;q.mission_id=id;q.command_id=command_id or 'c'+str(self.seq);self.seq+=1;q.schema_version=1;q.delivery_station='a';q.return_station='b';q.map_version='map';q.station_version='stations';q.created_at_unix_ms=1
  for k,v in fields.items():setattr(q,k,v)
  return q
 def cmd(self,op='query',id='m',**fields):return self.call(self.api,self.request(op,id,**fields))
 def ctrl(self,op,value=0,goal=''):
  q=FakeControl.Request();q.command=op;q.case_id=self.name;q.goal_uuid=goal;q.value_ms=value;return self.call(self.control,q)
 def query(self):return self.cmd()
 def expect_state(self,state):self.wait(lambda:self.query().state==state)
 def accepted(self):self.wait(lambda:bool(self.query().goal_uuid));return self.query()
 def fixture_event(self,kind,**fields):
  r=self.query();tokens=shlex.split(r.detail);q=SyntheticEvent.Request();q.event_id='fixture_'+str(self.seq);self.seq+=1;q.mission_id='m';q.kind=kind;q.epoch=tokens[10];q.operation_id=r.operation_id;q.attempt_id=r.attempt_id;q.goal_uuid=r.goal_uuid;q.gate_epoch='synthetic-gate';q.clock_epoch='synthetic-clock';q.valid_for_ms=5000;q.synthetic=True
  for k,v in fields.items():setattr(q,k,v)
  return self.call(self.fixture,q)
 def event_count(self,kind=None,contains=None):
  if not self.db.exists():return 0
  with sqlite3.connect(self.db,timeout=1) as db:
   if contains:return db.execute('SELECT count(*) FROM events WHERE identity LIKE ?',('%'+contains+'%',)).fetchone()[0]
   return db.execute('SELECT count(*) FROM events WHERE kind=?',(str(kind),)).fetchone()[0]
 def wait_result(self):self.wait(lambda:self.event_count(2)+self.event_count(3)+self.event_count(18)>0)
 def finish_leg(self,returning=False):
  assert self.ctrl('success').success;self.wait_result()
  assert self.fixture_event('stop').code=='APPLIED';assert self.fixture_event('arrival').code=='APPLIED';assert self.fixture_event('resolve').code=='APPLIED'
  self.expect_state('COMPLETED' if returning else 'WAIT_HANDOVER')
 def send_count(self):
  return sum(p.read_text().count('GOAL_REQUEST ') for p in self.root.glob('server*.log'))
 def snapshot(self):
  if self.db.exists():
   with sqlite3.connect(self.db) as db:self.records.append({'database':{t:db.execute('SELECT * FROM '+t).fetchall() for t in ['missions','manager','events','operations']}})
 def injected_exit(self,p):
  assert p and p.poll() is None;p.expected_exit=-signal.SIGKILL;os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5);self.records.append({'fault':'owned_process_SIGKILL','pid':p.pid,'exit':p.returncode});return p.returncode
 def close(self):
  cleanup=[]
  for p,log,tag,identity in reversed(self.procs):
   if p.poll() is None:
    assert Path('/proc/'+str(p.pid)+'/stat').read_text().split(') ')[1].split()[19]==identity
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=7)
    except subprocess.TimeoutExpired:
     cleanup.append({'tag':tag,'escalation':'TERM'});os.killpg(p.pid,signal.SIGTERM)
     try:p.wait(timeout=3)
     except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3);cleanup.append({'tag':tag,'escalation':'KILL'})
   cleanup.append({'tag':tag,'exit':p.returncode,'pid':p.pid,'expected_exit':p.expected_exit});log.close()
  self.snapshot();(self.root/'trace.json').write_text(json.dumps(self.records,indent=2));(self.root/'cleanup.json').write_text(json.dumps(cleanup,indent=2));self.node.destroy_node()
  assert not any('escalation' in x for x in cleanup),'forced cleanup'
  assert all(x.get('exit')==x.get('expected_exit') for x in cleanup),'unexpected process exit'

def run_matrix(args,suite,bin_dir):
 root=Path(args.output_dir);root.mkdir(parents=True,exist_ok=False);results=[]
 (root/'manifest.json').write_text(json.dumps({'suite_sha256':hashlib.sha256(Path(args.suite).read_bytes()).hexdigest(),'binaries':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in bin_dir.iterdir() if p.is_file()},'domain':72,'loopback':True,'synthetic':True,'install_prefix':str(bin_dir.parent.parent.parent)},indent=2))
 selected=args.cases.split(',') if args.cases else [c['id'] for c in suite['cases']]
 def execute(id,fn,config=None,start_server=True):
  name='p102_'+uuid.uuid4().hex[:12]+'_'+id;case=None;error=None
  try:
   case=Case.__new__(Case);case.__init__(root/id,bin_dir,name,config,start_server);fn(case)
  except Exception as e:error=repr(e)
  finally:
   if case:
    try:case.close()
    except Exception as e:error=(error+'; ' if error else '')+repr(e)
  row={'id':id,'status':'FAIL' if error else 'PASS','layer':'adapter-unit+ROS fixture' if id=='A11' else 'ROS+owner+SQLite','error':error,'startup':'PASS' if case and hasattr(case,'mission') and case.mission else 'FAIL','protocol':'ADAPTER_UNIT_AND_FIXTURE' if id=='A11' else ('PASS' if not error else 'FAIL'),'mission':'PASS' if not error else 'UNCONFIRMED','fault_activation':('CONFIRMED' if case and any('fault' in entry for entry in case.records) else 'NOT_EXERCISED') if error else ('NOT_APPLICABLE' if id in ['A01','A02','A17'] else 'CONFIRMED'),'harness':'PASS' if not error else 'FAIL','cleanup':'PASS' if case and all(p.poll() is not None for p,log,tag,identity in case.procs) else 'FAIL','evidence':str(root/id)};results.append(row);(root/'results.json').write_text(json.dumps(results,indent=2));print(json.dumps(row),flush=True)
  if error:raise RuntimeError(id+': '+error)
 def A01(c):
  assert c.cmd('submit').code=='ACCEPTED';c.accepted();c.finish_leg();q=c.query();assert c.cmd('handover',command_id='handover1',expected_state_version=q.state_version).code=='CONFIRMED';c.wait(lambda:c.query().state=='RETURNING');assert c.cmd('handover',command_id='handover1',expected_state_version=q.state_version).code=='CONFIRMED';assert c.cmd('handover',command_id='handover1',expected_state_version=q.state_version+1).code=='COMMAND_CONFLICT'
  old=c.event_count(2);assert c.ctrl('success').success;c.wait(lambda:c.event_count(2)>old);assert c.fixture_event('stop').code=='APPLIED';assert c.fixture_event('arrival').code=='APPLIED';assert c.fixture_event('resolve').code=='APPLIED';c.expect_state('COMPLETED');assert c.cmd('cancel',command_id='after_complete').code=='ALREADY_TERMINAL';assert c.send_count()==2
 def A02(c):
  assert c.cmd('submit',id='bad_schema',schema_version=99).code=='INVALID'
  assert c.cmd('submit',id='bad_station',delivery_station='missing').code=='INVALID_STATION_VERSION'
  assert c.cmd('submit',id='bad id').code=='INVALID'
  assert c.send_count()==0
  # Deliberately withhold client executor dispatch until its application deadline.
  # Independent SQLite confirms commit; this models a lost/late reply, not a
  # transport failure or a rollback. remove_pending_request is the Jazzy public API.
  request=c.request('submit',command_id='reply_timeout');future=c.api.call_async(request)
  deadline=time.monotonic()+0.2
  while time.monotonic()<deadline:time.sleep(0.005)
  assert not future.done()
  with sqlite3.connect(c.db) as db:
   assert db.execute("SELECT count(*) FROM missions WHERE id='m'").fetchone()[0]==1
  c.api.remove_pending_request(future)
  c.records.append({'fault':'client_reply_deadline_after_commit','request':message_to_ordereddict(request),'client_deadline_ms':200,'response_dispatch_intentionally_withheld':True,'time_monotonic':time.monotonic()})
  assert c.cmd('submit',command_id='reply_timeout').code=='EXISTING';c.accepted()
  assert c.cmd('cancel',command_id='bad id').code=='INVALID_COMMAND_ID'
  assert c.cmd('submit',created_at_unix_ms=99).code=='EXISTING';assert c.cmd('submit',return_station='a').code=='CONFLICT';assert c.cmd('submit',id='other').code=='BUSY';assert c.send_count()==1
 def A03(c):
  assert c.cmd('submit').code=='ACCEPTED';c.wait(lambda:c.event_count(contains='NOT_SENT READINESS_TIMEOUT')>0);assert not c.query().goal_uuid and c.send_count()==0
 def A04(c):
  assert c.ctrl('reject',1).success;c.cmd('submit');c.wait(lambda:c.event_count(contains='ACTION_REJECTED')>0);assert c.query().state=='PAUSED' and not c.query().goal_uuid and c.send_count()==1
 def A05(c):
  c.cmd('submit');c.accepted();c.wait(lambda:c.event_count(contains='FEEDBACK')>0);assert c.query().state=='NAVIGATING';c.ctrl('abort');c.wait(lambda:c.event_count(3)>0);assert c.query().pending=='FAILED';assert c.cmd('submit',id='other').code=='BUSY'
 def A06(c):
  c.ctrl('cancel_delay',800);c.cmd('submit');c.accepted();assert c.cmd('cancel',command_id='cancel').code=='CANCEL_PENDING';c.wait(lambda:c.event_count(contains='CANCEL_RESPONSE')>0);assert c.cmd('submit',id='other').code=='BUSY';c.wait(lambda:c.event_count(18)>0);assert c.fixture_event('stop').code=='APPLIED';assert c.fixture_event('resolve').code=='APPLIED';c.expect_state('CANCELED');assert c.send_count()==1
 def A07(c):
  c.ctrl('reject_cancel',1);c.cmd('submit');c.accepted();c.cmd('cancel');c.wait(lambda:c.event_count(contains='CANCEL_RESPONSE')>0);assert c.query().state=='PAUSED' and c.cmd('submit',id='other').code=='BUSY';assert c.send_count()==1
 def A08(c):
  c.ctrl('delay_accept',3000);c.cmd('submit');c.wait(lambda:'ACCEPT_BARRIER' in (c.root/'server.log').read_text());c.cmd('cancel');assert not c.query().goal_uuid;c.ctrl('release_accept');c.wait(lambda:'CANCEL_REQUEST uuid=' in (c.root/'server.log').read_text());c.wait(lambda:c.event_count(18)>0);assert c.send_count()==1
 def A09(c):
  c.ctrl('reject_cancel',1);c.cmd('submit');c.accepted();c.ctrl('success');c.wait_result();assert c.cmd('cancel',command_id='cancel_after_result').code=='CANCEL_PENDING';assert c.query().pending=='CANCELED';c.fixture_event('stop');c.fixture_event('resolve');c.expect_state('CANCELED')
  assert c.cmd('submit',id='m2').code=='ACCEPTED'
  c.wait(lambda:c.cmd('query',id='m2').state=='NAVIGATING');assert c.cmd('cancel',id='m2',command_id='before_result').code=='CANCEL_PENDING';previous=c.event_count(2);assert c.ctrl('success').success
  c.wait(lambda:c.event_count(2)>previous);c.wait(lambda:c.cmd('query',id='m2').pending=='CANCELED');assert c.send_count()==2
 def A10(c):
  c.ctrl('delay_accept',3000);c.cmd('submit');c.wait(lambda:c.event_count(contains='ACCEPT_TIMEOUT')>0);assert c.query().state=='PAUSED';c.ctrl('release_accept');c.wait(lambda:'CANCEL_REQUEST uuid=' in (c.root/'server.log').read_text());c.wait(lambda:c.event_count(18)>0);assert c.send_count()==1
  assert c.fixture_event('stop').code=='APPLIED';assert c.fixture_event('resolve').code=='APPLIED';c.expect_state('FAILED')
  c.ctrl('delay_accept',0);c.ctrl('reject_cancel',1);assert c.cmd('submit',id='m2').code=='ACCEPTED'
  c.wait(lambda:c.cmd('query',id='m2').state=='NAVIGATING')
  c.wait(lambda:c.event_count(contains='EXECUTION_TIMEOUT')>0)
  previous=c.event_count(2);assert c.ctrl('success').success;c.wait(lambda:c.event_count(2)>previous)
  q=c.cmd('query',id='m2');assert q.state=='PAUSED' and q.pending=='FAILED' and c.send_count()==2
 def A11(c):
  p=subprocess.run([str(c.bin/'action_correlation_test')],capture_output=True,text=True,timeout=10)
  (c.root/'adapter-unit.log').write_text(p.stdout+p.stderr);assert p.returncode==0
  c.cmd('submit');c.accepted();before=c.query().state
  assert c.fixture_event('stop',epoch='stale_epoch').code=='IDENTITY'
  assert c.fixture_event('stop',goal_uuid='wrong_uuid').code=='OPERATION'
  assert c.query().state==before and c.send_count()==1
 def A12(c):
  c.cmd('submit');c.accepted();c.injected_exit(c.server);c.wait(lambda:c.query().state=='PAUSED',5);c.start_server('server-restarted');time.sleep(0.3);assert c.send_count()==1 and c.query().state=='PAUSED'
 def A13(c):
  c.ctrl('delay_accept',3000);c.cmd('submit');c.wait(lambda:'ACCEPT_BARRIER' in (c.root/'server.log').read_text());c.injected_exit(c.mission);c.ctrl('release_accept');c.start_mission('mission-restarted',c.name+'_new');assert c.query().state=='PAUSED';time.sleep(0.3);assert c.send_count()==1 and not c.query().goal_uuid
 def A14(c):
  from rosgraph_msgs.msg import Clock
  pub=c.node.create_publisher(Clock,'/clock',10)
  c.wait(lambda:pub.get_subscription_count()>=1)
  clock=Clock();clock.clock.sec=10;pub.publish(clock)
  c.wait(lambda:'ROS_CLOCK_NS 10000000000' in (c.root/'mission.log').read_text())
  c.records.append({'fault':'clock_10_seconds_observed_by_Mission'})
  c.cmd('submit');c.accepted();clock.clock.sec=2;pub.publish(clock)
  c.wait(lambda:'ROS_CLOCK_NS 2000000000' in (c.root/'mission.log').read_text())
  c.records.append({'fault':'clock_rollback_to_2_seconds_observed_by_Mission'})
  c.wait(lambda:c.event_count(contains='EXECUTION_TIMEOUT')>0,5);assert c.query().state=='PAUSED' and c.send_count()==1
 def A15(c):
  c.cmd('submit');c.accepted();c.ctrl('success');c.wait_result();assert c.fixture_event('resolve').code!='APPLIED';assert c.fixture_event('stop',valid_for_ms=0).code=='STOP_EVIDENCE';assert c.fixture_event('stop',epoch='wrong').code=='IDENTITY';assert c.cmd('submit',id='other').code=='BUSY'
 def A16(c):
  with sqlite3.connect(c.db) as db:db.executescript("CREATE TRIGGER fail BEFORE INSERT ON events BEGIN SELECT RAISE(ABORT,'test storage failure'); END;")
  assert c.cmd('submit').code=='STORAGE_ERROR';assert c.send_count()==0
  with sqlite3.connect(c.db) as db:
   assert db.execute('SELECT count(*) FROM missions').fetchone()[0]==0
   db.execute('DROP TRIGGER fail')
  c.injected_exit(c.mission);c.start_mission('mission-after-db-fault',c.name+'_new')
  c.cmd('submit');c.accepted()
  q=SyntheticEvent.Request();q.kind='owner_pause';q.valid_for_ms=800;q.synthetic=True
  pause=c.fixture.call_async(q)
  c.wait(lambda:'OWNER_PAUSE_BEGIN' in (c.root/'mission-after-db-fault.log').read_text())
  futures=[c.api.call_async(c.request('query')) for _ in range(8)]
  assert c.ctrl('success').success
  codes=[]
  for f in futures:
   c.wait(f.done);codes.append(f.result().code)
  c.wait(pause.done);assert 'OVERLOADED' in codes
  (c.root/'overload-codes.json').write_text(json.dumps(codes))
  c.wait(lambda:c.event_count(2)>0);assert c.send_count()==1
 def A17(c):
  c.cmd('submit');c.accepted();import random
  clients=[c.node.create_client(MissionCommand,c.ns+'/command') for _ in range(3)]
  for client in clients:c.wait_service(client)
  inputs=[]
  if args.replay_input:
   inputs=[json.loads(line) for line in Path(args.replay_input).read_text().splitlines()]
   if args.replay_limit:inputs=inputs[:args.replay_limit]
  else:
   for seed in range(20):
    rng=random.Random(seed)
    for index in range(200):
     op=rng.choice(['query','submit','cancel','handover','conflict','busy']);fields={};mission='m'
     if op=='conflict':op='submit';fields['return_station']='a'
     if op=='busy':op='submit';mission='other'
     request=c.request(op,mission,command_id='s'+str(seed)+'e'+str(index),**fields)
     inputs.append({'seed':seed,'index':index,'client':index%3,'request':message_to_ordereddict(request)})
  eventfile=(c.root/'replay-input.jsonl').open('w')
  for begin in range(0,len(inputs),10):
   pending=[]
   for entry in inputs[begin:begin+10]:
    request=MissionCommand.Request()
    for key,value in entry['request'].items():setattr(request,key,value)
    eventfile.write(json.dumps(entry)+'\n');eventfile.flush()
    pending.append((entry,request,clients[entry['client']].call_async(request)))
   for entry,request,future in pending:
    c.wait(future.done);response=future.result();assert response is not None
    c.records.append({'request':message_to_ordereddict(request),'response':message_to_ordereddict(response),'service':c.ns+'/command','seed':entry['seed'],'index':entry['index'],'time_monotonic':time.monotonic()})
    assert response.code not in ['ACCEPTED','STORAGE_ERROR']
   with sqlite3.connect(c.db) as db:assert db.execute('SELECT count(*) FROM missions WHERE terminal=0').fetchone()[0]==1
  eventfile.close();assert c.send_count()==1
  (c.root/'interleaving-summary.json').write_text(json.dumps({'events':len(inputs),'seeds':sorted({x['seed'] for x in inputs}),'clients':3,'replayed':bool(args.replay_input)},indent=2))
 def A18(c):
  # Shutdown while the submit is queued behind owner work, before any reply.
  pause_request=SyntheticEvent.Request();pause_request.kind='owner_pause';pause_request.valid_for_ms=800;pause_request.synthetic=True
  paused=c.fixture.call_async(pause_request)
  c.wait(lambda:'OWNER_PAUSE_BEGIN' in (c.root/'mission.log').read_text())
  queued_request=c.request('submit',command_id='pending_shutdown');queued=c.api.call_async(queued_request)
  c.wait(lambda:'command=pending_shutdown' in (c.root/'mission.log').read_text())
  os.killpg(c.mission.pid,signal.SIGINT)
  c.records.append({'fault':'shutdown_with_pending_submit','time_monotonic':time.monotonic()})
  c.wait(queued.done,5);c.records.append({'request':message_to_ordereddict(queued_request),'response':message_to_ordereddict(queued.result()),'time_monotonic':time.monotonic(),'service':c.api.srv_name});assert queued.result().code=='CLOSED'
  c.wait(paused.done,5);c.wait(lambda:c.mission.poll() is not None,5)
  assert c.mission.returncode==0 and c.send_count()==0
  with sqlite3.connect(c.db) as db:assert db.execute('SELECT count(*) FROM missions').fetchone()[0]==0
  c.start_mission('mission-after-pending',c.name+'_pending')
  c.ctrl('delay_accept',700);c.cmd('submit');c.wait(lambda:'ACCEPT_BARRIER' in (c.root/'server.log').read_text());os.killpg(c.mission.pid,signal.SIGINT);c.wait(lambda:c.mission.poll() is not None,5);assert c.mission.returncode==0;assert c.send_count()==1
  for cycle in range(2):
   c.start_mission('mission-cycle-'+str(cycle),c.name+'_cycle'+str(cycle));assert c.query().state=='PAUSED'
   os.killpg(c.mission.pid,signal.SIGINT);c.wait(lambda:c.mission.poll() is not None,5);assert c.mission.returncode==0
  assert c.send_count()==1
 def A19(c):
  c.start_server('server-extra')
  c.wait(lambda:sum(n=='fake_nav2_server' and ns==c.ns for n,ns in c.node.get_node_names_and_namespaces())>=2)
  try:c.validate_action_graph();raise AssertionError('duplicate server was accepted')
  except RuntimeError as error:assert 'extra Action Server' in str(error)
  c.cmd('submit');c.wait(lambda:c.event_count(contains='ISOLATION_EXTRA_SERVER')>0);assert c.send_count()==0
  wrong=c.node.create_service(FakeControl,c.ns+'/wrong/_action/send_goal',lambda q,r:r)
  c.wait(lambda:any(name==c.ns+'/wrong/_action/send_goal' for name,types in c.node.get_service_names_and_types()))
  types=dict(c.node.get_service_names_and_types())[c.ns+'/wrong/_action/send_goal'];assert 'cmr_interfaces/srv/FakeControl' in types and 'nav2_msgs/action/NavigateToPose_SendGoal' not in types
  try:c.validate_action_graph(c.ns+'/wrong');raise AssertionError('wrong type accepted')
  except RuntimeError as error:assert 'type mismatch' in str(error)
  (c.root/'wrong-type.json').write_text(json.dumps(types));c.node.destroy_service(wrong)
  for tag,params in [('invalid-config',{'test_mode':True,'database':str(c.root/'unused.db'),'process_epoch':'invalid','action_name':'/outside/navigate_to_pose'}),('production',{'test_mode':False}),('unknown-option',{'test_mode':True,'action_name':c.action,'database':str(c.root/'unused-unknown.db'),'process_epoch':'unknown','unexpected_option':1})]:
   p=c.start('mission_node',params,tag);p.expected_exit=2;p.wait(timeout=5);assert p.returncode==2
  check=subprocess.run([sys.executable,__file__,'--suite',args.suite,'--install-prefix',args.install_prefix,'--output-dir',str(c.root),'--check'],capture_output=True,text=True)
  assert check.returncode!=0 and 'output exists' in check.stderr
 def A20(c):
  p=c.start('mission_node',{'test_mode':False,'action_name':c.action,'database':str(c.root/'formal.db'),'process_epoch':'formal'},'formal-refused');p.expected_exit=2;p.wait(timeout=5);assert p.returncode==2
  assert 'PRODUCTION_REFUSED' in (c.root/'formal-refused.log').read_text() and not (c.root/'formal.db').exists()
 functions={k:v for k,v in locals().items() if k.startswith('A') and callable(v)}
 configs={'A03':{'readiness_ms':300},'A10':{'accept_ms':200,'execute_ms':500},'A12':{'execute_ms':800},'A14':{'execute_ms':700,'use_sim_time':True},'A16':{'mailbox_capacity':1},'A17':{'execute_ms':120000}}
 for id in selected:
  if id in functions:execute(id,functions[id],configs.get(id),id!='A03')
  else:results.append({'id':id,'status':'NOT_EXECUTED','layer':'pending','error':'test implementation pending'});(root/'results.json').write_text(json.dumps(results,indent=2))
 return all(x['status']=='PASS' for x in results)

def main():
 p=argparse.ArgumentParser();p.add_argument('--suite',required=True);p.add_argument('--install-prefix',required=True);p.add_argument('--output-dir',required=True);p.add_argument('--cases');p.add_argument('--replay-input');p.add_argument('--replay-limit',type=int);p.add_argument('--check',action='store_true');args=p.parse_args()
 suite,prefix,bin_dir=check_args(args)
 if args.check:print('CHECK_OK no ROS/node/DB/output side effects');return 0
 if os.environ.get('ROS_DOMAIN_ID')!='72' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('explicit domain72 and loopback required')
 global rclpy,MissionCommand,SyntheticEvent,FakeControl,message_to_ordereddict
 import rclpy
 from cmr_interfaces.srv import MissionCommand,SyntheticEvent,FakeControl
 from rosidl_runtime_py.convert import message_to_ordereddict
 rclpy.init()
 try:return 0 if run_matrix(args,suite,bin_dir) else 1
 finally:rclpy.shutdown()
if __name__=='__main__':
 try:sys.exit(main())
 except Exception as e:print('SUPERVISOR_ERROR '+repr(e),file=sys.stderr);sys.exit(1)
