#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Isolated ROS Action protocol supervisor; --check has no ROS/filesystem writes."""
import argparse,json,os,signal,sqlite3,subprocess,sys,time,hashlib,traceback
from contextlib import closing
from pathlib import Path
if __package__ in (None,''):
 from runpy import run_path
 run_path(str(Path(__file__).resolve().parents[1]/'__init__.py'))['bootstrap']()

def check_args(args):
 suite=json.loads(Path(args.suite).read_text())
 if suite.get('domain')!=72 or not suite.get('synthetic') or suite.get('version')!=1:raise ValueError('invalid suite contract')
 expected={f'A{i:02}' for i in range(1,21)}
 if {c['id'] for c in suite['cases']}!=expected:raise ValueError('incomplete matrix')
 prefix=Path(args.install_prefix).resolve();bin_dir=prefix/'cmr_mission/lib/cmr_mission'
 for package,name in [('cmr_mission','mission_node'),('cmr_mission','fake_nav2_server'),
                      ('cmr_mission','action_correlation_test'),('cmr_execution','execution_gate'),
                      ('cmr_execution','execution_observer')]:
  if not (prefix/package/'lib'/package/name).is_file():raise ValueError('missing executable '+name)
 if Path(args.output_dir).exists():raise ValueError('output exists; refuse overwrite')
 return suite,prefix,bin_dir

def run_matrix(args,suite,prefix,bin_dir):
 from cmr_interfaces.srv import DeliveryCommand,MissionCommand
 from rosidl_runtime_py.convert import message_to_ordereddict as dic
 from tools.mission import mission_harness
 DeliveryCase=mission_harness.DeliveryCase
 root=Path(args.output_dir);root.mkdir(parents=True,exist_ok=False);results=[]
 (root/'manifest.json').write_text(json.dumps({'suite_sha256':hashlib.sha256(Path(args.suite).read_bytes()).hexdigest(),'binaries':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in bin_dir.iterdir() if p.is_file()},'domain':72,'loopback':True,'synthetic':True,'install_prefix':str(bin_dir.parent.parent.parent)},indent=2))
 (root/'mission-harness-source.py').write_bytes(Path(mission_harness.__file__).read_bytes())
 selected=args.cases.split(',') if args.cases else [c['id'] for c in suite['cases']]
 def execute(id,fn,config=None,start_server=True):
  case=None;error=None
  try:
   params={'queue_capacity':0,'accept_ms':1000,'execute_ms':15000,**(config or {})}
   case=DeliveryCase.__new__(DeliveryCase);case.__init__(root/id,prefix,params,start_server=start_server);fn(case)
  except Exception as e:error=repr(e);(root/(id+"-failure.txt")).write_text(traceback.format_exc())
  finally:
   if case:
    try:case.close()
    except Exception as e:error=(error+'; ' if error else '')+repr(e)
  row={'id':id,'status':'FAIL' if error else 'PASS','layer':'adapter unit + Delivery/Gate ROS' if id=='A11' else 'ROS+owner+SQLite','error':error,'startup':'PASS' if case and hasattr(case,'mission') and case.mission else 'FAIL','protocol':'ADAPTER_UNIT_AND_DELIVERY_GATE' if id=='A11' else ('PASS' if not error else 'FAIL'),'mission':'PASS' if not error else 'UNCONFIRMED','fault_activation':('CONFIRMED' if case and any('fault' in entry for entry in case.records) else 'NOT_EXERCISED') if error else ('NOT_APPLICABLE' if id in ['A01','A02','A17'] else 'CONFIRMED'),'harness':'PASS' if not error else 'FAIL','cleanup':'PASS' if case and all(p.poll() is not None for p,log,tag,identity in case.procs) else 'FAIL','evidence':str(root/id)};results.append(row);(root/'results.json').write_text(json.dumps(results,indent=2));print(json.dumps(row),flush=True)

 def A01(c):
  c.submit()
  outbound=c.leg()
  assert c.handover(outbound,command='handover1').code=='CONFIRMED'
  assert c.handover(outbound,command='handover1').code=='CONFIRMED'
  conflict=c.cmd('handover',command_id='handover1',handover_id=outbound.handover_id,
                 handover_entry_version=outbound.handover_entry_version+1,source='synthetic')
  assert conflict.code=='COMMAND_CONFLICT'
  returned=c.leg(True)
  assert returned.goal_uuid!=outbound.goal_uuid and c.send_count()==2
  assert c.cmd('cancel',command_id='after_complete').code=='ALREADY_TERMINAL'
 def A02(c):
  assert c.cmd('submit',id='bad_schema',schema_version=99).code=='INVALID_STATION_VERSION'
  assert c.cmd('submit',id='bad_station',delivery_station='missing').code=='INVALID_STATION_VERSION'
  assert c.cmd('submit',id='bad id').code=='INVALID_STATION_VERSION'
  assert c.send_count()==0
  c.wait(c.healthy)
  # Withhold client dispatch until the application deadline, then check the commit in SQLite.
  request=c.request('submit',command_id='reply_timeout')
  future=c.delivery.call_async(request)
  time.sleep(.2)
  assert not future.done()
  with closing(sqlite3.connect(c.db)) as db:
   assert db.execute("SELECT count(*) FROM registrations WHERE mission='m'").fetchone()[0]==1
  c.delivery.remove_pending_request(future)
  c.record_fault('client_reply_deadline_after_commit')
  assert c.cmd('submit',command_id='reply_timeout').code=='EXISTING'
  c.accepted()
  assert c.cmd('cancel',command_id='bad id').code=='INVALID_COMMAND_ID'
  assert c.cmd('submit',created_at_unix_ms=99).code=='EXISTING'
  assert c.cmd('submit',return_station='a').code=='CONFLICT'
  assert c.cmd('submit',id='other').code=='BUSY' and c.send_count()==1
 def A03(c):
  c.record_fault('missing_Action_server_before_Delivery_admission')
  assert c.validate_action_graph()==[] and c.cmd('submit').code=='INPUT_UNQUALIFIED'
  assert c.query().code=='NOT_FOUND' and c.send_count()==0 and not c.state.allowed
  c.start_server();c.submit();c.accepted()
  assert c.send_count()==1
 def A04(c):
  assert c.ctrl('reject',1).success
  c.record_fault('server_rejects_Goal')
  c.submit();c.wait(lambda:c.event_count(contains='ACTION_REJECTED')>0)
  c.expect_state('FAILED')
  assert not c.query().goal_uuid and c.send_count()==1
 def A05(c):
  c.submit();c.accepted();c.wait(lambda:c.event_count(contains='FEEDBACK')>0)
  assert c.query().state=='NAVIGATING'
  c.record_fault('server_aborts_active_Goal')
  assert c.ctrl('abort').success
  c.wait(lambda:c.event_count(3)>0)
  assert c.read_record()['pending']=='FAILED'
  c.assert_slot_occupied()
 def A06(c):
  assert c.ctrl('cancel_delay',800).success
  c.submit();c.accepted();c.record_fault('delayed_cancel_terminal')
  assert c.cmd('cancel',command_id='cancel').code=='CANCEL_PENDING'
  c.wait(lambda:c.event_count(contains='CANCEL_RESPONSE')>0)
  c.assert_slot_occupied()
  c.wait(lambda:c.event_count(18)>0)
  c.expect_state('CANCELED')
  assert c.send_count()==1 and not c.state.allowed
 def A07(c):
  assert c.ctrl('reject_cancel',1).success
  c.submit();c.accepted();c.record_fault('server_rejects_exact_cancel')
  assert c.cmd('cancel').code=='CANCEL_PENDING'
  c.wait(lambda:c.event_count(contains='CANCEL_RESPONSE')>0)
  assert c.query().state=='PAUSED' and c.event_count(18)==0
  c.assert_slot_occupied()
  assert c.send_count()==1
 def A08(c):
  assert c.ctrl('delay_accept',5000).success
  c.submit();c.wait(lambda:'ACCEPT_BARRIER' in (c.root/'server.log').read_text())
  c.record_fault('cancel_before_Action_acceptance')
  assert c.cmd('cancel').code=='CANCEL_PENDING' and not c.query().goal_uuid
  assert c.ctrl('release_accept').success
  c.wait(lambda:'CANCEL_REQUEST uuid=' in (c.root/'server.log').read_text())
  c.wait(lambda:c.event_count(18)>0)
  c.expect_state('CANCELED')
  assert c.send_count()==1
 def A09(c):
  assert c.ctrl('reject_cancel',1).success
  c.submit();c.accepted();c.record_fault('success_then_cancel')
  assert c.ctrl('success').success;c.wait(lambda:c.event_count(2)==1)
  assert c.cmd('cancel',command_id='cancel_after_result').code=='CANCEL_PENDING'
  assert c.read_record()['pending']=='CANCELED'
  c.expect_state('CANCELED')
  assert c.cmd('resume_schedule').code=='RESUMED'
  c.submit('m2');c.wait(lambda:bool(c.cmd(id='m2').goal_uuid),10)
  c.record_fault('cancel_then_success')
  assert c.cmd('cancel',id='m2',command_id='before_result').code=='CANCEL_PENDING'
  assert c.ctrl('success').success
  c.wait(lambda:c.event_count(2)==2)
  c.wait(lambda:c.cmd(id='m2').state=='CANCELED',7)
  assert c.send_count()==2
 def A10(c):
  assert c.ctrl('delay_accept',3000).success
  c.submit();c.wait(lambda:c.event_count(contains='ACCEPT_TIMEOUT')>0)
  c.record_fault('acceptance_deadline_elapsed_before_release')
  assert c.query().state=='PAUSED' and c.ctrl('release_accept').success
  c.wait(lambda:'CANCEL_REQUEST uuid=' in (c.root/'server.log').read_text())
  c.wait(lambda:c.event_count(18)>0);c.expect_state('FAILED')
  assert c.send_count()==1
  assert c.cmd('resume_schedule').code=='RESUMED'
  assert c.ctrl('delay_accept',0).success and c.ctrl('reject_cancel',1).success
  c.submit('m2');c.wait(lambda:c.cmd(id='m2').state=='NAVIGATING',10)
  c.wait(lambda:c.event_count(contains='EXECUTION_TIMEOUT')>0)
  c.record_fault('execution_deadline_elapsed_before_late_success')
  assert c.ctrl('success').success;c.wait(lambda:c.event_count(2)>0)
  q=c.cmd(id='m2')
  assert q.state=='PAUSED' and c.read_record('m2')['pending']=='FAILED'
  assert c.send_count()==2
 def A11(c):
  p=subprocess.run([str(c.bin/'action_correlation_test')],capture_output=True,text=True,timeout=10)
  (c.root/'adapter-unit.log').write_text(p.stdout+p.stderr);assert p.returncode==0
  c.submit();c.accepted()
  api=c.node.create_client(MissionCommand,c.ns+'/command');c.wait_service(api)
  assert c.call(api,MissionCommand.Request(operation='submit',mission_id='m')).code=='DELIVERY_API_REQUIRED'
  q=c.call(api,MissionCommand.Request(operation='query',mission_id='m'))
  assert q.state=='NAVIGATING' and q.goal_uuid==c.query().goal_uuid and c.send_count()==1
  assert c.ns+'/synthetic_event' not in dict(c.node.get_service_names_and_types())
 def A12(c):
  c.submit();c.accepted();c.injected_exit(c.server);c.expect_state('PAUSED')
  c.start_server('server-restarted');c.spin(.3)
  assert c.send_count()==1 and c.query().state=='PAUSED'
 def A13(c):
  assert c.ctrl('delay_accept',3000).success
  c.submit();c.wait(lambda:'ACCEPT_BARRIER' in (c.root/'server.log').read_text())
  c.injected_exit(c.mission);assert c.ctrl('release_accept').success
  c.start_delivery_node(c.name+'_new','mission-restarted');c.expect_state('PAUSED');c.spin(.3)
  assert c.send_count()==1 and not c.query().goal_uuid and c.query().schedule_paused
 def A14(c):
  c.submit();c.accepted()
  old=c.state.clock_epoch
  c.clock_paused=True;c.record_fault('private_source_clock_paused')
  c.wait(lambda:c.state.state=='STOPPING' and not c.state.allowed,2)
  c.clock_offset_ns=-5000000000;c.clock_paused=False
  c.record_fault('private_source_clock_rolled_back')
  c.wait(lambda:c.state.clock_epoch!=old,3)
  c.expect_state('PAUSED')
  assert not c.state.allowed and c.send_count()==1
 def A15(c):
  c.submit();c.accepted();c.odom_on=False
  c.record_fault('Goal_success_without_observer_stop_window')
  assert c.ctrl('success').success;c.wait_result()
  c.spin(3.3)
  assert c.query().state=='PAUSED' and c.report.verdict!='STOPPED'
  assert c.handover(c.query()).code=='INVALID_HANDOVER'
  assert c.ns+'/synthetic_event' not in dict(c.node.get_service_names_and_types())
  c.assert_slot_occupied('INPUT_UNQUALIFIED')
 def A16(c):
  c.wait(c.healthy)
  with closing(sqlite3.connect(c.db)) as db:
   db.executescript("CREATE TRIGGER fail BEFORE INSERT ON registrations BEGIN SELECT RAISE(ABORT,'test storage failure'); END;")
  c.record_fault('SQLite_trigger_aborts_registration')
  assert c.cmd('submit').code=='STORAGE_ERROR' and c.send_count()==0
  with closing(sqlite3.connect(c.db)) as db:
   assert db.execute('SELECT count(*) FROM registrations').fetchone()[0]==0
   db.execute('DROP TRIGGER fail')
  c.injected_exit(c.mission)
  epoch=c.name+'_new'
  c.restart_idle_gate(epoch)
  c.start_delivery_node(epoch,'mission-after-db-fault')
  assert c.cmd('queue').schedule_paused and c.cmd('resume_schedule').code=='RESUMED'
  c.submit();c.accepted()
  with closing(sqlite3.connect(c.db,timeout=1)) as writer:
   writer.execute('BEGIN IMMEDIATE')
   c.record_fault('external_SQLite_writer_and_concurrent_ROS_requests')
   blocker=c.delivery.call_async(c.request('pause_schedule',command_id='blocked'))
   futures=[c.delivery.call_async(c.request('query')) for _ in range(16)]
   # The native Result must arrive while the writer holds the owner transaction.
   assert c.ctrl('success').success
   log=c.root/'mission-after-db-fault.log'
   c.wait(lambda:any(line.startswith('CALLBACK ') and ' kind=2 ' in line
                     for line in log.read_text().splitlines()),.07)
   assert not blocker.done()
   writer.rollback()
  c.wait(lambda:all(f.done() for f in [blocker,*futures]))
  assert blocker.result().code=='PAUSED'
  codes=[f.result().code for f in futures]
  assert 'OVERLOADED' in codes and c.send_count()==1
  (c.root/'overload-codes.json').write_text(json.dumps(codes))
  c.wait(lambda:c.event_count(2)==1 and not c.state.allowed)
 def A17(c):
  import random
  c.submit();c.accepted()
  clients=[c.node.create_client(DeliveryCommand,c.ns+'/delivery',qos_profile=c.delivery_qos) for _ in range(3)]
  for client in clients:c.wait_service(client)
  if args.replay_input:
   inputs=[json.loads(line) for line in Path(args.replay_input).read_text().splitlines()]
   if args.replay_limit:inputs=inputs[:args.replay_limit]
  else:
   inputs=[]
   for seed in range(20):
    rng=random.Random(seed)
    for index in range(200):
     op=rng.choice(['query','submit','cancel','handover','conflict','busy']);fields={};mission='m'
     if op=='conflict':op='submit';fields['return_station']='a'
     if op=='busy':op='submit';mission='other'
     request=c.request(op,mission,command_id='s'+str(seed)+'e'+str(index),**fields)
     inputs.append({'seed':seed,'index':index,'client':index%3,'request':dic(request)})
  assert inputs
  # Hold the first Goal unresolved throughout the request interleavings.
  assert c.ctrl('reject_cancel',1).success
  with (c.root/'replay-input.jsonl').open('x') as eventfile:
   for begin in range(0,len(inputs),10):
    pending=[]
    for entry in inputs[begin:begin+10]:
     fields=dict(entry['request'])
     # Historical replay generated a version-zero handover with no token.
     if 'expected_state_version' in fields:
      if fields.pop('expected_state_version')!=0:raise ValueError('legacy replay contains a versioned command')
     request=DeliveryCommand.Request(**fields)
     eventfile.write(json.dumps({**entry,'request':dic(request)})+'\n')
     pending.append((entry,request,clients[entry['client']].call_async(request)))
    for entry,request,future in pending:
     c.wait(future.done);response=future.result()
     assert response is not None and response.code not in ['ACCEPTED','QUEUED','STORAGE_ERROR']
     c.records.append({'request':dic(request),'response':dic(response),'service':c.ns+'/delivery',
                       'seed':entry['seed'],'index':entry['index'],'time_monotonic':time.monotonic()})
    with closing(sqlite3.connect(c.db)) as db:
     assert db.execute('SELECT count(*) FROM missions WHERE terminal=0').fetchone()[0]==1
  assert c.send_count()==1
  (c.root/'interleaving-summary.json').write_text(json.dumps({'events':len(inputs),'seeds':sorted({x['seed'] for x in inputs}),'clients':3,'replayed':bool(args.replay_input)},indent=2))
 def A18(c):
  c.wait(c.healthy)
  with closing(sqlite3.connect(c.db,timeout=1)) as writer:
   writer.execute('BEGIN IMMEDIATE')
   blocker=c.delivery.call_async(c.request('pause_schedule',command_id='shutdown_blocker'))
   request=c.request('submit',command_id='pending_shutdown')
   queued=c.delivery.call_async(request)
   c.wait(lambda:'"command_id":"pending_shutdown"' in (c.root/'mission.log').read_text(),.08)
   os.killpg(c.mission.pid,signal.SIGINT)
   c.record_fault('shutdown_with_pending_submit_behind_SQLite_writer')
   c.wait(queued.done,5);assert queued.result().code=='CLOSED'
   c.wait(blocker.done,5)
  c.wait(lambda:c.mission.poll() is not None,5)
  assert c.mission.returncode==0 and c.send_count()==0
  with closing(sqlite3.connect(c.db)) as db:
   assert db.execute('SELECT count(*) FROM registrations').fetchone()[0]==0
  epoch=c.name+'_pending'
  c.restart_idle_gate(epoch)
  c.start_delivery_node(epoch,'mission-after-pending')
  assert c.cmd('queue').schedule_paused and c.cmd('resume_schedule').code=='RESUMED'
  assert c.ctrl('delay_accept',700).success
  c.submit();c.wait(lambda:'ACCEPT_BARRIER' in (c.root/'server.log').read_text())
  os.killpg(c.mission.pid,signal.SIGINT);c.wait(lambda:c.mission.poll() is not None,5)
  assert c.mission.returncode==0 and c.send_count()==1
  for cycle in range(2):
   c.start_delivery_node(c.name+'_cycle'+str(cycle),'mission-cycle-'+str(cycle))
   assert c.query().state=='PAUSED'
   os.killpg(c.mission.pid,signal.SIGINT);c.wait(lambda:c.mission.poll() is not None,5)
   assert c.mission.returncode==0
  assert c.send_count()==1
 def A19(c):
  c.start_server('server-extra')
  c.wait(lambda:sum(n=='fake_nav2_server' and ns==c.ns for n,ns in c.node.get_node_names_and_namespaces())>=2)
  try:c.validate_action_graph();raise AssertionError('duplicate server was accepted')
  except RuntimeError as error:assert 'extra Action Server' in str(error)
  c.wait_idle_stop()
  c.record_fault('duplicate_Action_server')
  c.submit()
  mission_log=c.root/'mission.log'
  c.wait(lambda:'ISOLATION_EXTRA_SERVER count=2' in mission_log.read_text())
  c.mission.expected_exit=2
  c.wait(lambda:c.mission.poll() is not None and not c.state.allowed,3)
  assert c.mission.returncode==2 and 'MISSION_FATAL adapter disarmed' in mission_log.read_text()
  assert c.send_count()==0 and 'SEND_CALL ' not in mission_log.read_text()
  # Store may commit the adapter rejection or owner health fault first. Both must revoke.
  with closing(sqlite3.connect(c.db)) as db:
   assert db.execute('SELECT count(*) FROM operations WHERE kind=2').fetchone()[0]>0
   assert db.execute('SELECT active FROM manager').fetchone()[0]=='m'
  assert c.event_count(contains='GATE_GRANTED')>0
  from cmr_interfaces.srv import FakeControl
  wrong=c.node.create_service(FakeControl,c.ns+'/wrong/_action/send_goal',lambda q,r:r)
  try:c.validate_action_graph(c.ns+'/wrong');raise AssertionError('wrong type accepted')
  except RuntimeError as error:assert 'type mismatch' in str(error)
  finally:c.node.destroy_service(wrong)
  check=subprocess.run([sys.executable,__file__,'--suite',args.suite,'--install-prefix',args.install_prefix,'--output-dir',str(c.root),'--check'],capture_output=True,text=True)
  assert check.returncode!=0 and 'output exists' in check.stderr
 def A20(c):
  base={'test_mode':True,'action_name':c.action,'process_epoch':'startup_check',
        'execution_run_id':c.name,'station_catalog':str(c.catalog)}
  for tag,overrides,reason in [
   ('invalid-action',{'action_name':'/outside/navigate_to_pose'},'invalid explicit delivery'),
   ('wrong-domain',{'test_mode':False,'action_name':'/navigate_to_pose'},'invalid explicit delivery'),
   ('namespace-mismatch',{'execution_namespace':'/wrong'},'execution run/namespace mismatch'),
   ('clock-policy',{'use_sim_time':True},'invalid explicit delivery')]:
   db=c.root/(tag+'.db')
   p=c.start('mission_node',{**base,**overrides,'database':str(db)},tag)
   p.expected_exit=2;p.wait(timeout=5)
   assert p.returncode==2 and reason in (c.root/(tag+'.log')).read_text() and not db.exists()
  c.record_fault('native_parameter_override_without_local_allowlist')
  c.injected_exit(c.mission)
  c.config['unexpected_option']=1
  c.start_delivery_node(c.name+'_override','mission-with-unused-override')
  assert c.query().code=='NOT_FOUND'
  from rcl_interfaces.srv import ListParameters
  params=c.node.create_client(ListParameters,c.ns+'/mission_node/list_parameters')
  c.wait_service(params)
  names=c.call(params,ListParameters.Request()).result.names
  assert 'unexpected_option' not in names and 'station_catalog' in names
 functions={k:v for k,v in locals().items() if k.startswith('A') and callable(v)}
 configs={'A08':{'accept_ms':3000},'A10':{'accept_ms':200,'execute_ms':1500},
          'A12':{'execute_ms':800},'A16':{'mailbox_capacity':1},
          'A17':{'execute_ms':220000,'task_ms':240000}}
 for id in selected:
  if id in functions:execute(id,functions[id],configs.get(id),id!='A03')
  else:results.append({'id':id,'status':'NOT_EXECUTED','layer':'pending','error':'test implementation pending'});(root/'results.json').write_text(json.dumps(results,indent=2))
 return all(x['status']=='PASS' for x in results)

def main():
 p=argparse.ArgumentParser();p.add_argument('--suite',required=True);p.add_argument('--install-prefix',required=True);p.add_argument('--output-dir',required=True);p.add_argument('--cases');p.add_argument('--replay-input');p.add_argument('--replay-limit',type=int);p.add_argument('--check',action='store_true');args=p.parse_args()
 suite,prefix,bin_dir=check_args(args)
 if args.check:print('CHECK_OK no ROS/node/DB/output side effects');return 0
 if os.environ.get('ROS_DOMAIN_ID')!='72' or os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp':raise ValueError('domain72 + loopback + FastDDS required')
 import rclpy
 rclpy.init()
 try:return 0 if run_matrix(args,suite,prefix,bin_dir) else 1
 finally:rclpy.shutdown()
if __name__=='__main__':
 try:sys.exit(main())
 except Exception as e:print('SUPERVISOR_ERROR '+repr(e),file=sys.stderr);sys.exit(1)
