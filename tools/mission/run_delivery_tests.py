#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""P1-04 Fake Action + real Mission/Gate + explicitly synthetic localization/motion.
--check only validates inputs/executables; never imports ROS or creates outputs.
"""
import argparse,json,os,sys,time,signal,hashlib,traceback
from pathlib import Path
if __package__ in (None,''):
 from runpy import run_path
 run_path(str(Path(__file__).resolve().parents[1]/'__init__.py'))['bootstrap']()

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--install-prefix',required=True);ap.add_argument('--output-dir',required=True);ap.add_argument('--cases',default='D01');ap.add_argument('--check',action='store_true');a=ap.parse_args();prefix=Path(a.install_prefix).resolve();out=Path(a.output_dir)
 if out.exists():raise ValueError('refuse overwrite')
 for pkg,exe in [('cmr_mission','mission_node'),('cmr_mission','fake_nav2_server'),('cmr_execution','execution_gate'),('cmr_execution','execution_observer')]:
  if not (prefix/pkg/'lib'/pkg/exe).is_file():raise ValueError('missing executable '+exe)
 selected=a.cases.split(',')
 if not selected or len(set(selected))!=len(selected) or any(x not in ['D%02d'%n for n in range(1,19)] for x in selected):raise ValueError('case selection')
 if a.check:print('CHECK_OK no ROS/DB/process/output');return 0
 if os.environ.get('ROS_DOMAIN_ID')!='72' or os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp':raise ValueError('isolated domain72 loopback FastDDS required')
 import rclpy
 from cmr_interfaces.srv import ExecutionLease
 from tools.mission import mission_harness
 rclpy.init();out.mkdir(parents=True,exist_ok=False);(out/'manifest.json').write_text(json.dumps({'domain':72,'synthetic':True,'layer':'real MissionRuntime/Store/NavigationAdapter/ExecutionPort/Gate; Fake Action and synthetic localization/motion','prefix':str(prefix),'supervisor_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'cases':selected},indent=2))
 (out/'supervisor-source.py').write_bytes(Path(__file__).read_bytes());(out/'mission-harness-source.py').write_bytes(Path(mission_harness.__file__).read_bytes())
 class Case(mission_harness.DeliveryCase):
  def __init__(self,root,config=None):
   super().__init__(root,prefix,config)
 def D01(c):
  c.submit();outbound=c.leg();assert not c.state.allowed and not outbound.handed_over;assert c.handover(outbound).code=='CONFIRMED';assert c.handover(outbound).code=='CONFIRMED';returned=c.leg(True);assert returned.handed_over and returned.handover_source=="synthetic" and returned.goal_uuid!=outbound.goal_uuid and c.send_count()==2
  assert c.handover(outbound).code=='CONFIRMED'
  states=[x['data'] for x in c.records if x.get('kind')=='gate_state' and x['data']['allowed']]
  assert len({x['lease_id'] for x in states})==2
  assert all('ACTIVE peak=1' in (c.root/'server.log').read_text() for _ in [0]) and 'ACTIVE peak=2' not in (c.root/'server.log').read_text()
 def D02(c):
  c.submit();c.wait(lambda:bool(c.query().goal_uuid),10);c.tf_on=False;c.odom_on=False;assert c.ctrl('success',goal=c.query().goal_uuid).success;c.spin(4);assert c.query().state=='PAUSED' and not c.query().handed_over
 def D03(c):
  c.submit();c.wait(lambda:c.x>.05 and abs(c.held.linear.x)>.01,10);assert c.cmd('cancel',command_id='out_cancel').code=='CANCEL_PENDING';c.wait(lambda:c.query().state=='CANCELED',8);assert c.cmd('queue').schedule_paused and c.send_count()==1
  root=c.root;c.close();c.__init__(root/'return_cancel');c.submit();q=c.leg();assert c.handover(q).code=='CONFIRMED';c.wait(lambda:c.query().state=='RETURNING' and c.held.linear.x<-.01,10);assert c.cmd('cancel',command_id='return_cancel').code=='CANCEL_PENDING';c.wait(lambda:c.query().state=='CANCELED',8);assert c.query().handed_over and c.send_count()==2
 def D04(c):
  c.submit();q=c.leg();assert c.cmd('handover',id='wrong',command_id='wrong_m',handover_id=q.handover_id,handover_entry_version=q.handover_entry_version,source='synthetic').code=='NOT_ACTIVE';assert c.cmd('handover',command_id='wrong_t',handover_id='wrong',handover_entry_version=q.handover_entry_version,source='synthetic').code=='HANDOVER_IDENTITY';assert c.cmd('handover',command_id='wrong_v',handover_id=q.handover_id,handover_entry_version=q.handover_entry_version+1,source='synthetic').code=='HANDOVER_IDENTITY';c.wait(lambda:c.query().state=='FAILED',5);assert not c.query().handed_over and c.send_count()==1 and not c.state.allowed
 def D05(c):
  c.submit();q=c.leg();assert c.handover(q).code=='CONFIRMED';c.wait(lambda:c.query().state=='RETURNING',10);uid=c.query().goal_uuid;assert c.ctrl('abort',goal=uid).success;c.wait_new_goal(uid);assert c.ctrl('abort',goal=c.query().goal_uuid).success;c.wait(lambda:c.query().state=='FAILED',8);assert c.query().handed_over and c.send_count()==3
 def D06(c):
  c.submit();c.wait(lambda:bool(c.query().goal_uuid),10)
  for i in range(4):assert c.cmd('submit',id='q'+str(i)).code=='QUEUED'
  assert c.cmd('submit',id='fifth').code=='QUEUE_FULL';assert c.cmd('submit',id='q0').code=='EXISTING';assert c.cmd('submit',id='q0',return_station='a').code=='CONFLICT';assert not c.cmd(id='q0').state and c.send_count()==1
  root=c.root;c.close();c.__init__(root/'concurrent_requests');c.submit();c.wait(lambda:bool(c.query().goal_uuid),10);assert c.cmd('submit',id='anchor').code=='QUEUED'
  pending=[]
  try:
   for i in range(7):pending.append(('unique',c.delivery.call_async(c.request('submit',id='u'+str(i)))))
   for _ in range(3):pending.append(('duplicate',c.delivery.call_async(c.request('submit',id='anchor'))))
   pending.append(('conflict',c.delivery.call_async(c.request('submit',id='anchor',return_station='a'))))
   in_flight=sum(not f.done() for _,f in pending);assert in_flight==11
   c.wait(lambda:all(f.done() for _,f in pending),8)
   codes=[(kind,f.result().code) for kind,f in pending]
   assert sum(kind=='unique' and code=='QUEUED' for kind,code in codes)==3 and sum(kind=='unique' and code=='QUEUE_FULL' for kind,code in codes)==4
   assert all(code=='EXISTING' for kind,code in codes if kind=='duplicate') and all(code=='CONFLICT' for kind,code in codes if kind=='conflict')
   orders=json.loads(c.cmd('queue').detail)['queue']['orders'];assert sum(o['queue_state']=='WAITING' for o in orders)==4 and c.send_count()==1
   c.records.append({'qualification':'11 real ROS service requests concurrently in flight before any response spin','in_flight':in_flight,'results':codes,'independent_evidence':'native DELIVERY_RX order, SQLite registrations and one native Goal'})
  finally:
   for _,future in pending:
    if not future.done():c.delivery.remove_pending_request(future)

 def D07(c):
  c.submit();c.wait(lambda:bool(c.query().goal_uuid),10);assert c.cmd('submit',id='waiting').code=='QUEUED';uid=c.query().goal_uuid;assert c.cmd('cancel',id='waiting',command_id='queue_cancel').code=='QUEUE_CANCELED';c.spin(.2);assert c.query().goal_uuid==uid and c.query().state=='NAVIGATING' and 'CANCEL_REQUEST' not in (c.root/'server.log').read_text();assert c.cmd('submit',id='waiting').code=='EXISTING'
 def D08(c):
  c.submit();assert c.cmd('submit',id='q').code=='QUEUED';out=c.leg();assert c.handover(out).code=='CONFIRMED';c.leg(True);c.wait(lambda:c.cmd(id='q').state=='NAVIGATING',12);out=c.leg(id='q');assert c.handover(out,id='q').code=='CONFIRMED';c.leg(True,id='q');assert c.send_count()==4 and 'ACTIVE peak=2' not in (c.root/'server.log').read_text()
 def D09(c):
  c.submit();c.wait(lambda:bool(c.query().goal_uuid),10);assert c.cmd('submit',id='q').code=='QUEUED';order=c.cmd(id='q').queue_seq;c.injected_exit(c.mission);c.records.append({'fault':'actual_Mission_exit_then_supervised_restart','synthetic_motion':True});c.wait(lambda:not c.state.allowed and c.held.linear.x==0,1);c.start_delivery_node(c.name+'_restart','mission-restarted');c.wait(lambda:c.query().state=='PAUSED',6);c.spin(.3);assert c.cmd(id='q').queue_seq==order and c.cmd(id='q').queue_state=='WAITING' and c.cmd('queue').schedule_paused and c.send_count()==1
 def D10(c):
  c.submit();c.wait(lambda:c.x>.05 and abs(c.held.linear.x)>.01,10);birth=next(x[3] for x in c.procs if x[0] is c.mission);assert Path('/proc/'+str(c.mission.pid)+'/stat').read_text().split(') ')[1].split()[19]==birth;os.killpg(c.mission.pid,signal.SIGSTOP);before=time.monotonic();c.records.append({'fault':'actual_owner_SIGSTOP','receive_monotonic':before});c.wait(lambda:c.state.reason=='LEASE_EXPIRED' and c.held.linear.x==0,1);assert time.monotonic()-before<.6;os.killpg(c.mission.pid,signal.SIGCONT);c.wait(lambda:c.query().state=='FAILED',10);assert c.send_count()==1
 def D11(c):
  c.ctrl('reject',1);c.submit();c.wait(lambda:c.query().state=='FAILED',10);assert c.send_count()==1 and not c.query().goal_uuid
  root=c.root;c.close();c.__init__(root/'accept_unknown',{'accept_ms':300});c.ctrl('delay_accept',3000);c.submit();c.wait(lambda:'ACCEPT_BARRIER' in (c.root/'server.log').read_text());c.wait(lambda:c.event_count(contains='ACCEPT_TIMEOUT')>0,3);c.ctrl('release_accept');c.wait(lambda:c.query().state=='FAILED',10);assert c.send_count()==1
  c.close();c.__init__(root/'server_loss',{'execute_ms':1200});c.submit();c.wait(lambda:bool(c.query().goal_uuid),10);c.injected_exit(c.server);c.wait(lambda:c.query().state=='PAUSED',5);c.start_server('server-restarted');c.spin(.3);assert c.query().state=='PAUSED' and c.send_count()==1
 def D12(c):
  c.submit();c.wait(lambda:c.x>.05 and c.state.allowed,10);q=ExecutionLease.Request();q.operation='revoke';q.request_id='external_test_revoke';q.request_seq=100000;q.identity=c.state.identity;q.lease_id=c.state.lease_id;q.old_goal_resolved=False;assert c.call(c.lease,q).code=='REVOKED';c.records.append({'fault':'controlled_external_Gate_revoke_no_Mission_bypass','synthetic':True});c.wait(lambda:c.query().state=='FAILED',8);assert c.send_count()==1
  root=c.root;c.close();c.__init__(root/'stop_stream_missing');c.submit();c.wait(lambda:bool(c.query().goal_uuid),10);c.odom_on=False;c.ctrl('success',goal=c.query().goal_uuid);c.spin(4);assert c.query().state=='PAUSED' and c.report.verdict!='STOPPED'
 def D13(c):
  for key,value in [('map_version','wrong'),('station_version','wrong'),('delivery_station','unknown')]:assert c.cmd('submit',id='bad_'+key,**{key:value}).code=='INVALID_STATION_VERSION'
  assert c.send_count()==0
  c.wait(lambda:c.state is not None);c.spin(.5);assert c.cmd('pause_schedule').code=='PAUSED';assert c.cmd('submit',id='q').code=='QUEUED';(c.root/'map.yaml').write_text('test resource hash changed');assert c.cmd('resume_schedule').code=='RESUMED';c.wait(lambda:c.cmd('queue').schedule_paused,3);assert c.cmd(id='q').queue_state=='WAITING' and c.send_count()==0
 def D14(c):
  c.submit();c.wait(lambda:bool(c.query().goal_uuid),10);old=c.query().goal_uuid;assert c.ctrl('abort',goal=old).success;c.wait_new_goal(old);q=c.leg();assert c.handover(q).code=='CONFIRMED';c.wait(lambda:c.query().state=='RETURNING',10);assert c.ctrl('abort',goal=c.query().goal_uuid).success;c.wait(lambda:c.query().state=='FAILED',8);assert c.query().handed_over and c.send_count()==3
  c.records.append({'recovery_budget':'one additional attempt shared across outbound/return','persistent_obstacle':'Fake ABORTED only; physical obstruction NOT_EXERCISED'})
 def D15(c):
  c.wait(c.healthy,7);c.records.append({'qualification':'healthy_before_second_Goal_owner_injection'})
  from rclpy.action import ActionClient
  from nav2_msgs.action import NavigateToPose
  extra=ActionClient(c.node,NavigateToPose,c.action)
  try:c.spin(.5);assert c.cmd('submit').code=='INPUT_UNQUALIFIED' and c.send_count()==0;c.records.append({'fault':'extra_business_Goal_client_discovered_before_admission','ever_sent':False})
  finally:extra.destroy()
  for name in ['docking_server','waypoint_follower']:
   c.spin(1.1);c.wait(c.healthy,7);c.records.append({'qualification':'healthy_before_forbidden_node_'+name})
   forbidden=rclpy.create_node(name,namespace=c.ns)
   try:c.spin(.6);assert c.cmd('submit').code=='INPUT_UNQUALIFIED' and c.send_count()==0;c.records.append({'fault':'forbidden_node_'+name,'ever_sent':False})
   finally:forbidden.destroy_node()
 def D16(c):
  from tools.delivery.records import validate_references,decode_record,audit_database
  from tools.runner.runner_processes import ProcessSupervisor
  from tools.runner.runner_core import DeadlineBudget
  c.submit();q=c.leg();assert c.handover(q).code=='CONFIRMED';done=c.leg(True);assert done.state=='COMPLETED'
  try:audit_database(c.db)
  except ValueError as error:assert 'live Mission owner' in str(error);c.records.append({'qualification':'closed-session audit correctly rejects active owner'})
  else:raise AssertionError('closed-session audit must refuse a live owner')
  raw=c.root/'raw-fixture.log';raw.open('x').write('declared synthetic record validator fixture')
  ref={'path':raw.name,'sha256':hashlib.sha256(raw.read_bytes()).hexdigest()}
  assert validate_references(c.root,[ref])==[] and validate_references(c.root,[{**ref,'sha256':'0'*64}]) and validate_references(c.root,[{**ref,'path':'missing.log'}])
  bad=False
  try:decode_record('99 corrupt')
  except ValueError:bad=True
  assert bad
  negative=c.root/'negative-cleanup';(negative/'logs').mkdir(parents=True);supervisor=ProcessSupervisor(negative)
  proc=supervisor.spawn('fixture',[sys.executable,'-c','import os;os._exit(7)'],os.environ.copy(),c.root,critical=False);proc.process.wait(timeout=2);cleanup=supervisor.close(proc,DeadlineBudget.after(2),initial_signal=False,grace=.1)
  assert cleanup['exit']==7 and not cleanup['ok'] and not cleanup['remaining']
  outcome={'business_state':done.state,'harness':'FAILED','cleanup':cleanup,'session_pass':False,'synthetic':True};(negative/'expected-failure.json').open('x').write(json.dumps(outcome,indent=2));assert c.query().state=='COMPLETED';c.records.append({'fault':'independent_child_exit7_and_wrong_hash_or_missing_record','outcome':outcome,'meaning':'negative harness result does not erase persisted business completion'})
 def D17(c):
  r=c.cmd('queue');assert json.loads(r.detail)['battery']=='unavailable';assert c.cmd('battery_low').code=='INVALID_OPERATION';assert c.send_count()==0
 def D18(c):
  c.submit();c.wait(lambda:bool(c.query().goal_uuid),10);assert c.cmd('submit',id='q').code=='QUEUED';assert c.cmd('cancel',command_id='cancel').code=='CANCEL_PENDING';assert c.cmd('resume_schedule').code=='ACTIVE_UNRESOLVED';c.wait(lambda:c.query().state=='CANCELED',10);assert c.cmd('queue').schedule_paused and c.cmd(id='q').queue_state=='WAITING';assert c.cmd('submit',id='new').code=='QUEUED';assert c.cmd(id='new').queue_seq>c.cmd(id='q').queue_seq and c.send_count()<=1;c.records.append({'early_cancel_native_send_count':c.send_count(),'meaning':'zero if canceled before native send; at most one if late acceptance race'})
 tests={k:v for k,v in locals().items() if k.startswith('D') and callable(v)};results=[]
 try:
  for label in selected:
   c=None;error=None
   try:
    if label not in tests:raise RuntimeError('NOT_IMPLEMENTED: '+label)
    c=Case.__new__(Case);c.__init__(out/label,{'handover_ms':1500} if label=='D04' else None);tests[label](c)
   except Exception as ex:error=repr(ex);(out/(label+'-failure.txt')).write_text(traceback.format_exc())
   finally:
    if c:
     try:c.close()
     except Exception as ex:error=(error+'; ' if error else '')+repr(ex)
   row={'id':label,'status':'FAIL' if error else 'PASS','layer':'actual Mission protocol plus record/supervisor counterexamples' if label=='D16' else 'real ROS Fake Action + actual Mission/Store/ExecutionPort/Gate; synthetic TF and motion','synthetic':True,'error':error,'cleanup':'PASS' if c and all(p.poll() is not None for p,_,_,_ in c.procs) else 'UNCONFIRMED'};results.append(row);(out/'results.json').write_text(json.dumps(results,indent=2));print(json.dumps(row),flush=True)
   if error:return 1
 finally:rclpy.shutdown()
 return 0
if __name__=='__main__':raise SystemExit(main())
