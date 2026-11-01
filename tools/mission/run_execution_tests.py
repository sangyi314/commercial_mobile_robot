#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Isolated execution protocol tests; --check never creates nodes/processes/DB/output."""
import argparse,json,os,sys,time,uuid,signal,subprocess,hashlib,math,sqlite3
from pathlib import Path

def check(args):
 suite=json.loads(Path(args.suite).read_text());prefix=Path(args.install_prefix).resolve();out=Path(args.output_dir)
 if suite['version']!=1 or suite['domain']!=72 or not suite['synthetic'] or suite['cases']!=['E%02d'%n for n in range(1,17)]:raise ValueError('invalid complete matrix')
 if out.exists():raise ValueError('output exists; refuse overwrite')
 for p in ['cmr_execution/lib/cmr_execution/execution_gate','cmr_execution/lib/cmr_execution/execution_observer','cmr_execution/lib/cmr_execution/execution_test','cmr_mission/lib/cmr_mission/mission_node','cmr_mission/lib/cmr_mission/fake_nav2_server']:
  if not (prefix/p).is_file():raise ValueError('missing binary '+p)
 return suite,prefix,out

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--suite',required=True);ap.add_argument('--install-prefix',required=True);ap.add_argument('--output-dir',required=True);ap.add_argument('--cases');ap.add_argument('--check',action='store_true');args=ap.parse_args()
 suite,prefix,out=check(args)
 if args.check:print('CHECK_OK: read-only, no ROS/DB/process/output');return 0
 if os.environ.get('ROS_DOMAIN_ID')!='72' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('domain72 + loopback + FastDDS required')
 import rclpy
 from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
 from geometry_msgs.msg import Twist,TwistStamped
 from nav_msgs.msg import Odometry
 from rosgraph_msgs.msg import Clock
 from cmr_interfaces.msg import ExecutionIdentity,GateState,StopReport
 from cmr_interfaces.srv import ExecutionLease,MissionCommand,SyntheticEvent,FakeControl
 from rosidl_runtime_py.convert import message_to_ordereddict as dic
 import run_protocol_tests as prior
 prior.rclpy=rclpy;prior.MissionCommand=MissionCommand;prior.SyntheticEvent=SyntheticEvent;prior.FakeControl=FakeControl;prior.message_to_ordereddict=dic
 rclpy.init();out.mkdir(parents=True,exist_ok=False)
 (out/'manifest.json').write_text(json.dumps({'suite':suite,'suite_sha256':hashlib.sha256(Path(args.suite).read_bytes()).hexdigest(),'prefix':str(prefix),'supervisor_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'synthetic':True,'domain':72},indent=2))
 class Case(prior.Case):
  def __init__(self,root,label,config=None):
   self.root=root;root.mkdir(parents=True,exist_ok=False);self.prefix=prefix;self.bin=prefix/'cmr_mission/lib/cmr_mission';self.name='p103_'+uuid.uuid4().hex[:12]+'_'+label;self.ns='/cmr_test/'+self.name;self.action=self.ns+'/navigate_to_pose';self.config=config or {};self.procs=[];self.seq=0;self.records=[];self.node=rclpy.create_node('execution_sink',namespace=self.ns);self.db=root/'mission.db';self.server=None;self.mission=None
   self.api=self.node.create_client(MissionCommand,self.ns+'/command');self.fixture=self.node.create_client(SyntheticEvent,self.ns+'/synthetic_event');self.control=self.node.create_client(FakeControl,self.ns+'/control');self.lease_client=self.node.create_client(ExecutionLease,self.ns+'/lease')
   self.scan_graph();self.state=None;self.report=None;self.outputs=[];self.states=[];self.source_on=True;self.odom_on=True;self.candidate_on=True;self.pause=False;self.offset=0;self.fixed_stamp=None;self.extra_u=None;self.bad_candidate_time=None;self.bad_clock_time=None;self.candidate_frame='base_footprint';self.held=Twist();self.x=0.;self.yaw=0.;self.clock_ns=10000000000;self.last_source=time.monotonic();self.started=self.last_source;self.nonce=0;self.lease_id='';self.identity=None;self.grant_request=None;self.grant=None
   qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
   self.clock_pub=self.node.create_publisher(Clock,'/clock',10);self.candidate_pub=self.node.create_publisher(TwistStamped,self.ns+'/candidate',qos);self.odom_pub=self.node.create_publisher(Odometry,self.ns+'/odom',10)
   def record(kind,m):
    def safe(v):
     if isinstance(v,float) and not math.isfinite(v):return {'nonfinite':repr(v)}
     if isinstance(v,dict):return {k:safe(x) for k,x in v.items()}
     if isinstance(v,list):return [safe(x) for x in v]
     return v
    self.records.append({'kind':kind,'receive_monotonic':time.monotonic(),'data':safe(dic(m))})
   def on_state(m):self.state=m;self.states.append((time.monotonic(),m));record('gate_state',m)
   def on_report(m):self.report=m;record('stop_report',m)
   def on_final(m):self.held=m;self.outputs.append((time.monotonic(),m));record('final',m)
   self.subs=[self.node.create_subscription(GateState,self.ns+'/state',on_state,qos),self.node.create_subscription(StopReport,self.ns+'/stop_report',on_report,qos),self.node.create_subscription(Twist,self.ns+'/final',on_final,qos)]
   def source_tick():
    t=time.monotonic();dt=t-self.last_source;self.last_source=t
    if not self.pause:self.clock_ns=10000000000+int((t-self.started)*1e9)+self.offset
    clock=Clock();clock.clock.sec=self.clock_ns//1000000000;clock.clock.nanosec=self.clock_ns%1000000000
    if self.bad_clock_time:clock.clock.sec,clock.clock.nanosec=self.bad_clock_time
    self.clock_pub.publish(clock)
    if not self.source_on:return
    self.x+=self.held.linear.x*dt;self.yaw+=self.held.angular.z*dt
    odom=Odometry();odom.header.stamp=clock.clock;odom.header.frame_id='odom';odom.child_frame_id='base_footprint';odom.pose.pose.position.x=self.x;odom.pose.pose.orientation.z=math.sin(self.yaw/2);odom.pose.pose.orientation.w=math.cos(self.yaw/2);odom.twist.twist=self.held;
    if self.odom_on:self.odom_pub.publish(odom);record('synthetic_odom',odom)
    if self.candidate_on:
     cmd=TwistStamped();stamp=self.fixed_stamp if self.fixed_stamp is not None else self.clock_ns;cmd.header.stamp.sec=stamp//1000000000;cmd.header.stamp.nanosec=stamp%1000000000;cmd.header.frame_id=self.candidate_frame;cmd.twist.linear.x=.1
     if self.bad_candidate_time:cmd.header.stamp.sec,cmd.header.stamp.nanosec=self.bad_candidate_time
     if self.extra_u:
      for path,value in self.extra_u.items():a,b=path.split('.');setattr(getattr(cmd.twist,a),b,value)
     self.candidate_pub.publish(cmd);record('candidate',cmd)
   self.timer=self.node.create_timer(.02,source_tick)
   self.observer=self.start('execution_observer',{'mode':'sink','use_sim_time':False},'observer')
   params={'mode':'sink','use_sim_time':False,'run_id':self.name,'owner_epoch':self.name,'source_node':'execution_sink'};params.update(self.config)
   self.gate=self.start('execution_gate',params,'gate');self.wait_service(self.lease_client);self.wait(lambda:self.state is not None)
  def start(self,name,params,tag):
   folder='cmr_execution' if name.startswith('execution_') else 'cmr_mission';cmd=[str(self.prefix/folder/'lib'/folder/name),'--ros-args','-r','__ns:='+self.ns,'-r','clock:=/clock']
   for k,v in params.items():cmd+=['-p',k+':='+str(v).lower() if isinstance(v,bool) else k+':='+str(v)]
   log=(self.root/(tag+'.log')).open('w');p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);identity=Path('/proc/'+str(p.pid)+'/stat').read_text().split(') ')[1].split()[19];p.expected_exit=0;self.procs.append([p,log,tag,identity]);self.records.append({'spawn':cmd,'pid':p.pid,'start_ticks':identity});return p
  def spin(self,seconds):
   until=time.monotonic()+seconds
   while time.monotonic()<until:rclpy.spin_once(self.node,timeout_sec=.01)
  def request_lease(self,operation,identity=None,request=None):
   q=request or ExecutionLease.Request();q.operation=operation
   if request is None:
    self.nonce+=1;q.request_id='request_'+str(self.nonce);q.request_seq=self.nonce;q.identity=identity or self.identity or ExecutionIdentity();q.lease_id=self.lease_id;q.old_goal_resolved=True
   return q
  def lease_call(self,q):return self.call(self.lease_client,q,3)
  def arm(self):
   until=time.monotonic()+8
   while time.monotonic()<until:
    q=self.request_lease('inspect');r=self.lease_call(q)
    i=ExecutionIdentity();i.run_id=self.name;i.gate_epoch=r.gate_epoch;i.owner_epoch=self.name;i.mission_id='m';i.operation_id='m.out';i.attempt_id='m.out.0';self.identity=i
    q=self.request_lease('challenge_begin');r=self.lease_call(q)
    if r.code=='CHALLENGE':
     q.operation='begin';q.challenge=r.challenge;granted=self.lease_call(q)
     if granted.code=='GRANTED':self.lease_id=granted.lease_id;self.grant_request=q;self.grant=granted;return granted
    self.spin(.05)
   raise RuntimeError('Begin never qualified by independent initial stop window')
  def renew(self):
   q=self.request_lease('challenge_renew');r=self.lease_call(q)
   if r.code!='CHALLENGE':return r,q
   q.operation='renew';q.challenge=r.challenge;return self.lease_call(q),q
  def revoke(self):return self.lease_call(self.request_lease('revoke'))
  def keep(self,seconds):
   until=time.monotonic()+seconds
   while time.monotonic()<until:
    r,q=self.renew();assert r.code=='RENEWED',r;self.spin(.05)
  def nonzero(self):return any(abs(m.linear.x)>.01 for t,m in self.outputs)
  def zero_after(self,when):return [m for t,m in self.outputs if t>=when]
  def actual_mission(self,tag='mission',epoch=None):
   # Reuse P1-02's response-qualified restart barrier, not a graph-ready bit.
   old=self.config;self.config={**old,'use_sim_time':True,'execution_mode':'gate','execution_run_id':self.name,'execution_namespace':self.ns,'accept_ms':1500,'execute_ms':15000}
   try:prior.Case.start_mission(self,tag,epoch)
   finally:self.config=old
  def begin_actual(self,cancel_delay=200):
   self.start_server();self.ctrl('cancel_delay',cancel_delay);self.actual_mission();assert self.cmd('submit').code=='ACCEPTED';self.accepted();self.wait(lambda:abs(self.held.linear.x)>.01)
   assert self.query().state=='NAVIGATING' and self.send_count()==1
  def record_fault(self,name):self.records.append({'fault':name,'time_monotonic':time.monotonic()});return time.monotonic()
 def stopped(c):
  return c.report and c.state and c.report.verdict=="STOPPED" and c.report.generation==c.state.generation and c.report.lease_id==c.state.lease_id and c.report.clock_epoch==c.state.clock_epoch and c.report.to_ros_ns<=c.state.ros_ns and c.state.ros_ns-c.report.to_ros_ns<200000000
 def E01(c):
  c.spin(.4);assert c.outputs and not c.nonzero();r=c.lease_call(c.request_lease('inspect'));i=ExecutionIdentity();i.run_id=c.name;i.gate_epoch=r.gate_epoch;i.owner_epoch='wrong';i.mission_id='m';i.operation_id='m.out';i.attempt_id='m.out.0';assert c.lease_call(c.request_lease('challenge_begin',i)).code=='IDENTITY'
 def E02(c):
  c.arm();c.wait(c.nonzero);assert any(m.linear.x==.1 and m.angular.z==0 for t,m in c.outputs);r=c.revoke();assert r.code=='REVOKED';c.wait(lambda:c.state.state=='STOPPING');t=c.record_fault('explicit_revoke_ACK');c.spin(.15);assert c.zero_after(t) and all(m.linear.x==0 and m.angular.z==0 for m in c.zero_after(t));c.wait(lambda:stopped(c),5)
 def E03(c):
  c.arm();deadline=c.grant.deadline_gate_ms;c.spin(.02);assert c.lease_call(c.grant_request).deadline_gate_ms==deadline;bad=ExecutionLease.Request();bad=type(c.grant_request)();
  for k,v in dic(c.grant_request).items():
   if k!='identity':setattr(bad,k,v)
  bad.identity=c.grant_request.identity;bad.request_seq+=1;assert c.lease_call(bad).code=='REQUEST_CONFLICT';r,q=c.renew();assert r.code=='RENEWED';d=r.deadline_gate_ms;c.spin(.02);assert c.lease_call(q).deadline_gate_ms==d;old=c.request_lease('challenge_renew');ch=c.lease_call(old);old.operation='renew';old.challenge=ch.challenge;c.spin(.12);assert c.lease_call(old).code=='CHALLENGE_EXPIRED';c.revoke();assert c.lease_call(q).state=='STOPPING';assert c.lease_call(c.grant_request).state=='STOPPING'
 def E05(c):
  c.arm();c.wait(c.nonzero);c.fixed_stamp=c.clock_ns;c.record_fault('freeze_source_stamp_keep_receiver_and_renew_active');until=time.monotonic()+.6
  while time.monotonic()<until:
   c.renew();c.spin(.025)
  assert c.state.state=='STOPPING' and c.state.reason in ['CANDIDATE_EXPIRED','SOURCE_EXPIRED'];c.fixed_stamp=None;c.spin(.1);assert not c.state.allowed
  c.wait(lambda:stopped(c),5);assert c.lease_call(c.request_lease('reset')).code=='RESET_DISARMED';c.arm();c.wait(lambda:abs(c.held.linear.x)>.01);c.candidate_on=False;c.record_fault('candidate_stream_stopped_while_owner_renews');last_deadline=0;until=time.monotonic()+.5
  while time.monotonic()<until:
   r,q=c.renew()
   if r.code=='RENEWED':last_deadline=r.deadline_gate_ms
   c.spin(.025)
  expired=[m for t,m in c.states if m.reason=='CANDIDATE_EXPIRED'];assert expired and expired[0].gate_steady_ms<last_deadline
  assert not c.state.allowed and c.held.linear.x==0
  p=subprocess.run([str(prefix/'cmr_execution/lib/cmr_execution/execution_test'),'freshness'],capture_output=True,text=True,timeout=10);(c.root/'pure-invalid-inputs.log').write_text(p.stdout+p.stderr);assert p.returncode==0
  root=c.root
  wire=[('nan',{'extra_u':{'linear.x':float('nan')}},'INVALID_CANDIDATE'),('inf',{'extra_u':{'angular.z':float('inf')}},'INVALID_CANDIDATE'),('dimension',{'extra_u':{'linear.y':.01}},'INVALID_CANDIDATE'),('limit',{'extra_u':{'linear.x':.3}},'INVALID_CANDIDATE'),('frame',{'candidate_frame':'wrong_frame'},'INVALID_CANDIDATE'),('negative_stamp',{'bad_candidate_time':(-1,0)},'SOURCE_TIME'),('missing_stamp',{'bad_candidate_time':(0,0)},'SOURCE_TIME'),('nanoseconds',{'bad_candidate_time':(1,1000000000)},'SOURCE_TIME'),('negative_clock',{'bad_clock_time':(-1,0)},'CLOCK_INVALID'),('zero_clock',{'bad_clock_time':(0,0)},'CLOCK_INVALID')]
  for label,changes,reason in wire:
   c.close();c.__init__(root/('wire_'+label),'E05'+label);c.arm();c.wait(lambda:abs(c.held.linear.x)>.01);c.keep(.05)
   for key,value in changes.items():setattr(c,key,value)
   c.record_fault('actual_ROS_wire_'+label);c.spin(.12)
   assert c.gate.poll() is None and c.observer.poll() is None,'malformed message killed process'
   assert not c.state.allowed and c.held.linear.x==0 and c.state.reason==reason,(label,c.state.reason)
 def E06(c):
  c.arm();c.wait(c.nonzero);c.pause=True;c.record_fault('clock_pause');c.spin(.5);assert c.state.state=='STOPPING' and c.report.verdict!='STOPPED';old=c.state.clock_epoch;c.pause=False;c.offset=-5000000000;c.record_fault('clock_rollback_resume');c.wait(lambda:c.state.clock_epoch!=old);assert not c.state.allowed;c.spin(.15);assert all(m.linear.x==0 for m in c.zero_after(time.monotonic()-.1));c.wait(lambda:stopped(c),5)
 def E07(c):
  c.arm();old=c.grant_request;c.injected_exit(c.gate);c.gate=c.start('execution_gate',{'mode':'sink','use_sim_time':False,'run_id':c.name,'owner_epoch':c.name,'source_node':'execution_sink'},'gate-restarted');c.wait(lambda:c.state.state=='DISARMED' and not c.state.clock_epoch.startswith(old.identity.gate_epoch+'.clock.'));c.node.destroy_client(c.lease_client);c.lease_client=c.node.create_client(ExecutionLease,c.ns+'/lease')
  # Response-qualified read-only restart barrier; graph entries can outlive SIGKILL.
  end=time.monotonic()+10;ready=None
  while time.monotonic()<end:
   if not c.lease_client.wait_for_service(timeout_sec=.05):c.spin(.02);continue
   q=c.request_lease('inspect');f=c.lease_client.call_async(q);until=time.monotonic()+.6
   while not f.done() and time.monotonic()<until:rclpy.spin_once(c.node,timeout_sec=.02)
   if f.done() and f.result() and f.result().gate_epoch!=old.identity.gate_epoch:
    ready=f.result();c.records.append({'restart_readiness_request':dic(q),'restart_readiness_response':dic(ready)});break
   c.lease_client.remove_pending_request(f);c.records.append({'restart_readiness_probe':'NO_RESPONSE','request':dic(q)})
  assert ready is not None,'new Gate never responded';assert c.lease_call(old).code=='IDENTITY';assert not c.state.allowed
 def E08(c):
  extra=c.node.create_publisher(TwistStamped,c.ns+'/candidate',10);c.spin(.4);r=c.lease_call(c.request_lease('inspect'));assert r.state=='DISARMED';i=ExecutionIdentity();i.run_id=c.name;i.gate_epoch=r.gate_epoch;i.owner_epoch=c.name;i.mission_id='m';i.operation_id='m.out';i.attempt_id='m.out.0';q=c.request_lease('challenge_begin',i);ch=c.lease_call(q);q.operation='begin';q.challenge=ch.challenge;assert c.lease_call(q).code=='INPUT_UNQUALIFIED';c.records.append({'fault':'extra_candidate_endpoint','publishers':len(c.node.get_publishers_info_by_topic(c.ns+'/candidate'))});c.node.destroy_publisher(extra)
  code="import rclpy,time,sys\nfrom geometry_msgs.msg import Twist\nrclpy.init();n=rclpy.create_node('wrong_type_probe',namespace=sys.argv[1]);p=n.create_publisher(Twist,sys.argv[1]+'/candidate',10);print('WRONG_TYPE_READY',flush=True);end=time.monotonic()+1.0\nwhile time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)\nn.destroy_node();rclpy.shutdown()"
  cmd=[sys.executable,'-c',code,c.ns];log=(c.root/'wrong-type.log').open('w');p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);identity=Path('/proc/'+str(p.pid)+'/stat').read_text().split(') ')[1].split()[19];p.expected_exit=0;c.procs.append([p,log,'wrong-type',identity]);c.records.append({'spawn':cmd,'pid':p.pid,'start_ticks':identity});c.wait(lambda:any(p.topic_type=='geometry_msgs/msg/Twist' for p in c.node.get_publishers_info_by_topic(c.ns+'/candidate')));c.spin(.15);assert not c.state.allowed;c.records.append({'fault':'wrong_candidate_type_separate_participant','types':[p.topic_type for p in c.node.get_publishers_info_by_topic(c.ns+'/candidate')]});p.wait(timeout=3);assert p.returncode==0
  final=c.node.create_publisher(Twist,c.ns+'/final',10);c.spin(.25);assert not c.state.allowed;c.records.append({'fault':'extra_final_publisher','count':len(c.node.get_publishers_info_by_topic(c.ns+'/final'))});c.node.destroy_publisher(final)
  bridge=rclpy.create_node('execution_bridge',namespace=c.ns);subscription=bridge.create_subscription(Twist,c.ns+'/final',lambda m:None,10);c.spin(.25);assert not c.state.allowed;c.records.append({'fault':'bridge_consumer_in_sink','actual_bridge_started':False});bridge.destroy_subscription(subscription);bridge.destroy_node()
 def E10(c):
  c.spin(.8);idle=[s.gate_steady_ms for t,s in c.states if s.trigger=='tick'];assert len(idle)>10
  cmd=[sys.executable,'-c','import time,math\nend=time.monotonic()+2\nx=0.1\nwhile time.monotonic()<end:x=math.sin(x)+0.1']
  log=(c.root/'owned-load.log').open('w');p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);identity=Path('/proc/'+str(p.pid)+'/stat').read_text().split(') ')[1].split()[19];p.expected_exit=0;c.procs.append([p,log,'owned-load',identity]);c.records.append({'spawn':cmd,'pid':p.pid,'start_ticks':identity});c.spin(2.2)
  ticks=[s.gate_steady_ms for t,s in c.states if s.trigger=='tick'];gaps=[b-a for a,b in zip(ticks,ticks[1:])];assert gaps and max(gaps)<300;(c.root/'scheduling.json').write_text(json.dumps({'samples':len(ticks),'max_publication_gap_ms':max(gaps),'p50_ms':sorted(gaps)[len(gaps)//2],'p99_ms':sorted(gaps)[int(len(gaps)*.99)],'idle_samples':len(idle),'clock':'Gate local steady, same process','hard_realtime':False},indent=2))
 def E11(c):
  c.arm();c.wait(lambda:abs(c.held.linear.x)>.01);os.killpg(c.gate.pid,signal.SIGSTOP);c.record_fault('Gate_SIGSTOP_holding_sink');c.spin(.5);assert c.held.linear.x==.1
  os.killpg(c.gate.pid,signal.SIGCONT);c.record_fault('Gate_SIGCONT_must_expire_before_any_new_nonzero');c.wait(lambda:c.state.state=='STOPPING' and c.held.linear.x==0,2);c.wait(lambda:stopped(c),5);assert c.lease_call(c.request_lease('reset')).code=='RESET_DISARMED';c.arm();c.wait(lambda:abs(c.held.linear.x)>.01);c.injected_exit(c.gate);t=c.record_fault('Gate_SIGKILL_with_last_command_holding_synthetic_sink');c.spin(.5);assert c.held.linear.x==.1;(c.root/'unsupported.json').write_text(json.dumps({'protection':'UNSUPPORTED','physical_stop':'STOP_UNKNOWN','holding_sink_value':c.held.linear.x,'after_s':time.monotonic()-t,'synthetic':True,'meaning':'negative test reproduced missing downstream watchdog; not safety PASS'},indent=2))
  root=c.root;c.close();c.__init__(root/'downstream_link','E11link');c.arm();c.wait(lambda:c.held.linear.x==.1);c.node.destroy_subscription(c.subs[2]);c.record_fault('logical_sink_transport_detached_holds_last_command');c.revoke();c.spin(.5)
  assert c.state.output.linear.x==0 and c.held.linear.x==.1 and c.report.verdict!='STOPPED'
  c.records.append({'protection':'UNSUPPORTED','boundary':'logical downstream receive subscription absent','synthetic':True,'actual_bridge_started':False})
  def reconnected(m):c.held=m;c.outputs.append((time.monotonic(),m));c.records.append({'kind':'reconnected_final','receive_monotonic':time.monotonic(),'data':dic(m)})
  subscription=c.node.create_subscription(Twist,c.ns+'/final',reconnected,10);c.record_fault('volatile_depth1_reconnect_after_revoke');c.wait(lambda:c.held.linear.x==0,1);c.spin(.2)
  received=[x for x in c.records if x.get('kind')=='reconnected_final'];assert received and all(x['data']['linear']['x']==0 for x in received);assert not c.state.allowed;c.wait(lambda:stopped(c),5)
 def E13(c):
  c.arm();c.wait(c.nonzero);r=c.revoke();t=c.record_fault('revoke_linearized');c.spin(.2);assert c.zero_after(t) and all(m.linear.x==0 for m in c.zero_after(t));assert all(s.output.linear.x==0 for tm,s in c.states if s.control_seq>=r.control_seq)
 def E14(c):
  p=subprocess.run([str(prefix/'cmr_execution/lib/cmr_execution/execution_test'),'boundaries'],capture_output=True,text=True,timeout=10);(c.root/'pure-boundaries.log').write_text(p.stdout+p.stderr);assert p.returncode==0
 def E15(c):
  c.arm();c.wait(c.nonzero);c.revoke();old_generation=c.state.generation;c.wait(lambda:stopped(c),5);r=c.lease_call(c.request_lease('reset'));assert r.code=='RESET_DISARMED';c.fixed_stamp=c.clock_ns;c.arm();c.spin(.15);assert c.state.generation>old_generation and not any(m.linear.x!=0 for m in c.zero_after(time.monotonic()-.1))
 def E16(c):
  c.arm();c.revoke();r=c.lease_call(c.request_lease('reset'));assert r.code=='RESET_UNQUALIFIED';assert not c.state.allowed
  for tag,params in [('invalid-mode',{'mode':'production','use_sim_time':False}),('bad-lease',{'mode':'sink','use_sim_time':False,'run_id':c.name,'owner_epoch':c.name,'source_node':'execution_sink','lease_ms':0}),('unknown-option',{'mode':'sink','use_sim_time':False,'run_id':c.name,'owner_epoch':c.name,'source_node':'execution_sink','unexpected_option':1})]:
   p=c.start('execution_gate',params,tag);p.expected_exit=2;p.wait(timeout=5);assert p.returncode==2
  p=c.start('execution_observer',{'mode':'simulation','use_sim_time':False},'observer-invalid-domain');p.expected_exit=2;p.wait(timeout=5);assert p.returncode==2
 def E04(c):
  c.begin_actual();assert c.held.linear.x==.1;os.killpg(c.mission.pid,signal.SIGSTOP);t=c.record_fault('actual_Mission_SIGSTOP_no_fixture_killed')
  c.wait(lambda:c.state.state=='STOPPING' and c.held.linear.x==0,1);assert time.monotonic()-t<.6;assert c.state.reason=='LEASE_EXPIRED';assert c.send_count()==1
  os.killpg(c.mission.pid,signal.SIGCONT);c.record_fault('actual_Mission_SIGCONT_no_auto_rearm');c.wait(lambda:c.query().state in ['FAILED','CANCELED'],8);assert not c.state.allowed
  root=c.root;c.close();c.__init__(root/'crash','E04crash');c.begin_actual();c.injected_exit(c.mission);t=c.record_fault('actual_Mission_SIGKILL')
  c.wait(lambda:c.state.state=='STOPPING' and c.held.linear.x==0,1);assert time.monotonic()-t<.6 and c.send_count()==1
  c.actual_mission('mission-restarted',c.name+'_restart');assert c.query().state=='PAUSED';assert c.cmd('submit',id='other').code=='BUSY';c.spin(.2);assert c.send_count()==1 and not c.state.allowed
  c.close();c.__init__(root/'database_busy','E04db');c.begin_actual();lock=sqlite3.connect(c.db,timeout=0);until=time.monotonic()+1;acquired=False
  while time.monotonic()<until:
   try:lock.execute('BEGIN IMMEDIATE');acquired=True;break
   except sqlite3.OperationalError as ex:
    if 'locked' not in str(ex):lock.close();raise
    c.spin(.005)
  if not acquired:lock.close();raise RuntimeError('NOT_EXERCISED: external DB writer never acquired lock')
  assert c.state.allowed and c.held.linear.x==.1;t=c.record_fault('external_SQLite_writer_blocks_real_owner_renew_transaction')
  try:
   c.wait(lambda:not c.state.allowed and c.held.linear.x==0,1);assert time.monotonic()-t<.6 and c.state.reason in ['LEASE_EXPIRED','REVOKED'];assert c.send_count()==1
   assert 'store fault latched' in (c.root/'mission.log').read_text()
   c.records.append({'database_fault_safety_path':c.state.reason,'meaning':'DB failure actively revokes when possible; otherwise no renewal and lease expiry'})
   c.spin(.3);assert not c.state.allowed
  finally:lock.rollback();lock.close()
  c.spin(.3);assert not c.state.allowed and c.send_count()==1
 def E09(c):
  c.begin_actual(4000);c.odom_on=False;assert c.cmd('cancel').code=='CANCEL_PENDING';c.wait(lambda:c.state.state=='STOPPING');c.spin(3.1)
  fake=c.node.create_publisher(StopReport,c.ns+'/stop_report',10);m=StopReport();m.identity=c.state.identity;m.clock_epoch=c.state.clock_epoch;m.lease_id=c.state.lease_id;m.generation=c.state.generation;m.synthetic=True;m.verdict='STOPPED';m.reason='TEST_FORGERY';m.evidence_ref='not_an_observer_window';m.to_ros_ns=c.state.ros_ns;m.from_ros_ns=m.to_ros_ns-3000000000;m.samples=31;m.max_gap_ns=100000000
  c.record_fault('non_observer_stop_report_same_wire_topic');fake.publish(m);c.spin(.15);assert c.query().state=='PAUSED' and c.cmd('submit',id='other').code=='BUSY';assert c.fixture_event('stop').code=='FIXTURE_STOP_PROHIBITED'
  c.node.destroy_publisher(fake);c.odom_on=True;c.wait(lambda:c.query().state=='CANCELED',8)
 def E12(c):
  c.begin_actual(4000);assert c.cmd('cancel').code=='CANCEL_PENDING';c.wait(lambda:c.event_count(contains='CANCEL_RESPONSE')>0);assert c.query().state=='PAUSED' and c.cmd('submit',id='other').code=='BUSY'
  c.wait(lambda:stopped(c),5);assert c.event_count(18)==0 and c.cmd('submit',id='other').code=='BUSY';c.wait(lambda:c.query().state=='CANCELED',5)
  root=c.root;c.close();c.__init__(root/'terminal_without_D','E12terminal');c.begin_actual();c.odom_on=False;assert c.ctrl('success').success;c.wait(lambda:c.event_count(2)>0);assert c.cmd('cancel').code=='CANCEL_PENDING';c.spin(3.3);assert c.query().state=='PAUSED' and c.cmd('submit',id='other').code=='BUSY';c.record_fault('Goal_terminal_and_final_zero_but_D_missing')
  c.odom_on=True;c.wait(lambda:c.query().state=='CANCELED',8)
 tests={k:v for k,v in locals().items() if k.startswith('E') and callable(v)}
 results=[]
 try:
  for caseid in (args.cases.split(',') if args.cases else suite['cases']):
   c=None;error=None
   try:c=Case.__new__(Case);c.__init__(out/caseid,caseid);tests[caseid](c)
   except Exception as e:error=repr(e)
   finally:
    if c:
     try:c.close()
     except Exception as e:error=(error+'; ' if error else '')+repr(e)
   row={'id':caseid,'status':'FAIL' if error else 'PASS','layer':('pure core (ROS harness only)' if caseid=='E14' else 'actual MissionRuntime+Gate+Fake Action+synthetic motion sink' if caseid in ['E04','E09','E12'] else 'ROS sink malformed wire + pure invalid-input cases' if caseid=='E05' else 'ROS graph/startup rejection' if caseid in ['E08','E16'] else 'ROS sink synthetic'),'error':error,'fault_activation':'NOT_EXERCISED' if error and 'NOT_EXERCISED' in error else 'CONFIRMED' if c and any('fault' in x for x in c.records) else 'NOT_APPLICABLE','safety_protection':'UNSUPPORTED' if caseid=='E11' else 'DECLARED_LAYER_ONLY','cleanup':'PASS' if c and all(p.poll() is not None for p,log,tag,identity in c.procs) else 'FAIL'};results.append(row);(out/'results.json').write_text(json.dumps(results,indent=2));print(json.dumps(row),flush=True)
   if error:return 1
 finally:rclpy.shutdown()
 return 0
if __name__=='__main__':
 try:raise SystemExit(main())
 except Exception as e:print('EXECUTION_TEST_ERROR',repr(e),file=sys.stderr);raise SystemExit(2)
