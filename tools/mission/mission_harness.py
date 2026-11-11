# SPDX-License-Identifier: MIT
"""Shared isolated ROS Mission harness for the Action, Execution and Delivery matrices."""

import hashlib,json,os,signal,sqlite3,subprocess,time,uuid
from contextlib import closing
from pathlib import Path

import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import Twist,TwistStamped,TransformStamped,PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry,OccupancyGrid
from rosgraph_msgs.msg import Clock
from cmr_interfaces.msg import GateState,StopReport
from cmr_interfaces.srv import DeliveryCommand,FakeControl,ExecutionLease
from rosidl_runtime_py.convert import message_to_ordereddict as dic

class ProcessCase:
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
 def start_server(self,tag='server'):
  self.server=self.start('fake_nav2_server',{'action_name':self.action},tag);self.wait_service(self.control)
  if tag!='server-extra':self.validate_action_graph()
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
  self.records.append({'request':dic(request),'response':dic(result),'time_monotonic':time.monotonic(),'service':client.srv_name});return result
 def ctrl(self,op,value=0,goal=''):
  q=FakeControl.Request();q.command=op;q.case_id=self.name;q.goal_uuid=goal;q.value_ms=value;return self.call(self.control,q)
 def expect_state(self,state):self.wait(lambda:self.query().state==state)
 def accepted(self):self.wait(lambda:bool(self.query().goal_uuid));return self.query()
 def event_count(self,kind=None,contains=None):
  if not self.db.exists():return 0
  with closing(sqlite3.connect(self.db,timeout=1)) as db:
   if contains:return db.execute('SELECT count(*) FROM events WHERE identity LIKE ?',('%'+contains+'%',)).fetchone()[0]
   return db.execute('SELECT count(*) FROM events WHERE kind=?',(str(kind),)).fetchone()[0]
 def wait_result(self):self.wait(lambda:self.event_count(2)+self.event_count(3)+self.event_count(18)>0)
 def send_count(self):
  return sum(p.read_text().count('GOAL_REQUEST ') for p in self.root.glob('server*.log'))
 def snapshot(self):
  if self.db.exists():
   with closing(sqlite3.connect(self.db)) as db:self.records.append({'database':{t:db.execute('SELECT * FROM '+t).fetchall() for t in ['missions','manager','events','operations']}})
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
  try:
   self.snapshot()
  finally:
   self.node.destroy_node()
   (self.root/'trace.json').write_text(json.dumps(self.records,indent=2))
   (self.root/'cleanup.json').write_text(json.dumps(cleanup,indent=2))
  assert not any('escalation' in x for x in cleanup),'forced cleanup'
  assert all(x.get('exit')==x.get('expected_exit') for x in cleanup),'unexpected process exit'

class DeliveryCase(ProcessCase):
 def __init__(self,root,prefix,config=None,*,start_server=True,cancel_delay=None):
  self.root=root;root.mkdir(parents=True,exist_ok=False);self.prefix=prefix;self.bin=prefix/'cmr_mission/lib/cmr_mission';self.name='p104_'+uuid.uuid4().hex[:10];self.ns='/cmr_test/'+self.name;self.action=self.ns+'/navigate_to_pose';self.config=config or {};self.procs=[];self.seq=0;self.records=[];self.node=rclpy.create_node('execution_sink',namespace=self.ns);self.db=root/'mission.db';self.server=None;self.mission=None
  self.delivery_qos=QoSProfile(depth=64,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
  self.control=self.node.create_client(FakeControl,self.ns+'/control');self.delivery=self.node.create_client(DeliveryCommand,self.ns+'/delivery',qos_profile=self.delivery_qos);self.lease=self.node.create_client(ExecutionLease,self.ns+'/lease');self.scan_graph()
  self.state=None;self.report=None;self.held=Twist();self.clock_paused=False;self.clock_offset_ns=0;self.x=0.;self.source_on=True;self.odom_on=True;self.tf_on=True;self.clock_ns=10000000000;self.started=time.monotonic();self.last_tick=self.started
  rel=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE);durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.clock_pub=self.node.create_publisher(Clock,self.ns+'/clock',rel);self.candidate_pub=self.node.create_publisher(TwistStamped,self.ns+'/candidate',rel);self.odom_pub=self.node.create_publisher(Odometry,self.ns+'/odom',rel);self.tf_pub=self.node.create_publisher(TFMessage,self.ns+'/tf',100);self.map_pub=self.node.create_publisher(OccupancyGrid,self.ns+'/map',durable);self.amcl_pub=self.node.create_publisher(PoseWithCovarianceStamped,self.ns+'/amcl_pose',durable);self.scan_pub=self.node.create_publisher(LaserScan,self.ns+'/scan',rel)
  def record(kind,m):self.records.append({'kind':kind,'receive_monotonic':time.monotonic(),'data':dic(m)})
  def state(m):self.state=m;record('gate_state',m)
  def stop(m):self.report=m;record('stop_report',m)
  def final(m):self.held=m;record('final',m)
  self.subs=[self.node.create_subscription(GateState,self.ns+'/state',state,rel),self.node.create_subscription(StopReport,self.ns+'/stop_report',stop,rel),self.node.create_subscription(Twist,self.ns+'/final',final,rel)]
  for f in ['map.yaml','map.pgm','bt.xml']:(root/f).write_text('synthetic P1-04 fixture '+f)
  ref=lambda f:{'path':str((root/f).resolve()),'sha256':hashlib.sha256((root/f).read_bytes()).hexdigest()}
  pose=lambda x:{'frame':'map','position':{'x':x,'y':0.,'z':0.},'orientation':{'x':0.,'y':0.,'z':0.,'w':1.}}
  mp=ref('map.yaml');im=ref('map.pgm');self.catalog=root/'catalog.json';self.catalog.write_text(json.dumps({'schema_version':1,'map_version':'map','station_version':'stations','map':{'yaml_path':mp['path'],'yaml_sha256':mp['sha256'],'image_path':im['path'],'image_sha256':im['sha256'],'metadata':{'width':100,'height':100,'resolution':.05,'origin_x':0.,'origin_y':0.,'origin_yaw':0.,'grid_sha256':hashlib.sha256(bytes(10000)).hexdigest()}},'bt':ref('bt.xml'),'bounds':{'min_x':-1.,'max_x':1.,'min_y':-1.,'max_y':1.},'stations':[{'station_id':i,'map_version':'map','station_version':'stations','pose':pose(x)} for i,x in [('a',.1),('b',0.)]]}))
  next_map=0.
  def source():
   nonlocal next_map
   now=time.monotonic()
   dt=now-self.last_tick
   self.last_tick=now
   if not self.clock_paused:self.clock_ns=10000000000+int((now-self.started)*1e9)+self.clock_offset_ns
   clock=Clock();clock.clock.sec=self.clock_ns//1000000000;clock.clock.nanosec=self.clock_ns%1000000000;self.clock_pub.publish(clock)
   if not self.source_on:return
   self.x+=self.held.linear.x*dt
   o=Odometry();o.header.stamp=clock.clock;o.header.frame_id='odom';o.child_frame_id='base_footprint';o.pose.pose.position.x=self.x;o.pose.pose.orientation.w=1.;o.twist.twist=self.held
   if self.odom_on:self.odom_pub.publish(o);record('synthetic_odom',o)
   if self.tf_on:
    tr=TransformStamped();tr.header.stamp=clock.clock;tr.header.frame_id='map';tr.child_frame_id='base_footprint';tr.transform.translation.x=self.x;tr.transform.rotation.w=1.;self.tf_pub.publish(TFMessage(transforms=[tr]));record('synthetic_tf',tr)
   # Refresh the static map for graph discovery without serializing it on every sensor tick.
   if now>=next_map:
    grid=OccupancyGrid()
    grid.header.stamp=clock.clock
    grid.header.frame_id='map'
    grid.info.width=grid.info.height=100
    grid.info.resolution=.05
    grid.info.origin.orientation.w=1.
    grid.data=[0]*10000
    self.map_pub.publish(grid)
    next_map=now+.5
   amcl=PoseWithCovarianceStamped();amcl.header.stamp=clock.clock;amcl.header.frame_id='map';amcl.pose.pose=o.pose.pose;amcl.pose.covariance[0]=amcl.pose.covariance[7]=amcl.pose.covariance[35]=.01;self.amcl_pub.publish(amcl)
   scan=LaserScan();scan.header.stamp=clock.clock;scan.header.frame_id='base_scan';scan.range_min=.1;scan.range_max=10.;scan.ranges=[1.]*10;self.scan_pub.publish(scan)
   cmd=TwistStamped();cmd.header.stamp=clock.clock;cmd.header.frame_id='base_footprint';cmd.twist.linear.x=self.candidate_velocity();self.candidate_pub.publish(cmd);record('candidate',cmd)
  self.timer=self.node.create_timer(.02,source)
  self.observer=self.start_native('cmr_execution','execution_observer',{'mode':'sink','use_sim_time':False},'observer');self.gate=self.start_gate(self.name,'gate')
  if start_server:
   self.start_server()
   if cancel_delay is not None:assert self.ctrl('cancel_delay',cancel_delay).success
  self.start_delivery_node()
 def candidate_velocity(self):
  target=0. if self.state and '.return' in self.state.identity.operation_id else .1
  return (.1 if target>self.x else -.1) if abs(target-self.x)>.005 else 0.
 def start_gate(self,epoch,tag):
  return self.start_native('cmr_execution','execution_gate',{
   'mode':'sink','use_sim_time':False,'run_id':self.name,
   'owner_epoch':epoch,'source_node':'execution_sink',
  },tag)
 def restart_idle_gate(self,epoch):
  assert self.mission.poll() is not None and self.send_count()==0
  assert self.state.state=='DISARMED' and not self.state.allowed
  previous=self.state.clock_epoch
  os.killpg(self.gate.pid,signal.SIGINT)
  self.gate.wait(timeout=5)
  assert self.gate.returncode==0
  self.gate=self.start_gate(epoch,'gate-restarted')
  self.wait(lambda:self.state.clock_epoch!=previous and self.state.state=='DISARMED')
 def start(self,name,params,tag):
  return self.start_native('cmr_mission',name,params,tag)
 def start_native(self,pkg,exe,params,tag):
  executable=self.prefix/pkg/'lib'/pkg/exe
  cmd=[str(executable),'--ros-args','-r','__ns:='+self.ns,
       '-r','/tf:='+self.ns+'/tf','-r','/tf_static:='+self.ns+'/tf_static']
  for key,value in params.items():
   formatted=str(value).lower() if isinstance(value,bool) else str(value)
   cmd.extend(['-p',f'{key}:={formatted}'])
  log=(self.root/(tag+'.log')).open('x')
  try:
   process=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  except OSError:
   log.close()
   raise
  process.expected_exit=0
  owned=[process,log,tag,None]
  self.procs.append(owned)
  stat=Path(f'/proc/{process.pid}/stat').read_text()
  owned[3]=stat.split(') ')[1].split()[19]
  self.records.append({'spawn':cmd,'pid':process.pid,'start_ticks':owned[3]})
  return process
 def start_delivery_node(self,epoch=None,tag='mission'):
  params={
   'test_mode':True,'use_sim_time':False,'execution_run_id':self.name,'execution_namespace':self.ns,
   'action_name':self.action,'database':str(self.db),'process_epoch':epoch or self.name,
   'station_catalog':str(self.catalog.resolve()),'task_ms':60000,'handover_ms':10000,
   'execute_ms':15000,**self.config,
  }
  self.mission=self.start_native('cmr_mission','mission_node',params,tag)
  if tag!='mission':
   self.node.destroy_client(self.delivery)
   self.delivery=self.node.create_client(DeliveryCommand,self.ns+'/delivery',qos_profile=self.delivery_qos)
  self.wait_service(self.delivery)
  self.wait(lambda:self.state is not None)
  # Only a response qualifies a new owner. A killed node can remain in the graph.
  until=time.monotonic()+10
  while time.monotonic()<until:
   request=DeliveryCommand.Request(operation='query',mission_id='__owner_readiness__')
   future=self.delivery.call_async(request)
   deadline=time.monotonic()+.6
   while not future.done() and time.monotonic()<deadline:
    self.spin(.01)
   if future.done() and future.result() is not None:
    self.records.append({'owner_readiness_request':dic(request),'owner_readiness_response':dic(future.result())})
    return
   self.delivery.remove_pending_request(future)
   self.records.append({'owner_readiness_probe':'NO_RESPONSE'})
  raise RuntimeError('Mission owner never answered readiness query')
 def wait(self,predicate,seconds=7):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   if predicate():return
   self.spin(.01)
  raise RuntimeError('condition timeout')
 def wait_new_goal(self,previous,seconds=12):
  result=None
  def ready():
   nonlocal result
   result=self.query();return bool(result.goal_uuid) and result.goal_uuid!=previous
  self.wait(ready,seconds);return result
 def request(self,op='query',id='m',command_id=None,**values):
  q=DeliveryCommand.Request();q.operation=op;q.mission_id=id;q.command_id=command_id or 'c'+str(self.seq);self.seq+=1;q.schema_version=1;q.delivery_station='a';q.return_station='b';q.map_version='map';q.station_version='stations';q.created_at_unix_ms=1
  for k,v in values.items():setattr(q,k,v)
  return q
 def cmd(self,op='query',id='m',**values):return self.call(self.delivery,self.request(op,id,**values))
 def query(self):return self.cmd()
 def read_record(self,id='m'):
  from tools.delivery.records import decode_record
  return decode_record(json.loads(self.cmd(id=id).detail)['record'])
 def assert_slot_occupied(self,reason='BUSY'):
  assert self.cmd('submit',id='other').code==reason
  assert self.cmd(id='other').code=='NOT_FOUND'
  assert self.query().queue_state=='ACTIVE'
 def record_fault(self,name):
  stamp=time.monotonic()
  self.records.append({'fault':name,'time_monotonic':stamp})
  return stamp
 def healthy(self):
  lines=(self.root/'mission.log').read_text().splitlines()
  values=[line.removeprefix('DELIVERY_HEALTH ') for line in lines if line.startswith('DELIVERY_HEALTH ')]
  return bool(values and json.loads(values[-1])['qualified'])
 def wait_idle_stop(self):
  def qualified():
   report=self.report
   state=self.state
   return (state is not None and state.state=='DISARMED' and not state.allowed
           and report is not None and report.verdict=='STOPPED'
           and report.identity==state.identity and report.generation==state.generation
           and report.clock_epoch==state.clock_epoch and report.lease_id==state.lease_id
           and 0<=state.ros_ns-report.to_ros_ns<200000000)
  self.wait(qualified)
  self.records.append({'qualification':'observer_has_current_idle_stop_window'})
 def submit(self,id='m'):
  end=time.monotonic()+6
  while time.monotonic()<end:
   r=self.cmd('submit',id=id)
   if r.code=='ACCEPTED':return r
   if r.code!='INPUT_UNQUALIFIED':raise RuntimeError('unexpected submit '+r.code+' '+r.detail)
   rclpy.spin_once(self.node,timeout_sec=.02)
  raise RuntimeError('delivery never healthy')
 def spin(self,seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=.01)
 def leg(self,returning=False,id='m'):
  self.wait(lambda:bool(self.cmd(id=id).goal_uuid),10);target=0. if returning else .1;self.wait(lambda:abs(self.x-target)<.015,5);r=self.cmd(id=id);assert self.ctrl('success',goal=r.goal_uuid).success;expected='COMPLETED' if returning else 'WAIT_HANDOVER';self.wait(lambda:self.cmd(id=id).state==expected,10);return self.cmd(id=id)
 def handover(self,q,id='m',command='handover'):
  return self.cmd('handover',id=id,command_id=command,handover_id=q.handover_id,handover_entry_version=q.handover_entry_version,source='synthetic')
 def snapshot(self):
  if self.db.exists():
   from tools.delivery.records import read_database
   self.records.append({'database':read_database(self.db)})
