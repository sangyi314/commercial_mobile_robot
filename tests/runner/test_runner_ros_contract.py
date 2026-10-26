"""Synthetic ROS interfaces in domain72 only. Never a navigation baseline."""
import copy,json,os,signal,subprocess,sys,threading,time,uuid
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from runner_core import DeadlineBudget,write_json
from runner_processes import ProcessSupervisor
from run_trial import WorkerLink

@pytest.fixture(scope='module')
def ros():
 assert os.environ.get('CMR_TEST_ROS_DOMAIN_ID')=='72' and os.environ.get('ROS_DOMAIN_ID')=='72'
 import rclpy
 from rclpy.executors import MultiThreadedExecutor
 rclpy.init();probe=rclpy.create_node('cmr_test_inventory');end=time.monotonic()+1
 while time.monotonic()<end:rclpy.spin_once(probe,timeout_sec=.05)
 outsiders=[x for x in probe.get_node_names() if x!='cmr_test_inventory' and not x.startswith('_ros2cli_daemon')]
 assert not outsiders,outsiders
 probe.destroy_node();ex=MultiThreadedExecutor(num_threads=6);thread=threading.Thread(target=ex.spin,daemon=True);thread.start();yield rclpy,ex
 ex.shutdown(timeout_sec=2);thread.join(timeout=2);rclpy.shutdown()
class Fake:
 def __init__(self,rclpy,ex,ns,mode='success',wrong_frame=False):
  from rclpy.callback_groups import ReentrantCallbackGroup
  from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
  from rclpy.action import ActionServer,GoalResponse,CancelResponse
  from nav2_msgs.action import NavigateToPose
  from nav2_msgs.srv import ManageLifecycleNodes
  from std_srvs.srv import Trigger
  from lifecycle_msgs.srv import GetState
  from nav_msgs.msg import Odometry,OccupancyGrid
  from sensor_msgs.msg import LaserScan
  from rosgraph_msgs.msg import Clock
  from geometry_msgs.msg import TransformStamped,PoseWithCovarianceStamped
  from std_msgs.msg import String
  from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
  self.ns=ns;self.mode=mode;self.gate_calls=0;self.startups=0;self.pending=0;self.count=0;self.cancels=0;self.sim=0.;self.paused=False;self.wrong=wrong_frame;self.ex=ex;self.rclpy=rclpy
  self.n=rclpy.create_node('fake',namespace='/'+ns,cli_args=['--ros-args','-r','/tf:=/'+ns+'/tf','-r','/tf_static:=/'+ns+'/tf_static']);group=ReentrantCallbackGroup();self.group=group
  self.pubs={k:self.n.create_publisher(t,k,(QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL) if k=='amcl_pose' else 100) if k not in ['clock','scan','odom'] else qos_profile_sensor_data) for k,t in [('clock',Clock),('scan',LaserScan),('odom',Odometry),('amcl_pose',PoseWithCovarianceStamped),('initialpose',PoseWithCovarianceStamped),('cmr_experiment/events',String)]}
  self.globalclock=self.n.create_publisher(Clock,'/clock',qos_profile_sensor_data);self.tf=TransformBroadcaster(self.n);self.static=StaticTransformBroadcaster(self.n)
  tf=[]
  for parent,child in [('base_footprint','base_link'),('base_link','base_scan')]:
   t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.;tf.append(t)
  self.static.sendTransform(tf)
  self.n.create_subscription(PoseWithCovarianceStamped,'initialpose',lambda m:None,10)
  def guarded(fn):
   def wrapped(*args):
    self.pending+=1
    try:return fn(*args)
    finally:self.pending-=1
   return wrapped
  def state(req,res):res.current_state.id=3;res.current_state.label='active';return res
  for node in ['map_server','amcl']:self.n.create_service(GetState,'/'+ns+'/'+node+'/get_state',state,callback_group=group)
  def manage(req,res):
   if req.command==req.STARTUP:self.startups+=1
   if mode=='manager_late':time.sleep(.5)
   if mode=='manager_never':time.sleep(2)
   res.success=mode!='manager_false';return res
  self.n.create_service(ManageLifecycleNodes,'lifecycle_manager_localization/manage_nodes',guarded(manage),callback_group=group)
  def inactive(req,res):
   self.gate_calls+=1
   if mode=='gate_lost_once' and self.gate_calls==1:time.sleep(2.2)
   res.success=False;return res
  self.n.create_service(Trigger,'lifecycle_manager_localization/is_active',guarded(inactive),callback_group=group)
  def goal(req):
   self.count+=1
   if mode=='accept_late':time.sleep(.4)
   return GoalResponse.REJECT if mode=='reject' else GoalResponse.ACCEPT
  def cancel(handle):
   self.cancels+=1
   if mode=='no_cancel':time.sleep(1.2)
   return CancelResponse.ACCEPT
  def execute(handle):
   limit=time.monotonic()+(2 if mode in ['cancel','no_cancel','accept_late'] else .15)
   while time.monotonic()<limit:
    if handle.is_cancel_requested:
     time.sleep(.15);handle.canceled();return NavigateToPose.Result()
    time.sleep(.01)
   if mode=='aborted':handle.abort()
   else:handle.succeed()
   return NavigateToPose.Result()
  self.server=ActionServer(self.n,NavigateToPose,'navigate_to_pose',execute_callback=guarded(execute),goal_callback=guarded(goal),cancel_callback=guarded(cancel),callback_group=group)
  self.timer=self.n.create_timer(.02,self.tick,callback_group=group);ex.add_node(self.n)
 def tick(self):
  from rosgraph_msgs.msg import Clock
  from sensor_msgs.msg import LaserScan
  from nav_msgs.msg import Odometry
  from geometry_msgs.msg import TransformStamped,PoseWithCovarianceStamped
  if self.mode=='no_clock':return
  if not self.paused:self.sim+=.02
  c=Clock();c.clock.sec=int(self.sim);c.clock.nanosec=int((self.sim%1)*1e9);self.pubs['clock'].publish(c);self.globalclock.publish(c)
  scan=LaserScan();scan.header.stamp=c.clock;scan.header.frame_id='wrong' if self.wrong else 'base_scan';scan.angle_min=-1.;scan.angle_max=1.;scan.angle_increment=.1;scan.range_min=.1;scan.range_max=5.;scan.ranges=[1.2]*20;self.pubs['scan'].publish(scan)
  od=Odometry();od.header.stamp=c.clock;od.header.frame_id='odom';od.child_frame_id='base_footprint';od.pose.pose.orientation.w=1.;self.pubs['odom'].publish(od)
  am=PoseWithCovarianceStamped();am.header.stamp=c.clock;am.header.frame_id='map';am.pose.pose.orientation.w=1.;self.pubs['amcl_pose'].publish(am)
  trans=[]
  for parent,child in [('odom','base_footprint'),('map','odom')]:
   tr=TransformStamped();tr.header.stamp=c.clock;tr.header.frame_id=parent;tr.child_frame_id=child;tr.transform.rotation.w=1.;trans.append(tr)
  if self.mode!='no_tf':self.tf.sendTransform(trans)
 def event(self,run_id,kind):
  from std_msgs.msg import String
  m=String();m.data=json.dumps({'run_id':run_id,'type':kind});self.pubs['cmr_experiment/events'].publish(m)
 def close(self):
  end=time.monotonic()+3.5
  while self.pending and time.monotonic()<end:time.sleep(.02)
  assert self.pending==0,'fake callback still active'
  time.sleep(.1)
  self.ex.remove_node(self.n);self.server.destroy();self.n.destroy_node()
@pytest.fixture
def session(ros,tmp_path):
 active=[]
 def create(mode='success'):
  rclpy,ex=ros;ns='test_'+uuid.uuid4().hex[:8];fake=Fake(rclpy,ex,ns,mode);run=tmp_path/('fixture_'+ns)
  for name in ['logs','runtime','artifacts']:(run/name).mkdir(parents=True)
  root=Path(__file__).resolve().parents[1];p=json.loads((root/'experiments/runner_profile.yaml').read_text());p.update({'selected_method':'mppi','synthetic_test':True,'test_namespace':ns,'fault':None});write_json(run/'artifacts/profile.json',p);(run/'artifacts/goal.json').write_bytes(Path(p['files']['goal']['path']).read_bytes())
  s=ProcessSupervisor(run);env=os.environ.copy();proc=s.spawn('worker',[sys.executable,'-B',str(root/'tools/nav_goal.py'),'--run-dir',str(run),'--test-namespace',ns],env,root,ipc=True);link=WorkerLink(proc,run.name,lambda *x:None,s);active.append((s,proc,link,fake));link.rpc('sensors',{},DeadlineBudget.after(3));return fake,run,link,s
 yield create
 for s,proc,link,fake in reversed(active):
  try:proc.critical=False;link.rpc('exit',{},DeadlineBudget.after(2))
  except Exception:pass
  s.close(proc,DeadlineBudget.after(3),initial_signal=False,grace=.5);fake.close()

def until(link,predicate,seconds=3):
 b=DeadlineBudget.after(seconds)
 while not predicate():b.require();link.pump()
@pytest.mark.parametrize('mode,terminal',[('success','SUCCEEDED'),('reject','REJECTED')])
def test_action_acceptance_and_uuid(session,mode,terminal):
 fake,run,link,s=session(mode);uid=uuid.uuid4().hex;link.rpc('send',{'uuid':uid},DeadlineBudget.after(3));until(link,lambda:any(e['type']=='ACTION_TERMINAL' for e in link.events));t=[e for e in link.events if e['type']=='ACTION_TERMINAL'][-1]['data']['data'];assert t['uuid']==uid and t['action']==terminal and fake.count==1

def test_duplicate_request_and_second_goal(session):
 fake,run,link,s=session();uid=uuid.uuid4().hex;deadline=time.monotonic()+4;link.send('send',{'uuid':uid},deadline,rid='same');until(link,lambda:'same' in link.results);first=link.results.pop('same');link.send('send',{'uuid':uid},deadline,rid='same');until(link,lambda:'same' in link.results);assert link.results.pop('same')==first and fake.count==1
 with pytest.raises(RuntimeError,match='second goal'):link.rpc('send',{'uuid':uuid.uuid4().hex},DeadlineBudget.after(1))
@pytest.mark.parametrize('mode',['cancel','no_cancel','accept_late'])
def test_cancel_and_acceptance_loss(session,mode):
 fake,run,link,s=session(mode);uid=uuid.uuid4().hex
 
 if mode=='accept_late':
  rid=link.send('send',{'uuid':uid},time.monotonic()+.15);until(link,lambda:rid in link.results);reply=link.results.pop(rid)['value']
 else:reply=link.rpc('send',{'uuid':uid},DeadlineBudget.after(2))
 if mode=='accept_late':assert reply['acceptance']=='UNKNOWN';until(link,lambda:fake.count==1);time.sleep(.4)
 if mode=='no_cancel':
  with pytest.raises((TimeoutError,RuntimeError)):link.rpc('cancel',{},DeadlineBudget.after(.15))
  assert not any(e['type']=='ACTION_TERMINAL' for e in link.events)
 else:
  link.rpc('cancel',{},DeadlineBudget.after(1));until(link,lambda:any(e['type']=='ACTION_TERMINAL' for e in link.events));assert [e for e in link.events if e['type']=='ACTION_TERMINAL'][-1]['data']['data']['action']=='CANCELED'
 assert fake.count==1
@pytest.mark.parametrize('mode',['manager_false','manager_late','manager_never'])
def test_manager_false_late_timeout(session,mode):
 fake,run,link,s=session(mode)
 with pytest.raises((RuntimeError,TimeoutError)):link.rpc('manager',{'phase':'localization','command':'STARTUP'},DeadlineBudget.after(.2 if mode!='manager_false' else 2))
 assert fake.count==0

def test_sensor_pause_and_clock_jump(session):
 fake,run,link,s=session();link.rpc('sensors',{},DeadlineBudget.after(3));fake.paused=True;time.sleep(1.7)
 with pytest.raises((RuntimeError,TimeoutError)):link.rpc('sensors',{},DeadlineBudget.after(.25))
 fake.paused=False;fake.sim=0.
 until(link,lambda:any(e['type']=='CLOCK_JUMP' for e in link.events));assert fake.count==0

def test_worker_bad_protocol_eof_still_owned(session):
 fake,run,link,s=session();until(link,lambda:any(e['type']=='HELLO' for e in link.events));os.write(link.item.process.stdin.fileno(),b'{bad-json}\n')
 with pytest.raises((RuntimeError,TimeoutError)):until(link,lambda:False,1)
 assert fake.count==0

@pytest.mark.parametrize('wrong_frame,missing_static',[(False,False),(True,False),(False,True)])
def test_real_mcap_late_static_and_content(ros,tmp_path,wrong_frame,missing_static):
 from inspect_bag import inspect
 rclpy,ex=ros;ns='bag_'+uuid.uuid4().hex[:8];fake=Fake(rclpy,ex,ns,wrong_frame=wrong_frame);run=tmp_path/('fixture_'+ns)
 for n in ['logs','runtime','artifacts']:(run/n).mkdir(parents=True)
 root=Path(__file__).resolve().parents[1];profile=json.loads((root/'experiments/runner_profile.yaml').read_text());profile.update({'synthetic_test':True,'test_namespace':ns,'fault':'withhold-initial-pose'});write_json(run/'artifacts/profile.json',profile)
 qos=json.loads((root/'experiments/recording_qos.yaml').read_text());write_json(run/'artifacts/qos.yaml',{'/'+ns+t:v for t,v in qos.items()})
 supervisor=ProcessSupervisor(run);topics=['/clock','/tf','/scan','/odom','/cmr_experiment/events']+([] if missing_static else ['/tf_static'])
 proc=supervisor.spawn('recorder',['ros2','bag','record','-s','mcap','-o',str(run/'bag'),'--use-sim-time','--node-name','fixture_rec_'+ns,'--qos-profile-overrides-path',str(run/'artifacts/qos.yaml'),'--topics',*['/'+ns+t for t in topics]],os.environ.copy(),root)
 try:
  deadline=time.monotonic()+5
  while time.monotonic()<deadline:
   if fake.n.count_subscribers('/'+ns+'/cmr_experiment/events')>=1:break
   time.sleep(.03)
  else:raise AssertionError('recorder not discovered')
  time.sleep(.4)
  for kind in ['RECORDING_START','FAULT_ACTIVATED','RECORDING_END']:fake.event(run.name,kind);time.sleep(.1)
  time.sleep(.2);r=supervisor.close(proc,DeadlineBudget.after(5),grace=4);assert r['ok']
  report=inspect(run);assert report['valid']==(not wrong_frame and not missing_static),report
  if not wrong_frame and not missing_static:
   assert report['topic_counts']['/cmr_experiment/events']==3 and report['topic_counts']['/scan']>=5 and ['base_link','base_scan'] in report['fixed_edges']
   shard=next((run/'bag').glob('*.mcap'));shard.write_bytes(b'broken')
   broken=inspect(run);assert not broken['valid'] and broken['errors']
 finally:
  if not proc.closed:supervisor.close(proc,DeadlineBudget.after(3),grace=.2)
  fake.close()


@pytest.mark.parametrize('_repeat',[0,1,2])
def test_installed_nav2_localization_manager_single_startup(ros,tmp_path,_repeat):
 """Real installed lifecycle components, isolated72; no simulator/initialpose/goal."""
 rclpy,ex=ros;root=Path(__file__).resolve().parents[1];run=tmp_path/'fixture_real_localization'
 for n in ['logs','runtime','artifacts']:(run/n).mkdir(parents=True)
 p=json.loads((root/'experiments/runner_profile.yaml').read_text());p.update({'selected_method':'mppi','synthetic_test':True,'fault':None});write_json(run/'artifacts/profile.json',p)
 sup=ProcessSupervisor(run);env=os.environ.copy();worker=sup.spawn('worker',[sys.executable,'-B',str(root/'tools/nav_goal.py'),'--run-dir',str(run)],env,root,ipc=True);link=WorkerLink(worker,run.name,lambda *a:None,sup);localization=None
 try:
  link.rpc('graph',{},DeadlineBudget.after(3))
  localization=sup.spawn('localization',['ros2','launch','--noninteractive','nav2_bringup','localization_launch.py','autostart:=false','use_sim_time:=false','use_composition:=False','use_respawn:=False','params_file:='+str(root/'src/cmr_bringup/config/nav2_mppi.yaml'),'map:=/opt/ros/jazzy/share/nav2_bringup/maps/tb3_sandbox.yaml'],env,root)
  reply=link.rpc('manager',{'phase':'localization','command':'STARTUP'},DeadlineBudget.after(15));assert reply['success'] and all(x['id']==3 for x in reply['states'].values())
  assert any(e['type']=='MANAGER_SERVICE_GATE_READY' for e in link.events)
  assert not (run/'runtime/initialpose-request.json').exists() and not (run/'runtime/goal-dispatch.json').exists()
  link.rpc('manager',{'phase':'localization','command':'SHUTDOWN'},DeadlineBudget.after(5));assert sup.close(localization,DeadlineBudget.after(5),grace=3)['ok']
 finally:
  if localization and not localization.closed:sup.close(localization,DeadlineBudget.after(5),grace=.5)
  worker.critical=False
  try:link.rpc('exit',{},DeadlineBudget.after(2))
  except Exception:pass
  sup.close(worker,DeadlineBudget.after(3),initial_signal=False,grace=.5)


def test_readiness_query_lost_once_does_not_repeat_startup(session):
 fake,run,link,s=session('gate_lost_once');reply=link.rpc('manager',{'phase':'localization','command':'STARTUP'},DeadlineBudget.after(4))
 assert reply['success'] and fake.gate_calls>=2 and fake.startups==1 and fake.count==0
 assert any(e['type']=='READINESS_QUERY_TIMEOUT' for e in link.events)


def test_no_tf_prevents_any_dispatch(session,tmp_path):
 with pytest.raises((TimeoutError,RuntimeError)):session('no_tf')
 files=list(tmp_path.rglob('action-client.jsonl'));assert files and all(not p.read_text() for p in files)



def test_action_aborted_is_preserved(session):
 fake,run,link,s=session('aborted');uid=uuid.uuid4().hex
 reply=link.rpc('send',{'uuid':uid},DeadlineBudget.after(3));assert reply['acceptance']=='ACCEPTED'
 until(link,lambda:any(e['type']=='ACTION_TERMINAL' for e in link.events))
 terminal=[e for e in link.events if e['type']=='ACTION_TERMINAL'][-1]['data']['data']
 assert terminal['uuid']==uid and terminal['action']=='ABORTED' and fake.count==1

def test_absent_clock_prevents_any_dispatch(session,tmp_path):
 with pytest.raises((TimeoutError,RuntimeError)):session('no_clock')
 files=list(tmp_path.rglob('action-client.jsonl'));assert files and all(not p.read_text() for p in files)

def test_runtime_plugin_mismatch_prevents_goal(session):
 from lifecycle_msgs.srv import GetState
 from rcl_interfaces.srv import ListParameters,GetParameters
 from rcl_interfaces.msg import ParameterValue,ParameterType
 fake,run,link,s=session()
 names=['map_server','amcl','controller_server','smoother_server','planner_server','route_server','behavior_server','velocity_smoother','collision_monitor','bt_navigator','waypoint_follower','docking_server','local_costmap/local_costmap','global_costmap/global_costmap']
 def state(req,res):res.current_state.id=3;res.current_state.label='active';return res
 def listing(name):
  def callback(req,res):res.result.names=['FollowPath.plugin'] if name=='controller_server' else [];return res
  return callback
 def values(req,res):
  res.values=[ParameterValue(type=ParameterType.PARAMETER_STRING,string_value='invalid::WrongController') for key in req.names];return res
 for name in names:
  prefix='/'+fake.ns+'/'+name
  if name not in ['map_server','amcl']:fake.n.create_service(GetState,prefix+'/get_state',state,callback_group=fake.group)
  fake.n.create_service(ListParameters,prefix+'/list_parameters',listing(name),callback_group=fake.group)
  fake.n.create_service(GetParameters,prefix+'/get_parameters',values,callback_group=fake.group)
 with pytest.raises(RuntimeError,match='plugin mismatch'):link.rpc('nav_ready',{},DeadlineBudget.after(8))
 assert fake.count==0 and not (run/'runtime/goal-dispatch.json').exists()
 actual=json.loads((run/'runtime/all-parameters.json').read_text());assert actual['controller_server']['FollowPath.plugin']=='invalid::WrongController'
