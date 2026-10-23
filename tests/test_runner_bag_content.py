"""Synthetic MCAP content tests, never published and never counted as real runs."""
import json,sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from runner_core import write_json

def make_bag(run,case):
 import rosbag2_py
 from rclpy.serialization import serialize_message
 from rosgraph_msgs.msg import Clock
 from nav_msgs.msg import Odometry,OccupancyGrid
 from sensor_msgs.msg import LaserScan
 from tf2_msgs.msg import TFMessage
 from geometry_msgs.msg import Twist,TransformStamped,PoseWithCovarianceStamped
 from std_msgs.msg import String
 from action_msgs.msg import GoalStatusArray,GoalStatus
 for name in ['artifacts','runtime','logs']:(run/name).mkdir(parents=True)
 uid='1'*32;types={'/clock':('rosgraph_msgs/msg/Clock',Clock),'/odom':('nav_msgs/msg/Odometry',Odometry),'/scan':('sensor_msgs/msg/LaserScan',LaserScan),'/tf':('tf2_msgs/msg/TFMessage',TFMessage),'/tf_static':('tf2_msgs/msg/TFMessage',TFMessage),'/map':('nav_msgs/msg/OccupancyGrid',OccupancyGrid),'/initialpose':('geometry_msgs/msg/PoseWithCovarianceStamped',PoseWithCovarianceStamped),'/cmr_experiment/events':('std_msgs/msg/String',String),'/navigate_to_pose/_action/status':('action_msgs/msg/GoalStatusArray',GoalStatusArray)}
 for t in ['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel']:types[t]=('geometry_msgs/msg/Twist',Twist)
 write_json(run/'artifacts/profile.json',{'synthetic_test':True,'fault':None,'recording':{'topics':list(types)}})
 goal={'pose':{'pose':{'position':{'x':1.,'y':0.},'orientation':{'x':0.,'y':0.,'z':0.,'w':1.}}}};write_json(run/'artifacts/goal.json',goal);write_json(run/'runtime/all-parameters.json',{'controller_server':{'general_goal_checker.xy_goal_tolerance':.25,'general_goal_checker.yaw_goal_tolerance':.25}})
 actions=[{'kind':'dispatch','monotonic_s':101.,'ros_s':1.,'data':{'uuid':uid}},{'kind':'response','monotonic_s':101.1,'ros_s':1.1,'data':{'uuid':uid,'accepted':True}},{'kind':'terminal','monotonic_s':108.,'ros_s':8.,'data':{'uuid':uid,'action':'SUCCEEDED'}}]
 (run/'logs/action-client.jsonl').write_text(''.join(json.dumps(a)+'\n' for a in actions))
 writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(run/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
 for i,(topic,(typ,_)) in enumerate(types.items()):writer.create_topic(rosbag2_py.TopicMetadata(id=i,name=topic,type=typ,serialization_format='cdr'))
 def put(topic,msg,t):writer.write(topic,serialize_message(msg),int(t*1e9))
 def trans(parent,child,stamp,x=0.):
  tr=TransformStamped();tr.header.frame_id=parent;tr.header.stamp=stamp;tr.child_frame_id=child;tr.transform.translation.x=x;tr.transform.rotation.w=1.;return tr
 for i in range(151):
  t=i*.1;c=Clock();c.clock.sec=int(t);c.clock.nanosec=round((t%1)*1e9)
  if c.clock.nanosec>=1000000000:c.clock.sec+=1;c.clock.nanosec=0
  put('/clock',c,t)
  x=min(1.,max(0.,(t-1)/6))
  od=Odometry();od.header.frame_id='odom';od.child_frame_id='base_footprint';od.header.stamp=c.clock;od.pose.pose.position.x=x;od.pose.pose.orientation.w=1.;od.twist.twist.linear.x=1/6 if 1<t<7 else 0.;put('/odom',od,t)
  scan=LaserScan();scan.header.frame_id='wrong' if case=='wrong_frame' else 'base_scan';scan.header.stamp=c.clock;scan.range_min=.1;scan.range_max=5.;scan.ranges=[1.2]*20;put('/scan',scan,t)
  tf=TFMessage();tf.transforms=[trans('map','odom',c.clock),trans('odom','base_footprint',c.clock,x)];put('/tf',tf,t)
  if i==1:
   if case!='missing_static':
    tf=TFMessage();tf.transforms=[trans('base_footprint','base_link',c.clock),trans('base_link','base_scan',c.clock)];put('/tf_static',tf,t)
   grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=2;grid.info.height=2;grid.info.resolution=.1;grid.data=[0]*4;put('/map',grid,t)
   initial=PoseWithCovarianceStamped();initial.header.frame_id='map';initial.header.stamp=c.clock;put('/initialpose',initial,t)
  if t<=9:
   cmd=Twist();cmd.linear.x=.1 if t<8 else 0.
   for topic in ['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel']:put(topic,cmd,t)
  if i in [2,80,149]:
   kind={2:'RECORDING_START',80:'ACTION_TERMINAL',149:'RECORDING_END'}[i]
   if case!='missing_terminal' or kind!='ACTION_TERMINAL':
    e=String();e.data=json.dumps({'type':kind,'run_id':run.name,'data':{'data':{'uuid':uid}}});put('/cmr_experiment/events',e,t)
  if i==80 and case!='missing_terminal':
   st=GoalStatusArray();one=GoalStatus();one.goal_info.goal_id.uuid=list(bytes.fromhex(uid));one.status=4;st.status_list=[one];put('/navigate_to_pose/_action/status',st,t)
 del writer

@pytest.mark.parametrize('case',['valid','missing_static','wrong_frame','missing_terminal','corrupt'])
def test_full_bag_exact_content_and_negative_cases(tmp_path,case):
 from inspect_bag import inspect
 run=tmp_path/'fixture';make_bag(run,case)
 if case=='corrupt':next((run/'bag').glob('*.mcap')).write_bytes(b'broken')
 report=inspect(run)
 assert report['valid']==(case=='valid'),report
 if case=='valid':assert report['topic_counts']['/odom']==151 and report['behavior']['checks']['stop'] and report['coverage']=='full-navigation'
 if case=='missing_static':assert any('fixed edge' in e for e in report['errors'])
 if case=='wrong_frame':assert any('scan frame' in e for e in report['errors'])
 if case=='missing_terminal':assert 'bag terminal event missing' in report['errors']


def test_missing_metadata_is_not_a_valid_recording(tmp_path):
 from inspect_bag import inspect
 run=tmp_path/'fixture_missing_metadata';(run/'bag').mkdir(parents=True);(run/'artifacts').mkdir()
 (run/'artifacts/profile.json').write_text(json.dumps({'synthetic_test':True,'fault':None}))
 report=inspect(run);assert not report['valid'] and report['errors']
