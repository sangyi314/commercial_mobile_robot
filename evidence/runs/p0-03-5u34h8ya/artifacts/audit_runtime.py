"""Read-only runtime identity and clearance evidence before the single goal."""
from pathlib import Path
import os,json,time,math,gzip,yaml
import rclpy
from rclpy.parameter import Parameter,parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import GetCostmap
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer,TransformListener,TransformException
from rosidl_runtime_py.convert import message_to_yaml
run=Path(os.environ['CMR_RUN']);rclpy.init();n=rclpy.create_node('p03_runtime_audit',parameter_overrides=[Parameter('use_sim_time',value=True)])
buf=Buffer(node=n);listener=TransformListener(buf,n)
def call(typ,service,request,timeout=8):
 c=n.create_client(typ,service)
 if not c.wait_for_service(timeout_sec=timeout):raise RuntimeError('No service '+service)
 f=c.call_async(request);end=time.monotonic()+timeout
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 if not f.done():raise RuntimeError('Timeout '+service)
 result=f.result();n.destroy_client(c);return result
nodes=['map_server','amcl','controller_server','smoother_server','planner_server','route_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother','collision_monitor','docking_server']
states={}
for node in nodes:
 r=call(GetState,'/'+node+'/get_state',GetState.Request());states[node]={'id':r.current_state.id,'label':r.current_state.label}
requested={
'controller_server':['use_sim_time','controller_plugins','FollowPath.plugin','controller_frequency','general_goal_checker.plugin','general_goal_checker.xy_goal_tolerance','general_goal_checker.yaw_goal_tolerance','general_goal_checker.stateful','enable_stamped_cmd_vel'],
'velocity_smoother':['use_sim_time','max_velocity','min_velocity','max_accel','max_decel','enable_stamped_cmd_vel'],
'collision_monitor':['use_sim_time','enable_stamped_cmd_vel','polygons','observation_sources'],
'local_costmap/local_costmap':['use_sim_time','robot_radius','footprint','footprint_padding','global_frame','robot_base_frame'],
'global_costmap/global_costmap':['use_sim_time','robot_radius','footprint','footprint_padding','global_frame','robot_base_frame'],
'amcl':['use_sim_time','base_frame_id','global_frame_id','odom_frame_id','scan_topic'],
'bt_navigator':['use_sim_time','global_frame','robot_base_frame']}
params={}
for node,names in requested.items():
 req=GetParameters.Request();req.names=names
 r=call(GetParameters,'/'+node+'/get_parameters',req)
 params[node]={key:parameter_value_to_python(v) for key,v in zip(names,r.values)}
endpoints={}
types=dict(n.get_topic_names_and_types())
for topic in ['/clock','/scan','/odom','/initialpose','/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel','/navigate_to_pose/_action/status']:
 endpoints[topic]={'types':types.get(topic,[])}
 for side,method in [('publishers',n.get_publishers_info_by_topic),('subscriptions',n.get_subscriptions_info_by_topic)]:
  endpoints[topic][side]=[{'node_name':v.node_name,'node_namespace':v.node_namespace,'topic_type':v.topic_type,'qos':str(v.qos_profile)} for v in method(topic)]
costmaps={}
for kind in ['global','local']:
 msg=call(GetCostmap,f'/{kind}_costmap/get_costmap',GetCostmap.Request()).map
 with gzip.open(run/'runtime'/f'{kind}-costmap.yaml.gz','xt') as f:f.write(message_to_yaml(msg))
 costmaps[kind]=msg
scans=[];polys=[]
s=n.create_subscription(LaserScan,'/scan',lambda m:scans.append(m),qos_profile_sensor_data)
p=n.create_subscription(PolygonStamped,'/local_costmap/published_footprint',lambda m:polys.append(m),10)
end=time.monotonic()+3
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
t=buf.lookup_transform('map','base_footprint',Time())
v=t.transform.translation;q=t.transform.rotation
yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
pose={'x':v.x,'y':v.y,'yaw':yaw,'stamp_s':t.header.stamp.sec+t.header.stamp.nanosec*1e-9,'receive_ros_s':n.get_clock().now().nanoseconds*1e-9,'quaternion_xyzw':[q.x,q.y,q.z,q.w]}
if not scans or not polys:raise RuntimeError('Missing current scan or footprint')
(run/'runtime/pre-goal-scan.yaml').write_text(message_to_yaml(scans[-1]))
(run/'runtime/runtime-footprint.yaml').write_text(message_to_yaml(polys[-1]))
# Evaluate conservative swept disks in both actual costmaps; unknown and inflated costs >=200 rejected.
radius=params['local_costmap/local_costmap']['robot_radius']+params['local_costmap/local_costmap']['footprint_padding']
def path_cost(msg,gx,gy):
 md=msg.metadata;res=md.resolution;ox=md.origin.position.x;oy=md.origin.position.y
 # Costmap origins must be planar/unrotated for this grid calculation.
 oq=md.origin.orientation
 if abs(oq.x)+abs(oq.y)+abs(oq.z)>1e-6 or abs(abs(oq.w)-1)>1e-6:raise RuntimeError('Rotated costmap unsupported')
 worst=0
 for i in range(46):
  f=i/45;x=pose['x']+(gx-pose['x'])*f;y=pose['y']+(gy-pose['y'])*f
  for iy in range(math.floor((y-radius-oy)/res),math.ceil((y+radius-oy)/res)+1):
   for ix in range(math.floor((x-radius-ox)/res),math.ceil((x+radius-ox)/res)+1):
    cx=ox+(ix+.5)*res;cy=oy+(iy+.5)*res
    if math.hypot(cx-x,cy-y)>radius+res*.71:continue
    if not(0<=ix<md.size_x and 0<=iy<md.size_y):return 255
    worst=max(worst,int(msg.data[iy*md.size_x+ix]))
 return worst
candidates=[]
for offset in [0.,math.pi/2,-math.pi/2,math.pi]:
 theta=yaw+offset;gx=v.x+.9*math.cos(theta);gy=v.y+.9*math.sin(theta)
 costs={k:path_cost(m,gx,gy) for k,m in costmaps.items()}
 candidates.append({'x':gx,'y':gy,'yaw':theta,'distance_m':.9,'max_cost_by_map':costs,'clear':max(costs.values())<200})
# Local laser endpoint alignment with occupied static map cells, nearest wall within .30 m.
scan=scans[-1];st=buf.lookup_transform('map',scan.header.frame_id,Time())
sv=st.transform.translation;sq=st.transform.rotation
syaw=math.atan2(2*(sq.w*sq.z+sq.x*sq.y),1-2*(sq.y*sq.y+sq.z*sq.z))
with gzip.open(run/'runtime/initialize-map.yaml.gz','rt') as f:static=yaml.safe_load(f)
md=static['info'];res=md['resolution'];ox=md['origin']['position']['x'];oy=md['origin']['position']['y'];width=md['width'];height=md['height'];distances=[]
for i,r in enumerate(scan.ranges):
 if not math.isfinite(r) or not scan.range_min<=r<=scan.range_max:continue
 angle=syaw+scan.angle_min+i*scan.angle_increment
 x=sv.x+r*math.cos(angle);y=sv.y+r*math.sin(angle);ix=int((x-ox)/res);iy=int((y-oy)/res)
 near=[]
 for yy in range(max(0,iy-6),min(height,iy+7)):
  for xx in range(max(0,ix-6),min(width,ix+7)):
   if static['data'][yy*width+xx]>=65:near.append(math.hypot(ox+(xx+.5)*res-x,oy+(yy+.5)*res-y))
 distances.append(min(near) if near else .5)
alignment={'valid_endpoints':len(distances),'within_0_15_m_fraction':sum(d<=.15 for d in distances)/len(distances),'median_nearest_occupied_m':sorted(distances)[len(distances)//2],'method':'latest map-scan TF; nearest occupied static-map cell within 0.30m search; local numeric consistency only, not global alignment/independent truth'}
report={'lifecycle':states,'parameters':params,'endpoints':endpoints,'pose':pose,'radius_including_padding_m':radius,'candidates':candidates,'scan_map_local_alignment':alignment,'costmap_stamps':{k:m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for k,m in costmaps.items()}}
with (run/'runtime/pre-goal-audit.json').open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
print(json.dumps({k:report[k] for k in ['lifecycle','pose','radius_including_padding_m','candidates','scan_map_local_alignment','costmap_stamps']},allow_nan=False))
assert all(s['id']==3 for s in states.values())
assert params['controller_server']['FollowPath.plugin']=='nav2_mppi_controller::MPPIController'
assert any(c['clear'] for c in candidates)
assert alignment['within_0_15_m_fraction']>=.8
assert abs(pose['receive_ros_s']-pose['stamp_s'])<2
n.destroy_node();rclpy.shutdown()
