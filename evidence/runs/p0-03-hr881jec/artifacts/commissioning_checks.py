"""One-run commissioning checks. Only --phase initialize publishes one AMCL input."""
from pathlib import Path
import argparse, os, json, math, time, statistics, gzip
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosidl_runtime_py.convert import message_to_yaml
from tf2_ros import Buffer, TransformListener, TransformException
p=argparse.ArgumentParser();p.add_argument('--phase',choices=['pre','initialize','post'],required=True);a=p.parse_args()
run=Path(os.environ['CMR_RUN']);rclpy.init()
n=rclpy.create_node('p03_commissioning_checks',parameter_overrides=[Parameter('use_sim_time',value=True)])
buf=Buffer(node=n);listener=TransformListener(buf,n);subs=[];clock=[];odom=[];scans=[];maps=[];tf=[];echo=[];poses=[]
def sec(t):return t.sec+t.nanosec*1e-9
def cbclock(m):clock.append({'receive_monotonic_s':time.monotonic(),'stamp_s':sec(m.clock)})
def cbodom(m):odom.append({'receive_monotonic_s':time.monotonic(),'stamp_s':sec(m.header.stamp),'frame_id':m.header.frame_id,'child_frame_id':m.child_frame_id,'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,'vx':m.twist.twist.linear.x,'wz':m.twist.twist.angular.z})
qmap=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
for typ,topic,cb,qos in [(Clock,'/clock',cbclock,qos_profile_sensor_data),(Odometry,'/odom',cbodom,qos_profile_sensor_data),(LaserScan,'/scan',lambda m:scans.append(m),qos_profile_sensor_data),(OccupancyGrid,'/map',lambda m:maps.append(m),qmap),(PoseWithCovarianceStamped,'/initialpose',lambda m:echo.append(m),10),(PoseWithCovarianceStamped,'/amcl_pose',lambda m:poses.append(m),qmap)]:
 subs.append(n.create_subscription(typ,topic,cb,qos))
pub=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10) if a.phase=='initialize' else None
start=time.monotonic();sent=False;last_tf=0
while time.monotonic()-start<8:
 rclpy.spin_once(n,timeout_sec=.05)
 if pub and not sent and clock and maps and pub.get_subscription_count()>=2:
  m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg()
  m.pose.pose.position.x=-2.;m.pose.pose.position.y=-.5;m.pose.pose.orientation.w=1.
  m.pose.covariance[0]=.25;m.pose.covariance[7]=.25;m.pose.covariance[35]=.06853891945200942
  with (run/'runtime/initialpose-request.yaml').open('x') as f:f.write(message_to_yaml(m))
  (run/'runtime/initialpose-dispatch.json').write_text(json.dumps({'monotonic_s':time.monotonic(),'stamp_s':sec(m.header.stamp),'kind':'actual AMCL estimate input, not independent spawn truth'})+'\n')
  pub.publish(m);sent=True
 if time.monotonic()-last_tf>.2:
  last_tf=time.monotonic()
  for parent,child in [('odom','base_footprint'),('base_link','base_scan'),('map','odom'),('map','base_footprint')]:
   try:
    t=buf.lookup_transform(parent,child,Time());v=t.transform.translation;q=t.transform.rotation
    tf.append({'receive_monotonic_s':time.monotonic(),'parent':parent,'child':child,'stamp_s':sec(t.header.stamp),'position':[v.x,v.y,v.z],'quaternion':[q.x,q.y,q.z,q.w]})
   except TransformException:pass
def dump(name,v):
 with (run/'runtime'/name).open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
summary={'phase':a.phase,'clock_count':len(clock),'clock_first':clock[0] if clock else None,'clock_last':clock[-1] if clock else None,'clock_backjumps':sum(b['stamp_s']<c['stamp_s'] for c,b in zip(clock,clock[1:])),'odom_count':len(odom),'scan_count':len(scans),'map_count':len(maps),'initialpose_sent':sent,'initialpose_echo_count':len(echo),'amcl_pose_count':len(poses)}
if scans:
 s=scans[-1];valid=[x for x in s.ranges if math.isfinite(x) and s.range_min<=x<=s.range_max]
 summary['scan']={'stamp_s':sec(s.header.stamp),'frame_id':s.header.frame_id,'rays':len(s.ranges),'valid':len(valid),'nonfinite':sum(not math.isfinite(x) for x in s.ranges),'min_m':min(valid) if valid else None,'median_m':statistics.median(valid) if valid else None,'max_m':max(valid) if valid else None,'range_min':s.range_min,'range_max':s.range_max}
 (run/'runtime'/f'{a.phase}-scan.yaml').write_text(message_to_yaml(s))
if maps:
 m=maps[-1];summary['map']={'frame_id':m.header.frame_id,'width':m.info.width,'height':m.info.height,'resolution':m.info.resolution}
 with gzip.open(run/'runtime'/f'{a.phase}-map.yaml.gz','xt') as f:f.write(message_to_yaml(m))
if poses:(run/'runtime'/f'{a.phase}-amcl-pose.yaml').write_text(message_to_yaml(poses[-1]))
if echo:
 with (run/'runtime'/f'{a.phase}-initialpose-echo.yaml').open('x') as f:f.write(message_to_yaml(echo[-1]))
dump(f'{a.phase}-samples.json',{'clock':clock,'odom':odom,'tf':tf})
summary['tf_pairs']=sorted({(t['parent'],t['child']) for t in tf});dump(f'{a.phase}-summary.json',summary);print(json.dumps(summary,allow_nan=False))
n.destroy_node();rclpy.shutdown()
if not clock or clock[-1]['stamp_s']<=clock[0]['stamp_s'] or not odom or not scans:raise SystemExit(2)
if a.phase=='initialize' and (not sent or not echo):raise SystemExit(3)
