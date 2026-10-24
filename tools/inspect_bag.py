"""Read closed MCAP messages without replaying anything into ROS."""
import argparse,json,math,sys
from pathlib import Path
import yaml
from runner_core import digest,within,write_json,strict_json
from runner_analysis import analyze,yaw

def inspect(run):
 import rosbag2_py
 from rclpy.serialization import deserialize_message
 from rosidl_runtime_py.utilities import get_message
 run=Path(run).resolve();errors=[];counts={};types={};edges=set();dynamic={};rows=[];events=[];files=[];ranges={};scan_valid=0;odom_frames=set();scan_frames=set();lastclock=None
 manifest=strict_json((run/'manifest.json').read_text()) if (run/'manifest.json').exists() else None
 profile=strict_json((run/'artifacts/profile.json').read_text());fault=profile.get('fault');testprefix=('/'+profile.get('test_namespace','').strip('/')) if profile.get('synthetic_test') else ''
 metadata=run/'bag/metadata.yaml'
 try:
  md=yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
  paths=md['relative_file_paths']
  if not paths:raise ValueError('no shards')
  for name in paths:
   path=within(run/'bag',run/'bag'/name)
   if not path.is_file() or path.stat().st_size==0:raise ValueError('missing/empty shard '+name)
   files.append({'path':str(path.relative_to(run)),'size_bytes':path.stat().st_size,'sha256':digest(path)})
  reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(run/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
  rawtypes={x.name:x.type for x in reader.get_all_topics_and_types()};types={(t[len(testprefix):] if testprefix and t.startswith(testprefix+'/') else t):typ for t,typ in rawtypes.items()};message_types={t:get_message(typ) for t,typ in rawtypes.items()}
  def stamp(t):return t.sec+t.nanosec*1e-9
  def row(kind,data,ros):return {'kind':kind,'data':data,'ros_now_s':ros,'receive_monotonic_s':None,'line':None}
  while reader.has_next():
   topic,data,bag_ns=reader.read_next();msg=deserialize_message(data,message_types[topic]);topic=topic[len(testprefix):] if testprefix and topic.startswith(testprefix+'/') else topic;ros=bag_ns*1e-9;counts[topic]=counts.get(topic,0)+1;ranges.setdefault(topic,{'first_bag_ros_s':ros,'last_bag_ros_s':ros});ranges[topic]['last_bag_ros_s']=ros
   if topic=='/clock':
    current=stamp(msg.clock)
    if lastclock is not None and current<lastclock:errors.append('clock backjump')
    lastclock=current;rows.append(row('clock',{'sim_s':current},ros))
   elif topic=='/odom':
    p=msg.pose.pose.position;q=msg.pose.pose.orientation;v=msg.twist.twist;odom_frames.add((msg.header.frame_id,msg.child_frame_id));rows.append(row('odom',{'stamp_s':stamp(msg.header.stamp),'position':[p.x,p.y,p.z],'quaternion_xyzw':[q.x,q.y,q.z,q.w],'linear':[v.linear.x,v.linear.y,v.linear.z],'angular':[v.angular.x,v.angular.y,v.angular.z]},ros))
   elif topic=='/map':
    if msg.header.frame_id!='map' or msg.info.width<=0 or msg.info.height<=0 or msg.info.resolution<=0:errors.append('map frame/content')
   elif topic=='/initialpose':
    if msg.header.frame_id!='map' or abs(stamp(msg.header.stamp)-ros)>.5:errors.append('initialpose frame/stamp')
   elif topic=='/scan':
    scan_frames.add(msg.header.frame_id)
    if any(math.isfinite(x) and msg.range_min<=x<=msg.range_max for x in msg.ranges):scan_valid+=1
    if not math.isfinite(stamp(msg.header.stamp)) or (lastclock is not None and abs(stamp(msg.header.stamp)-lastclock)>.5):errors.append('stale scan stamp')
   elif topic in ['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel']:
    if types[topic]!='geometry_msgs/msg/Twist':errors.append('unsupported velocity type')
    rows.append(row(topic,{'linear':[msg.linear.x,msg.linear.y,msg.linear.z],'angular':[msg.angular.x,msg.angular.y,msg.angular.z]},ros))
   elif topic in ['/tf','/tf_static']:
    for tr in msg.transforms:
     edge=(tr.header.frame_id,tr.child_frame_id)
     if topic=='/tf_static':edges.add(edge)
     else:dynamic.setdefault(edge,[]).append((stamp(tr.header.stamp),tr,ros))
   elif topic=='/navigate_to_pose/_action/status':rows.append(row('action_status',[{'goal_uuid':bytes(x.goal_info.goal_id.uuid).hex(),'status':x.status} for x in msg.status_list],ros))
   elif topic=='/cmr_experiment/events':
    event=strict_json(msg.data)
    if event.get('run_id')!=run.name:errors.append('wrong event run_id')
    events.append(event)
  for t in ['/clock','/tf','/tf_static','/scan','/odom','/cmr_experiment/events']:
   if counts.get(t,0)==0:errors.append('required topic empty '+t)
  for edge in [('base_footprint','base_link'),('base_link','base_scan')]:
   if edge not in edges:errors.append('missing fixed edge '+str(edge))
  if scan_frames!={'base_scan'} or scan_valid<2:errors.append('scan frame/content')
  if not odom_frames or any(frame!='odom' or child not in ['base_footprint','base_link'] for frame,child in odom_frames):errors.append('odom frame/content')
  for t in ['/clock','/odom','/scan']:
   if t in ranges and ranges[t]['last_bag_ros_s']<=ranges[t]['first_bag_ros_s']:errors.append('no advancing data '+t)
  eventtypes={e['type'] for e in events}
  if not {'RECORDING_START','RECORDING_END'}<=eventtypes:errors.append('missing recording interval markers')
  behavior=None;actions=[]
  sidecar=run/'logs/action-client.jsonl'
  if sidecar.exists():actions=[strict_json(l) for l in sidecar.read_text().splitlines()]
  if profile.get('mode')=='probe':
   coverage='probe-no-navigation'
   if any(x['kind']=='dispatch' for x in actions) or any(r['data'] for r in rows if r['kind']=='action_status'):errors.append('probe goal forbidden')
   if counts.get('/initialpose',0)!=1 or 'PROBE_ARMED' not in eventtypes:errors.append('probe preparation/arm absent')
  elif fault:
   if 'FAULT_ACTIVATED' not in eventtypes:errors.append('fault stage not reached')
   if counts.get('/initialpose',0)>0 or any(x['kind']=='dispatch' for x in actions):errors.append('fault emitted pose/goal')
   coverage='partial-localization-fault'
  elif any(x['kind']=='terminal' and x['data']['action']=='SUCCEEDED' for x in actions):
   coverage='full-navigation'
   for t in profile['recording']['topics']:
    if not counts.get(t):errors.append('normal required topic empty '+t)
   if counts.get('/initialpose',0)!=1:errors.append('initialpose count is not one')
   if ('map','odom') not in dynamic or ('odom','base_footprint') not in dynamic:errors.append('dynamic TF chain absent')
   else:
    # Compose recorded map->odom with recorded odometry TF on the simulation
    # time axis. No bag receive time is labeled as a monotonic wall time.
    maps=dynamic[('map','odom')];odoms=dynamic[('odom','base_footprint')]
    for ost,ot,received in odoms:
     candidates=[x for x in maps if abs(x[0]-ost)<=1.5 and x[0]<=ost+1.0]
     if not candidates:continue
     mt=min(candidates,key=lambda x:abs(x[0]-ost))[1];a=mt.transform.translation;b=ot.transform.translation;ma=yaw([getattr(mt.transform.rotation,k) for k in ['x','y','z','w']]);oa=yaw([getattr(ot.transform.rotation,k) for k in ['x','y','z','w']]);angle=ma+oa
     rows.append(row('map_base',{'stamp_s':ost,'position':[a.x+math.cos(ma)*b.x-math.sin(ma)*b.y,a.y+math.sin(ma)*b.x+math.cos(ma)*b.y,0.0],'quaternion_xyzw':[0.,0.,math.sin(angle/2),math.cos(angle/2)]},received))
    rows.sort(key=lambda x:x['ros_now_s'])
    try:
     behavior=analyze(rows,actions,strict_json((run/'artifacts/goal.json').read_text()),strict_json((run/'runtime/all-parameters.json').read_text())['controller_server'],axis='ros_sim')
     if not all(behavior['checks'].values()):errors.append('bag behavior checks '+str(behavior['checks']))
    except Exception as exc:errors.append('bag behavior '+str(exc))
   terminal=[x for x in actions if x['kind']=='terminal'][-1]
   if not any(e['type']=='ACTION_TERMINAL' and e['data']['data']['uuid']==terminal['data']['uuid'] for e in events):errors.append('bag terminal event missing')
  else:coverage='partial-startup';behavior=None
  for topic in types:
   listed=next((x['message_count'] for x in md['topics_with_message_count'] if x['topic_metadata']['name']==testprefix+topic),None)
   if listed is not None and listed!=counts.get(topic,0):errors.append('metadata count differs '+topic)
 except Exception as exc:errors.append(type(exc).__name__+': '+str(exc));coverage='invalid';behavior=None
 return {'valid':not errors,'errors':errors,'coverage':coverage,'record_origin':'fixture' if profile.get('synthetic_test') else 'real','topic_counts':counts,'types':types,'ranges':ranges,'fixed_edges':sorted([list(e) for e in edges]),'dynamic_edges':[list(e) for e in dynamic],'events':sorted({e['type'] for e in events}),'files':[{'path':'bag/metadata.yaml','sha256':digest(metadata)}] + files if metadata.is_file() else files,'behavior':behavior,'time_axis':'bag receive timestamps and message stamps are ROS simulation time; source and parent monotonic times remain sidecars','collision_evaluation':'not performed; bag coverage is unrelated to collision coverage'}
def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--run-dir',required=True);ap.add_argument('--output',required=True);a=ap.parse_args()
 report=inspect(a.run_dir);write_json(Path(a.output),report);print(json.dumps({'valid':report['valid'],'coverage':report['coverage'],'errors':report['errors']}));return 0 if report['valid'] else 3
if __name__=='__main__':sys.exit(main())
