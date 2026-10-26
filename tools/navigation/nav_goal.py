"""Independent ROS worker. JSONL IPC only; never creates processes."""
if __package__ in (None, ''):
    from pathlib import Path
    from runpy import run_path
    run_path(str(Path(__file__).resolve().parents[1] / '__init__.py'))['bootstrap']()
import argparse,math,os,select,sys,time,uuid
from pathlib import Path
from tools.runner.runner_core import strict_json, encode, write_json, validate_envelope, PLUGINS
from tools.runner.runner_analysis import analyze
from tools.runner.failure_stop import observe_stop, shutdown_transition

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--run-dir',required=True);parser.add_argument('--test-namespace',default='');args=parser.parse_args()
 if args.test_namespace and (os.environ.get('ROS_DOMAIN_ID')!='72' or os.environ.get('CMR_TEST_ROS_DOMAIN_ID')!='72'):parser.error('test namespace requires explicit isolated domain72')
 run=Path(args.run_dir);profile=strict_json((run/'artifacts/profile.json').read_text());method=profile['selected_method']
 import rclpy
 from rclpy.parameter import Parameter,parameter_value_to_python
 from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,DurabilityPolicy
 from rclpy.action import ActionClient
 from rclpy.time import Time
 from rclpy.duration import Duration
 from rclpy.signals import SignalHandlerOptions
 from tf2_ros import Buffer,TransformListener
 from sensor_msgs.msg import LaserScan
 from nav_msgs.msg import Odometry,OccupancyGrid,Path as NavPath
 from rosgraph_msgs.msg import Clock
 from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
 from action_msgs.msg import GoalStatusArray
 from action_msgs.srv import CancelGoal
 from unique_identifier_msgs.msg import UUID
 from nav2_msgs.action import NavigateToPose
 from nav2_msgs.srv import ManageLifecycleNodes,GetCostmap
 from lifecycle_msgs.srv import GetState,ChangeState
 from rcl_interfaces.srv import ListParameters,GetParameters
 from rosbag2_interfaces.srv import IsPaused
 from std_srvs.srv import Trigger
 from std_msgs.msg import String
 from rosidl_runtime_py.convert import message_to_ordereddict,message_to_yaml
 import yaml
 ns=args.test_namespace.strip('/');prefix=('/'+ns) if ns else ''
 topic=lambda name:prefix+'/'+name.lstrip('/')
 rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
 node=rclpy.create_node('cmr_worker',namespace=prefix or '/',parameter_overrides=[Parameter('use_sim_time',value=True)],cli_args=['--ros-args','-r','/clock:='+topic('clock'),'-r','/tf:='+topic('tf'),'-r','/tf_static:='+topic('tf_static')] if ns else None)
 buffer=Buffer(node=node);listener=TransformListener(buffer,node)
 durable=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
 observations=(run/'runtime/observations.jsonl').open('x');actionfile=(run/'logs/action-client.jsonl').open('x')
 rows=[];actions=[];latest={};lastclock=None;lastadvance=time.monotonic();clockticks=0;clock_jump=False;seq=0;lastseq=0;lastheart=0.;lasttf=0.;outbuf=bytearray();inbuf=bytearray();cache={};initial_sent=False;navigation_ready=False
 goal_future=None;handle=None;result_future=None;response_recorded=False;terminal=None;goal_uuid=None;action_request='';late_services=[];stale_emitted=False;service_clients={};probe_origin=None;probe_pause_deadline=None;fault_deactivated=False
 eventpub=node.create_publisher(String,topic('cmr_experiment/events'),100)
 initialpub=node.create_publisher(PoseWithCovarianceStamped,topic('initialpose'),10)
 ac=ActionClient(node,NavigateToPose,topic('navigate_to_pose'))
 os.set_blocking(sys.stdin.fileno(),False);os.set_blocking(sys.stdout.fileno(),False)
 def rosnow():return node.get_clock().now().nanoseconds*1e-9
 def wire(kind,data,rid='',mirror=True):
  nonlocal seq,outbuf
  seq+=1;value={'protocol_version':1,'run_id':run.name,'seq':seq,'request_id':rid,'type':kind,'source_monotonic_s':time.monotonic(),'ros_s':rosnow() if lastclock is not None else None,'data':data}
  outbuf.extend(encode(value))
  if len(outbuf)>65536:raise RuntimeError('IPC output backpressure')
  if mirror and lastclock is not None:
   msg=String();msg.data=encode(value).decode().strip();eventpub.publish(msg)
  flush()
 def flush():
  if outbuf:
   try:count=os.write(sys.stdout.fileno(),outbuf);del outbuf[:count]
   except BlockingIOError:pass
 def obs(kind,data):
  value={'kind':kind,'receive_monotonic_s':time.monotonic(),'ros_now_s':rosnow(),'data':data,'line':len(rows)+1};rows.append(value);observations.write(encode(value).decode());observations.flush()
 def clean(v):return [float(x) if math.isfinite(float(x)) else None for x in v]
 def stamp(t):return t.sec+t.nanosec*1e-9
 def receive(key,msg):
  nonlocal lastclock,clockticks,clock_jump,lastadvance
  latest[key]=(time.monotonic(),msg)
  if key=='clock':
   value=stamp(msg.clock)
   if lastclock is not None and value<lastclock:clock_jump=True;obs('clock_jump',{'before':lastclock,'after':value});wire('CLOCK_JUMP',{'before':lastclock,'after':value})
   if lastclock is not None and value>lastclock:clockticks+=1;lastadvance=time.monotonic()
   lastclock=value
   if not rows or time.monotonic()-getattr(receive,'clock_emit',0)>.1:obs('clock',{'sim_s':value});receive.clock_emit=time.monotonic()
  elif key=='odom':
   p=msg.pose.pose.position;q=msg.pose.pose.orientation;v=msg.twist.twist
   obs('odom',{'stamp_s':stamp(msg.header.stamp),'frame_id':msg.header.frame_id,'child_frame_id':msg.child_frame_id,'position':clean([p.x,p.y,p.z]),'quaternion_xyzw':clean([q.x,q.y,q.z,q.w]),'linear':clean([v.linear.x,v.linear.y,v.linear.z]),'angular':clean([v.angular.x,v.angular.y,v.angular.z])})
  elif key.startswith('/cmd_vel'):
   obs(key,{'linear':clean([msg.linear.x,msg.linear.y,msg.linear.z]),'angular':clean([msg.angular.x,msg.angular.y,msg.angular.z])})
  elif key=='action_status':obs(key,[{'goal_uuid':bytes(x.goal_info.goal_id.uuid).hex(),'status':x.status,'accepted_stamp_s':stamp(x.goal_info.stamp)} for x in msg.status_list])
 for name,typ,qos in [('clock',Clock,qos_profile_sensor_data),('scan',LaserScan,qos_profile_sensor_data),('odom',Odometry,qos_profile_sensor_data),('map',OccupancyGrid,durable),('amcl_pose',PoseWithCovarianceStamped,durable)]:node.create_subscription(typ,topic(name),lambda m,k=name:receive(k,m),qos)
 for name in ['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel']:node.create_subscription(Twist,topic(name),lambda m,k=name:receive(k,m),qos_profile_sensor_data)
 node.create_subscription(GoalStatusArray,topic('navigate_to_pose/_action/status'),lambda m:receive('action_status',m),durable)
 if profile.get('task_version'):
  def path_evidence(m):
   with (run/'runtime/plans.jsonl').open('a') as f:f.write(encode({'receive_monotonic_s':time.monotonic(),'source_ros_s':stamp(m.header.stamp),'message':message_to_ordereddict(m)}).decode())
  node.create_subscription(NavPath,topic('plan'),path_evidence,10)
 def action_record(kind,data):
  item={'kind':kind,'monotonic_s':time.monotonic(),'ros_s':rosnow(),'data':data,'line':len(actions)+1};actions.append(item);actionfile.write(encode(item).decode());actionfile.flush();return item
 def finish(action,status=None,result=None):
  nonlocal terminal
  if terminal is not None:return
  terminal=action_record('terminal',{'uuid':goal_uuid,'action':action,'status':status,'result':result})
  write_json(run/'runtime/goal-terminal.json',terminal);wire('ACTION_TERMINAL',terminal,action_request)
 def pump():
  nonlocal lastheart,lasttf,handle,response_recorded,result_future,stale_emitted
  rclpy.spin_once(node,timeout_sec=.02);now=time.monotonic()
  if goal_future is not None and goal_future.done() and not response_recorded:
   handle=goal_future.result();response_recorded=True
   item=action_record('response',{'uuid':bytes(handle.goal_id.uuid).hex(),'accepted':handle.accepted});wire('ACTION_RESPONSE',item,action_request)
   if handle.accepted:result_future=handle.get_result_async()
   else:finish('REJECTED')
  if result_future is not None and result_future.done() and terminal is None:
   response=result_future.result();finish({4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}.get(response.status,'UNKNOWN'),response.status,message_to_ordereddict(response.result))
  if now-lasttf>.1:
   lasttf=now
   try:
    t=buffer.lookup_transform('map','base_footprint',Time());p=t.transform.translation;q=t.transform.rotation
    obs('map_base',{'stamp_s':stamp(t.header.stamp),'position':clean([p.x,p.y,p.z]),'quaternion_xyzw':clean([q.x,q.y,q.z,q.w])})
   except Exception:pass
  if navigation_ready and goal_future is not None and terminal is None and not stale_emitted and (now-lastadvance>1.5 or any(now-latest.get(k,(0,None))[0]>1.5 for k in ['scan','odom'])):
   stale_emitted=True;wire('INPUT_STALE',{'clock_last_advance_age_wall_s':now-lastadvance,'input_ages':{k:now-latest.get(k,(0,None))[0] for k in ['scan','odom']}})
  if probe_pause_deadline is not None and now>probe_pause_deadline:raise RuntimeError('PAUSE_CHECK_DEADLINE')
  if now-lastheart>.5:wire('HEARTBEAT',{'clock_s':lastclock},mirror=False);lastheart=now
  flush()
 def wait(predicate,deadline,label):
  while time.monotonic()<deadline:
   pump()
   if clock_jump:raise RuntimeError('CLOCK_JUMP')
   if predicate():return
  raise TimeoutError(label)
 def client_for(typ,name):
  key=(typ,name)
  if key not in service_clients:service_clients[key]=node.create_client(typ,topic(name))
  return service_clients[key]
 def service(typ,name,request,deadline):
  c=client_for(typ,name);wait(lambda:c.service_is_ready(),deadline,'service unavailable '+name);wire('SERVICE_REQUEST',{'service':name,'request_type':typ.__name__,'command':getattr(request,'command',None)});f=c.call_async(request)
  try:wait(f.done,deadline,'service timeout '+name)
  except TimeoutError:late_services.append((name,c,f));raise
  response=f.result();wire('SERVICE_RESPONSE',{'service':name,'completed':True});return response
 def read_service(typ,name,request,deadline):
  # Only idempotent readiness queries are polled. Never repeat STARTUP or Action.
  while time.monotonic()<deadline:
   try:return service(typ,name,request,min(deadline,time.monotonic()+2.0))
   except TimeoutError:wire('READINESS_QUERY_TIMEOUT',{'service':name,'remaining_s':max(0.,deadline-time.monotonic())})
  raise TimeoutError('read-only readiness deadline '+name)
 navnodes=['controller_server','smoother_server','planner_server','route_server','behavior_server','velocity_smoother','collision_monitor','bt_navigator','waypoint_follower','docking_server']
 def states(names,deadline):
  out={}
  for name in names:
   response=read_service(GetState,name+'/get_state',GetState.Request(),deadline);out[name]={'id':response.current_state.id,'label':response.current_state.label}
  return out
 def sensor_ready():
  if time.monotonic()-lastadvance>1.5:return False
  if clockticks<profile['readiness']['clock_samples'] or any(k not in latest for k in ['scan','odom','clock']):return False
  if any(time.monotonic()-latest[k][0]>1.5 for k in ['scan','odom','clock']):return False
  scan=latest['scan'][1];odom=latest['odom'][1]
  if scan.header.frame_id!='base_scan' or odom.header.frame_id!='odom' or odom.child_frame_id not in ['base_footprint','base_link']:return False
  if abs(stamp(scan.header.stamp)-lastclock)>.5 or abs(stamp(odom.header.stamp)-lastclock)>.5:return False
  if sum(math.isfinite(x) and scan.range_min<=x<=scan.range_max for x in scan.ranges)<5:return False
  try:
   t=buffer.lookup_transform('odom','base_footprint',Time());buffer.lookup_transform('base_footprint','base_scan',Time())
   if abs(stamp(t.header.stamp)-lastclock)>.5:return False
  except Exception:return False
  return True
 def params_snapshot(names,deadline):
  allp={}
  for name in names:
   req=ListParameters.Request();req.depth=0;keys=service(ListParameters,name+'/list_parameters',req,deadline).result.names
   req=GetParameters.Request();req.names=keys;vals=service(GetParameters,name+'/get_parameters',req,deadline).values
   allp[name]=dict(zip(keys,[parameter_value_to_python(v) for v in vals]));write_json(run/'runtime'/(name.replace('/','__')+'.yaml'),{name:{'ros__parameters':allp[name]}})
  write_json(run/'runtime/all-parameters.json',allp);return allp
 def execute(op,data,deadline,rid):
  nonlocal initial_sent,navigation_ready,goal_future,goal_uuid,action_request,probe_origin,probe_pause_deadline,fault_deactivated
  if op=='graph':
   names=node.get_node_names();foreign=[x for x in names if x!='cmr_worker' and not x.startswith('_ros2cli_daemon')]
   if foreign:raise RuntimeError('existing ROS owners '+str(foreign))
   return {'nodes':names,'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
  if op=='sensors':
   wait(sensor_ready,deadline,'clock/scan/odom/TF readiness');samples={}
   for k in ['clock','scan','odom']:
    m=latest[k][1];(run/'runtime'/('sensor-'+k+'-'+rid+'.yaml')).write_text(message_to_yaml(m));samples[k]={'receive_age_wall_s':time.monotonic()-latest[k][0],'stamp_s':stamp(m.clock if k=='clock' else m.header.stamp)}
   write_json(run/'runtime'/('sensor-readiness-'+rid+'.json'),samples);return samples
  if op=='manager':
   phase=data['phase'];command=data['command']
   if phase not in ['localization','navigation'] or command not in ['STARTUP','SHUTDOWN']:raise ValueError('manager command')
   if command=='STARTUP':
    client_for(ManageLifecycleNodes,'lifecycle_manager_'+phase+'/manage_nodes')
    names=['map_server','amcl'] if phase=='localization' else navnodes
    before=states(names,deadline)
    gate=read_service(Trigger,'lifecycle_manager_'+phase+'/is_active',Trigger.Request(),deadline)
    if gate.success:raise RuntimeError('manager already active before owned STARTUP')
    wire('MANAGER_SERVICE_GATE_READY',{'phase':phase,'states':before,'is_active':gate.success},rid)
   req=ManageLifecycleNodes.Request();req.command=getattr(req,command)
   response=service(ManageLifecycleNodes,'lifecycle_manager_'+phase+'/manage_nodes',req,deadline)
   if not response.success:raise RuntimeError('manager returned false '+phase)
   names=['map_server','amcl'] if phase=='localization' else navnodes
   st=states(names,deadline)
   if command=='STARTUP' and any(x['id']!=3 for x in st.values()):raise RuntimeError('manager not active '+str(st))
   return {'success':True,'states':st}
  if op=='shutdown_actual':
   names=['map_server','amcl'] if data['phase']=='localization' else navnodes
   records=[]
   if data.get('coordinate_manager'):
    def stable_manager():
     gate=read_service(Trigger,'lifecycle_manager_'+data['phase']+'/is_active',Trigger.Request(),deadline)
     return gate.success and all(x['id']==3 for x in states(names,deadline).values())
    wait(stable_manager,deadline,'manager recovery did not reach stable active state')
    before=states(names,deadline);wire('MANAGER_RECOVERY_STABLE',{'states':before},rid)
    req=ManageLifecycleNodes.Request();req.command=req.SHUTDOWN
    response=service(ManageLifecycleNodes,'lifecycle_manager_'+data['phase']+'/manage_nodes',req,deadline)
    after=states(names,deadline);wire('COORDINATED_MANAGER_SHUTDOWN',{'before':before,'response':response.success,'after':after},rid)
    if not response.success or any(x['id']!=4 for x in after.values()):raise RuntimeError('manager shutdown not finalized')
    return {'success':True,'manager_coordinated':True,'states':after}
   for name in reversed(names):
    seen=set()
    while time.monotonic()<deadline:
     st=states([name],deadline)[name]['id']
     if st==4:break
     if st not in [1,2,3]:pump();continue
     if st in seen:raise RuntimeError('shutdown transition repeated '+name+' '+str(st))
     seen.add(st);req=ChangeState.Request();req.transition.id=shutdown_transition(st)
     response=service(ChangeState,name+'/change_state',req,deadline)
     after=states([name],deadline)[name]
     record={'node':name,'before':st,'transition':req.transition.id,'response':response.success,'after':after};records.append(record);wire('ACTUAL_SHUTDOWN_TRANSITION',record,rid)
     if after['id']==4:break
    else:raise TimeoutError('shutdown actual '+name)
   return {'success':True,'transitions':records,'states':states(names,deadline)}
  if op=='fault_ready':
   if profile.get('fault') not in ['controller-deactivate','short-deadline','pause-gazebo'] or data['uuid']!=goal_uuid:raise ValueError('fault scope/UUID')
   if not navigation_ready or handle is None or not handle.accepted:raise RuntimeError('NOT_EXERCISED: no accepted ready goal')
   st=states(['controller_server'],deadline)
   if st['controller_server']['id']!=3:raise RuntimeError('NOT_EXERCISED: controller inactive')
   answer={}
   def qualifies():
    if terminal is not None:raise RuntimeError('NOT_EXERCISED: early terminal')
    accepted=[a for a in actions if a['kind']=='response' and a['data']['accepted']][-1]
    od=[r for r in rows if r['kind']=='odom' and r['receive_monotonic_s']>=accepted['monotonic_s']]
    if len(od)<3 or not sensor_ready():return False
    stamps=[r['data']['stamp_s'] for r in od]
    if not all(b>a for a,b in zip(stamps,stamps[1:])):return False
    movement=math.hypot(od[-1]['data']['position'][0]-od[0]['data']['position'][0],od[-1]['data']['position'][1]-od[0]['data']['position'][1])
    speed=math.hypot(*od[-1]['data']['linear'][:2])
    statuses=latest.get('action_status',(0,None))[1]
    active=statuses is not None and any(bytes(x.goal_info.goal_id.uuid).hex()==goal_uuid and x.status==2 for x in statuses.status_list)
    if movement<.05 or speed<.01 or not active:return False
    answer.update(uuid=goal_uuid,monotonic_s=time.monotonic(),sim_s=lastclock,odom_lines=[od[0]['line'],od[-1]['line']],samples=len(od),displacement_m=movement,linear_mps=speed,active_status=2,controller=st['controller_server'],fresh=True)
    return True
   wait(qualifies,deadline,'NOT_EXERCISED: movement qualification');return answer
  if op=='fault_deactivate':
   if profile.get('fault')!='controller-deactivate' or data['uuid']!=goal_uuid or fault_deactivated:raise ValueError('deactivate whitelist/once/UUID')
   if terminal is not None:return {'activated':False,'reason':'terminal before request'}
   before=states(['controller_server'],deadline)
   if before['controller_server']['id']!=3:raise RuntimeError('NOT_EXERCISED: controller already inactive')
   fault_deactivated=True;req=ChangeState.Request();req.transition.id=4
   response=service(ChangeState,'controller_server/change_state',req,deadline)
   after=states(['controller_server'],deadline)
   return {'activated':bool(response.success and after['controller_server']['id'] in [1,2,4]),'before':before,'response':response.success,'after':after,'uuid':goal_uuid,'monotonic_s':time.monotonic(),'sim_s':lastclock}
  if op=='failure_stop':
   if not profile.get('p0_08') or goal_uuid is None:raise ValueError('abnormal stop scope')
   request={'after_monotonic_s':time.monotonic(),'after_sim_s':lastclock,**data};report=None
   while time.monotonic()<deadline-.3:
    pump();report=observe_stop(rows,request['after_monotonic_s'],request['command_after_monotonic_s'],profile['stop'])
    if all(report['checks'].values()) and sensor_ready():break
   if report is None:raise RuntimeError('no failure stop observation budget')
   report['request']=request;report['uuid']=goal_uuid;report['terminal']=terminal['data']['action'] if terminal else 'UNKNOWN'
   write_json(run/'runtime/failure-stop.json',report);return report
  if op=='localize':
   if initial_sent:raise RuntimeError('initialpose already sent')
   wait(lambda:initialpub.get_subscription_count()>=1 and 'map' in latest,deadline,'map / initialpose subscriber')
   req=strict_json((run/'artifacts/initialpose.json').read_text());m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=node.get_clock().now().to_msg()
   for k,v in req['pose']['pose']['position'].items():setattr(m.pose.pose.position,k,v)
   for k,v in req['pose']['pose']['orientation'].items():setattr(m.pose.pose.orientation,k,v)
   m.pose.covariance=req['pose']['covariance'];write_json(run/'runtime/initialpose-request.json',message_to_ordereddict(m));initial_sent=True;initialpub.publish(m);wire('INITIALPOSE_PUBLISHED',{'stamp_s':stamp(m.header.stamp)},rid)
   def localized():
    if 'amcl_pose' not in latest:return False
    try:t=buffer.lookup_transform('map','base_link',Time());return abs(stamp(t.header.stamp)-lastclock)<=.5
    except Exception:return False
   wait(localized,deadline,'localized map->base TF');return {'localized':True}
  if op=='withhold':
   st=states(['map_server','amcl'],deadline)
   if any(x['id']!=3 for x in st.values()) or not sensor_ready():raise RuntimeError('fault prerequisites absent')
   wire('FAULT_ACTIVATED',{'fault':'withhold-initial-pose','initialpose_published':False,'goal_sent':False},rid)
   wait(lambda:False,deadline,'CONTROLLED_INITIALPOSE_TIMEOUT')
  if op=='recorder':
   name=data['name'];required=data['topics']
   wait(lambda:all(any(x.node_name==name for x in node.get_subscriptions_info_by_topic(topic(t))) for t in required),deadline,'recorder matching subscriptions')
   paused=service(IsPaused,name+'/is_paused',IsPaused.Request(),deadline)
   if paused.paused:raise RuntimeError('recorder paused')
   return {'name':name,'matched':required,'paused':False}
  if op=='nav_ready':
   names=['map_server','amcl',*navnodes,'local_costmap/local_costmap','global_costmap/global_costmap'];st=states(names,deadline)
   if any(x['id']!=3 for x in st.values()):raise RuntimeError('nodes inactive')
   p=params_snapshot(names,deadline)
   if p['controller_server']['FollowPath.plugin']!=PLUGINS[method]:raise RuntimeError('plugin mismatch')
   wait(lambda:sensor_ready() and ac.server_is_ready(),deadline,'navigation fresh inputs / action')
   if not initial_sent:raise RuntimeError('initialpose not published')
   if node.get_node_names().count('bt_navigator')!=1 and not ns:raise RuntimeError('not unique navigator')
   endpoints={}
   for t,owner in [('/cmd_vel_nav','controller_server'),('/cmd_vel_smoothed','velocity_smoother'),('/cmd_vel','collision_monitor')]:
    pubs=node.get_publishers_info_by_topic(topic(t));subs=node.get_subscriptions_info_by_topic(topic(t));endpoints[t]={'publishers':[x.node_name for x in pubs],'subscribers':[x.node_name for x in subs],'types':[x.topic_type for x in pubs]}
    if not ns and (owner not in endpoints[t]['publishers'] or not subs or any(x.topic_type!='geometry_msgs/msg/Twist' for x in pubs)):raise RuntimeError('velocity chain '+t)
   # Preserved P0-04 profile is already a verified short task; additionally reject lethal/unknown endpoint footprint cells.
   costs={};g=strict_json((run/'artifacts/goal.json').read_text())['pose']['pose']['position']
   for kind in ['global','local']:
    cm=service(GetCostmap,kind+'_costmap/get_costmap',GetCostmap.Request(),deadline).map
    (run/'runtime'/(kind+'-costmap.yaml')).write_text(message_to_yaml(cm));md=cm.metadata;x,y=g['x'],g['y']
    # Rolling local map covers the current robot. The global map checks the remote goal.
    if kind=='local' and profile.get('task_version'):
     tr=buffer.lookup_transform('map','base_link',Time()).transform;x,y=tr.translation.x,tr.translation.y
    if cm.header.frame_id!='map':
     tr=buffer.lookup_transform(cm.header.frame_id,'map',Time()).transform;angle=math.atan2(2*(tr.rotation.w*tr.rotation.z+tr.rotation.x*tr.rotation.y),1-2*(tr.rotation.y**2+tr.rotation.z**2));x,y=tr.translation.x+math.cos(angle)*x-math.sin(angle)*y,tr.translation.y+math.sin(angle)*x+math.cos(angle)*y
    radius=p[kind+'_costmap/'+kind+'_costmap']['robot_radius']+p[kind+'_costmap/'+kind+'_costmap']['footprint_padding'];worst=0
    for iy in range(math.floor((y-radius-md.origin.position.y)/md.resolution),math.ceil((y+radius-md.origin.position.y)/md.resolution)+1):
     for ix in range(math.floor((x-radius-md.origin.position.x)/md.resolution),math.ceil((x+radius-md.origin.position.x)/md.resolution)+1):
      if math.hypot(md.origin.position.x+(ix+.5)*md.resolution-x,md.origin.position.y+(iy+.5)*md.resolution-y)>radius+.71*md.resolution:continue
      if not 0<=ix<md.size_x or not 0<=iy<md.size_y:raise RuntimeError('goal outside costmap')
      worst=max(worst,int(cm.data[iy*md.size_x+ix]))
    if worst>=253:raise RuntimeError('goal footprint not free '+str(worst))
    costs[kind]=worst
   gate_samples=[]
   def final_gate():
    try:
     tr=buffer.lookup_transform('map','base_link',Time());tfstamp=stamp(tr.header.stamp);sample={'monotonic_s':time.monotonic(),'tf_stamp_s':tfstamp,'clock_subscription_s':lastclock,'node_ros_s':rosnow(),'age_s':abs(tfstamp-lastclock),'sensors_ready':sensor_ready(),'action_ready':ac.server_is_ready()}
     gate_samples.append(sample)
     return sample['age_s']<=.5 and sample['sensors_ready'] and sample['action_ready']
    except Exception as exc:gate_samples.append({'monotonic_s':time.monotonic(),'error':str(exc)});return False
   try:wait(final_gate,deadline,'final fresh map/base TF and sensor gate')
   finally:write_json(run/'runtime/nav-final-gate.json',{'samples':gate_samples[-100:],'threshold_s':.5})
   current_tf=buffer.lookup_transform('map','base_link',Time())
   navigation_ready=True;report={'map_base_tf_stamp_s':stamp(current_tf.header.stamp),'states':st,'plugin':p['controller_server']['FollowPath.plugin'],'endpoints':endpoints,'costs':costs,'sim_s':lastclock};write_json(run/'runtime/nav-readiness.json',report);return report
  if op=='input_isolation':
   if profile.get('profile_version')!=2:raise RuntimeError('dynamic audit only')
   inventory={}
   for name,namespace in node.get_node_names_and_namespaces():
    if name in navnodes+['amcl','local_costmap','global_costmap']:
     subscriptions=node.get_subscriber_names_and_types_by_node(name,namespace)
     inventory[namespace+'/'+name]=subscriptions
     if any(t.startswith('/cmr_eval/') or 'future_trajectory' in t for t,types in subscriptions):raise RuntimeError('truth input leak')
   sources={}
   for t in ['/scan','/odom','/tf','/clock']:
    pubs=node.get_publishers_info_by_topic(t);sources[t]=[{'node':p.node_name,'namespace':p.node_namespace,'type':p.topic_type} for p in pubs]
    if any('truth' in p.node_name or 'scene_driver' in p.node_name for p in pubs):raise RuntimeError('evaluation publisher in control input')
   report={'subscriptions':inventory,'control_publishers':sources,'truth_leak_detected':False,'scope':'actual graph and original Nav2 config; no truth bridge added'}
   if not inventory:raise RuntimeError('missing Nav2 graph evidence')
   write_json(run/'runtime/dynamic-input-isolation.json',report);return report
  if op=='probe_pause':
   if profile.get('mode')!='probe' or goal_future is not None:raise RuntimeError('probe pause only before goals')
   probe_pause_deadline=time.monotonic()+10 if data['enter'] else None
   return {'pause_check':data['enter']}
  if op=='probe_observe':
   if profile.get('mode')!='probe' or goal_future is not None:raise RuntimeError('probe must not send goal')
   if not sensor_ready():raise RuntimeError('probe sensors stale')
   if probe_origin is None:probe_origin=list(message_to_ordereddict(latest['odom'][1])['pose']['pose']['position'].values())
   end_sim=lastclock+data['duration_sim_s'];first=len(rows)
   wait(lambda:lastclock>=end_sim,deadline,'probe observation')
   observed=[r for r in rows[first:] if r['kind']=='odom']
   if not observed:raise RuntimeError('probe missing odom')
   for row in observed:
    p=row['data']
    if not all(isinstance(x,(int,float)) and math.isfinite(x) for key in ['position','linear','angular'] for x in p[key]):raise RuntimeError('probe invalid odom')
    if math.dist(p['position'][:2],probe_origin[:2])>.02 or math.hypot(*p['linear'][:2])>.01 or abs(p['angular'][2])>.03:raise RuntimeError('probe robot moved')
   return {'robot_stationary':True,'samples':len(observed),'sim_s':lastclock}
  if op=='send':
   if profile.get('mode')=='probe':raise RuntimeError('probe forbids navigation goals')
   if not navigation_ready and not ns:raise RuntimeError('not navigation ready')
   if goal_future is not None:raise RuntimeError('second goal refused')
   if not ns and not sensor_ready():raise RuntimeError('send gate input freshness lost')
   wait(ac.server_is_ready,deadline,'Action server discovery')
   goal_uuid=data['uuid'];action_request=rid
   if len(goal_uuid)!=32 or int(goal_uuid,16)==0:raise ValueError('invalid UUID')
   g=strict_json((run/'artifacts/goal.json').read_text());m=NavigateToPose.Goal();m.pose.header.frame_id='map';m.pose.header.stamp=node.get_clock().now().to_msg()
   for k,v in g['pose']['pose']['position'].items():setattr(m.pose.pose.position,k,v)
   for k,v in g['pose']['pose']['orientation'].items():setattr(m.pose.pose.orientation,k,v)
   m.behavior_tree=g['behavior_tree'];d=action_record('dispatch',{'uuid':goal_uuid,'request':message_to_ordereddict(m)});write_json(run/'runtime/goal-dispatch.json',d);wire('GOAL_SEND',d,rid)
   goal_future=ac.send_goal_async(m,goal_uuid=UUID(uuid=list(bytes.fromhex(goal_uuid))),feedback_callback=lambda x:action_record('feedback',message_to_ordereddict(x)))
   try:wait(lambda:response_recorded,deadline,'acceptance response')
   except TimeoutError:return {'acceptance':'UNKNOWN','uuid':goal_uuid}
   return {'acceptance':'ACCEPTED' if handle.accepted else 'REJECTED','uuid':goal_uuid}
  if op=='cancel':
   if goal_uuid is None:return {'not_sent':True}
   req=CancelGoal.Request();req.goal_info.goal_id=UUID(uuid=list(bytes.fromhex(goal_uuid)))
   response=service(CancelGoal,'navigate_to_pose/_action/cancel_goal',req,deadline);d=message_to_ordereddict(response);action_record('cancel_response',d);wire('CANCEL_RESPONSE',d,rid);return d
  if op=='stop':
   if terminal is None:return {'observed':False,'reason':'terminal unknown'}
   goal=strict_json((run/'artifacts/goal.json').read_text());p=strict_json((run/'runtime/all-parameters.json').read_text())['controller_server'];target=lastclock+8
   wait(lambda:lastclock>=target,deadline,'post-terminal sim-time stop window')
   report=analyze(rows,actions,goal,p,arrival_policy=profile.get('p0_08',{}).get('arrival_policy','last_fresh'));write_json(run/'runtime/behavior.json',report);return report
  if op=='mark':
   wire(data['name'],data.get('data',{}),rid)
   if data['name'] in ('RECORDING_START','RECORDING_END'):
    if lastclock is None:raise RuntimeError('MARKER_CLOCK_MISSING')
    marked_clock=lastclock
    if not eventpub.wait_for_all_acked(Duration(seconds=min(.5,max(0.,deadline-time.monotonic())))):raise RuntimeError('MARKER_ACK_TIMEOUT')
    # A DDS acknowledgement is not a disk-content claim. Allow fresh source
    # callbacks to drain before parent closes recorder; closed MCAP is checked.
    wait(lambda:lastclock is not None and lastclock>=marked_clock+.1,deadline,'marker transport drain')
    return {'marked':True,'reliable_acknowledged':True,'source_clock_advance_s':lastclock-marked_clock,'disk_content_verified':False}
   return {'marked':True}
  if op=='exit':return {'exiting':True}
  raise ValueError('unknown op '+op)
 wire('HELLO',{'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'namespace':ns})
 running=True
 try:
  while running:
   pump()
   for name,c,f in list(late_services):
    if f.done():wire('LATE_SERVICE_RESPONSE',{'service':name,'response':message_to_ordereddict(f.result())});late_services.remove((name,c,f))
   ready,_,_=select.select([sys.stdin],[],[],0)
   if ready:
    data=os.read(sys.stdin.fileno(),65536)
    if not data:break
    inbuf.extend(data)
    if len(inbuf)>65536:raise ValueError('IPC line/buffer too large')
    while b'\n' in inbuf:
     line,_,rest=inbuf.partition(b'\n');inbuf=bytearray(rest);cmd=strict_json(line.decode());lastseq=validate_envelope(cmd,run.name,lastseq)
     if cmd['type']!='COMMAND':raise ValueError('not a command')
     rid=cmd['request_id'];op=cmd['data']['op']
     if rid in cache:
      if cache[rid][0]!=cmd['data']:raise ValueError('conflicting request_id')
      wire('RESULT',cache[rid][1],rid);continue
     deadline=cmd['data']['deadline'];result={}
     try:
      if not math.isfinite(deadline) or deadline<=time.monotonic():raise TimeoutError('expired command')
      result={'ok':True,'value':execute(op,cmd['data'].get('args',{}),deadline,rid)}
     except Exception as exc:result={'ok':False,'error':type(exc).__name__+': '+str(exc)}
     cache[rid]=(cmd['data'],result);wire('RESULT',result,rid)
     if op=='exit':running=False
  flush()
 finally:
  observations.flush();observations.close();actionfile.flush();actionfile.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
