"""Pair-scoped commissioning session. One real cold start and one goal only."""
from pathlib import Path
import ast,copy,gzip,hashlib,importlib.util,json,math,os,platform,signal,subprocess,sys,tarfile,tempfile,time,traceback
from datetime import datetime,timezone
import yaml
root=Path('/home/robot/commercial_mobile_robot');pair=root/'evidence/env/p0-04-pair-q2MXrdh3'
spec=json.loads((pair/'pair.json').read_text());method=sys.argv[1]
assert method in ['mppi','dwb']
plugin={'mppi':'nav2_mppi_controller::MPPIController','dwb':'dwb_core::DWBLocalPlanner'}[method]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def put(p,data):
 with p.open('x') as f:f.write(data if isinstance(data,str) else json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
for name,h in spec['inputs_sha256'].items():assert sha(pair/'inputs'/name)==h,name
for resource in spec['resources'].values():assert sha(Path(resource['path']))==resource['sha256']
# Refuse to touch another owner's simulator or ROS stack.
conflicts=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  if p.stat().st_uid!=os.getuid():continue
  cmd=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
  if any(s in cmd for s in ['/lib/nav2_controller/controller_server','/lib/nav2_bt_navigator/bt_navigator','/lib/nav2_amcl/amcl','gz sim -s','gz-sim-server']):conflicts.append({'pid':int(p.name),'command':cmd})
 except (PermissionError,ProcessLookupError,FileNotFoundError):pass
assert not conflicts,conflicts
run=Path(tempfile.mkdtemp(prefix=f'p0-04-{method}-',dir=root/'evidence/runs'))
for d in ['artifacts','runtime','logs','graphs']:(run/d).mkdir()
print('NEW_RUN',run,flush=True)
for p in (pair/'inputs').iterdir():
 target=run/'artifacts'/({'nav2_'+method+'.yaml':'nav2.yaml','scenario-template.yaml':'scenario-before.yaml'}.get(p.name,p.name))
 if p.name in ['nav2_mppi.yaml','nav2_dwb.yaml'] and p.name!=f'nav2_{method}.yaml':continue
 with target.open('xb') as f:f.write(p.read_bytes())
with (run/'artifacts/pair-at-start.json').open('xb') as f:f.write((pair/'pair.json').read_bytes())
with (run/'artifacts/commission.py').open('xb') as f:f.write(Path(__file__).read_bytes())
start=time.monotonic();boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
status=subprocess.check_output(['git','status','--porcelain=v1'],cwd=root,text=True)
head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
put(run/'repo-at-start.json',{'git_commit':head,'dirty':bool(status),'status':status,'method':method,'pair_id':pair.name,'started_at_utc':datetime.now(timezone.utc).isoformat().replace('+00:00','Z'),'start_monotonic_s':start,'boot_id':boot})
protected=json.loads((pair/'implementation-protected-before.json').read_text())
assert all((root/name).is_file() and sha(root/name)==h for name,h in protected.items()),'Protected source changed'
if status:
 names=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z'],cwd=root).decode().split('\0')
 with tarfile.open(run/'artifacts/source-at-start.tgz','w:gz') as archive:
  for name in sorted(set(names)):
   if name and Path(name).parts[0] not in ['evidence','build','install','log','.git'] and (root/name).is_file():archive.add(root/name,arcname=name,recursive=False)
env=os.environ.copy();env.update({'CMR_RUN':str(run),'CMR_METHOD':method,'ROS_DOMAIN_ID':'71','GZ_PARTITION':'cmr_'+run.name,'ROS_AUTOMATIC_DISCOVERY_RANGE':'SYSTEM_DEFAULT','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp','LIBGL_ALWAYS_SOFTWARE':'1','QT_QPA_PLATFORM':'xcb','PYTHONDONTWRITEBYTECODE':'1','PYTHONUNBUFFERED':'1','RCUTILS_COLORIZED_OUTPUT':'0'})
env.pop('ROS_LOCALHOST_ONLY',None)
profile=str(root/'src/cmr_bringup/config/fastdds_loopback_udp.xml')
env.update({'FASTRTPS_DEFAULT_PROFILES_FILE':profile,'FASTDDS_DEFAULT_PROFILES_FILE':profile,'GZ_SIM_RESOURCE_PATH':'/opt/ros/jazzy/share/nav2_minimal_tb3_sim/models:/opt/ros/jazzy/share'})
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  if p.stat().st_uid==os.getuid() and (p/'comm').read_text().strip()=='xfce4-session':
   e=dict(x.split('=',1) for x in (p/'environ').read_bytes().decode().split('\0') if '=' in x)
   for k in ['DISPLAY','XAUTHORITY']:env[k]=e[k]
 except (PermissionError,ProcessLookupError,FileNotFoundError):pass
assert subprocess.run(['xdpyinfo'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0
os.environ.update(env)
import shlex
put(run/'session.bash','source /opt/ros/jazzy/setup.bash\n'+'\n'.join('export '+k+'='+shlex.quote(env[k]) for k in ['CMR_RUN','CMR_METHOD','ROS_DOMAIN_ID','GZ_PARTITION','ROS_AUTOMATIC_DISCOVERY_RANGE','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','LIBGL_ALWAYS_SOFTWARE','QT_QPA_PLATFORM','GZ_SIM_RESOURCE_PATH','DISPLAY','XAUTHORITY','PYTHONDONTWRITEBYTECODE','PYTHONUNBUFFERED'])+'\nunset ROS_LOCALHOST_ONLY\n')
put(run/'runtime/environment.json',{k:env[k] for k in ['ROS_DOMAIN_ID','GZ_PARTITION','ROS_AUTOMATIC_DISCOVERY_RANGE','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','LIBGL_ALWAYS_SOFTWARE','QT_QPA_PLATFORM','GZ_SIM_RESOURCE_PATH','DISPLAY','XAUTHORITY']})
events=(run/'logs/session-events.jsonl').open('x');owned=[];files=[]
def emit(kind,data):
 events.write(json.dumps({'kind':kind,'monotonic_s':time.monotonic(),'utc':datetime.now(timezone.utc).isoformat(),'data':data},allow_nan=False)+'\n');events.flush();print(kind,json.dumps({k:v for k,v in data.items() if k!='command'}),flush=True)
def spawn(label,command):
 f=(run/'logs'/f'{label}.log').open('x');files.append(f)
 proc=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,env=env,start_new_session=True)
 ticks=Path(f'/proc/{proc.pid}/stat').read_text().split()[21]
 owned.append((label,proc,ticks));emit('process_started',{'label':label,'pid':proc.pid,'start_ticks':ticks,'command':command});return proc
def short(label,command,timeout=30,allowed=(0,)):
 out=subprocess.run(command,env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=timeout)
 put(run/'logs'/f'{label}.out',out.stdout);put(run/'logs'/f'{label}.err',out.stderr);put(run/'logs'/f'{label}.exit',str(out.returncode)+'\n')
 emit('command_return',{'label':label,'exit':out.returncode});assert out.returncode in allowed,(label,out.stdout,out.stderr);return out.stdout
import rclpy
from rclpy.parameter import parameter_value_to_python
from rclpy.action import ActionClient
from rclpy.time import Time
from rcl_interfaces.srv import ListParameters,GetParameters
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes,GetCostmap
from nav2_msgs.action import NavigateToPose
from rosidl_runtime_py.convert import message_to_ordereddict,message_to_yaml
modspec=importlib.util.spec_from_file_location('p04start',run/'artifacts/startup_scoped.py');mod=importlib.util.module_from_spec(modspec);modspec.loader.exec_module(mod)
class Session(mod.Startup):
 def start(self,label,command):
  proc=spawn(label,command);self.children.append((label,proc))
rclpy.init(signal_handler_options=mod.SignalHandlerOptions.NO)
s=Session((-2.0,-.5,0.0));observer=None;failure=None;ready=False;cleanup=[]
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(s.node,timeout_sec=.03)
def call(typ,path,request):
 c=s.node.create_client(typ,path);res=s.call(c,request,10);s.node.destroy_client(c)
 assert res is not None,path
 return res
nodes=['map_server','amcl',*mod.NAV_NODES,'local_costmap/local_costmap','global_costmap/global_costmap']
def states():
 out={}
 for node in nodes:
  st=call(GetState,'/'+node+'/get_state',GetState.Request()).current_state;out[node]={'id':st.id,'label':st.label,'monotonic_s':time.monotonic(),'sim_s':s.clock}
 return out
def snapshot():
 params={}
 for node in nodes:
  req=ListParameters.Request();req.depth=0
  names=call(ListParameters,'/'+node+'/list_parameters',req).result.names
  req2=GetParameters.Request();req2.names=names
  values=call(GetParameters,'/'+node+'/get_parameters',req2).values
  params[node]=dict(zip(names,[parameter_value_to_python(v) for v in values]))
  put(run/'runtime'/(node.replace('/','__')+'.yaml'),yaml.safe_dump({node:{'ros__parameters':params[node]}},sort_keys=True))
 put(run/'runtime/all-parameters.json',params)
 return params
def freeze_manifest():
 scen=json.loads((run/'artifacts/scenario-before.yaml').read_text());scen['scenario_revision']=run.name
 scen['resources']['world']['path']=str(run/'artifacts/world-ogre1.sdf')
 scen['task']['common_constraints_ref']['path']=str(run/'artifacts/common_constraints.json')
 scen['verification']['evidence_refs']=[str(run/'runtime/pre-goal.json')]
 put(run/'artifacts/scenario.yaml',scen)
 artifacts=[]
 for p in sorted((run/'artifacts').iterdir()):
  if not p.is_file() or not p.stat().st_size:continue
  kind={'scenario.yaml':'scenario','nav2.yaml':'config','common_constraints.json':'common_constraints','source-at-start.tgz':'source_snapshot'}.get(p.name,'other')
  artifacts.append({'id':p.stem.replace('-','_').replace('.','_'),'path':str(p.relative_to(run)),'sha256':sha(p),'kind':kind})
 for p in [run/'runtime/controller_server.yaml',run/'runtime/all-parameters.json',run/'runtime/pre-goal.json',run/'runtime/initialpose-request.yaml',run/'runtime/environment.json',run/'repo-at-start.json',run/'session.bash']:
  artifacts.append({'id':'runtime_parameters' if p.name=='controller_server.yaml' else 'frozen_'+p.stem.replace('-','_'),'path':str(p.relative_to(run)),'sha256':sha(p),'kind':'runtime_parameters' if p.name=='controller_server.yaml' else 'other'})
 manifest={'schema_version':1,'run_id':run.name,'record_origin':'real','scenario':{'scenario_id':scen['scenario_id'],'scenario_revision':scen['scenario_revision'],'spec_sha256':sha(run/'artifacts/scenario.yaml')},'source':{'git_commit':head,'dirty':bool(status),'snapshot_ref':'source_at_start' if status else None},'baseline':{'method':method,'config_sha256':sha(run/'artifacts/nav2.yaml'),'expected_plugin':plugin,'common_constraints_sha256':sha(run/'artifacts/common_constraints.json')},'environment':{'ros_distro':'jazzy','platform':platform.platform(),'package_versions':spec['package_versions'],'external_seed':None,'seed_scope':[],'controller_seed_controlled':False},'clock':{'clock_epoch':run.name+':sim0','started_at_utc':json.loads((run/'repo-at-start.json').read_text())['started_at_utc'],'sim_start_s':None},'artifacts':artifacts}
 put(run/'manifest.json',manifest)
 short('scenario-check',['python3','-B',str(root/'tools/validate_run.py'),'scenario',str(run/'artifacts/scenario.yaml')])
 short('pre-goal-run-check',['python3','-B',str(root/'tools/validate_run.py'),'run',str(run)],allowed=(3,))
try:
 gazebo=spawn('gazebo',['gz','sim','-s','-r','-v','3','--render-engine-server','ogre',str(run/'artifacts/world-ogre1.sdf')])
 short('spawn',['ros2','run','ros_gz_sim','create','-world','default','-file',str(run/'artifacts/robot.sdf'),'-name','turtlebot3_waffle','-x','-2.0','-y','-0.5','-z','0.01'],45)
 s.run();ready=True
 observer=spawn('observer',['python3','-B',str(run/'artifacts/observe.py'),'--output',str(run/'runtime/observations.jsonl'),'--seconds','240'])
 spin(3)
 params=snapshot();lifecycle=states()
 assert all(v['id']==3 for v in lifecycle.values()),lifecycle
 actual=params['controller_server']['FollowPath.plugin'];assert actual==plugin,actual
 short('runtime-plugin',['ros2','param','get','/controller_server','FollowPath.plugin'],15)
 endpoints={}
 for topic in ['/clock','/scan','/odom','/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel']:
  endpoints[topic]={}
  for key,fn in [('publishers',s.node.get_publishers_info_by_topic),('subscribers',s.node.get_subscriptions_info_by_topic)]:
   endpoints[topic][key]=[{'node':v.node_name,'namespace':v.node_namespace,'type':v.topic_type} for v in fn(topic)]
 client=ActionClient(s.node,NavigateToPose,'/navigate_to_pose');assert client.wait_for_server(timeout_sec=5)
 t=s.tf.lookup_transform('map','base_link',Time());p=t.transform.translation;q=t.transform.rotation;ay=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 pose={'x':p.x,'y':p.y,'yaw':ay,'stamp_s':t.header.stamp.sec+t.header.stamp.nanosec*1e-9,'ros_s':s.clock}
 assert abs(pose['stamp_s']-s.clock)<=.5
 # Query simulator state independently for a preflight localization sanity check only.
 raw=short('world-state-before',['gz','service','-s','/world/default/generate_world_sdf','--reqtype','gz.msgs.SdfGeneratorConfig','--reptype','gz.msgs.StringMsg','--timeout','5000','--req',''],10)
 import xml.etree.ElementTree as ET
 world=ET.fromstring(ast.literal_eval(raw.split('data: ',1)[1].strip()));physical=list(map(float,world.find("./world/model[@name='turtlebot3_waffle']/pose").text.split()))
 assert math.hypot(p.x-physical[0],p.y-physical[1])<.15
 assert abs(math.atan2(math.sin(ay-physical[5]),math.cos(ay-physical[5])))<.15
 goal=json.loads((run/'artifacts/goal.yaml').read_text())['pose']['pose']['position'];gx,gy=goal['x'],goal['y']
 radius=params['local_costmap/local_costmap']['robot_radius']+params['local_costmap/local_costmap']['footprint_padding']
 costs={}
 for kind in ['local','global']:
  cm=call(GetCostmap,f'/{kind}_costmap/get_costmap',GetCostmap.Request()).map
  with gzip.open(run/'runtime'/f'{kind}-costmap.yaml.gz','xt') as f:f.write(message_to_yaml(cm))
  md=cm.metadata;ox,oy=md.origin.position.x,md.origin.position.y;res=md.resolution
  assert abs(md.origin.orientation.z)<1e-6
  tr=s.tf.lookup_transform(cm.header.frame_id,'map',Time()).transform if cm.header.frame_id!='map' else None
  cy=math.atan2(2*(tr.rotation.w*tr.rotation.z+tr.rotation.x*tr.rotation.y),1-2*(tr.rotation.y**2+tr.rotation.z**2)) if tr else 0
  worst=0
  for i in range(46):
   x=p.x+(gx-p.x)*i/45;y=p.y+(gy-p.y)*i/45
   if tr:x,y=tr.translation.x+math.cos(cy)*x-math.sin(cy)*y,tr.translation.y+math.sin(cy)*x+math.cos(cy)*y
   for iy in range(math.floor((y-radius-oy)/res),math.ceil((y+radius-oy)/res)+1):
    for ix in range(math.floor((x-radius-ox)/res),math.ceil((x+radius-ox)/res)+1):
     if math.hypot(ox+(ix+.5)*res-x,oy+(iy+.5)*res-y)>radius+.71*res:continue
     assert 0<=ix<md.size_x and 0<=iy<md.size_y,'outside costmap'
     worst=max(worst,int(cm.data[iy*md.size_x+ix]))
  costs[kind]={'max_swept_disk_cost':worst,'stamp_s':cm.header.stamp.sec+cm.header.stamp.nanosec*1e-9,'frame':cm.header.frame_id};assert worst<253,costs
 pre={'lifecycle':lifecycle,'actual_plugin':actual,'pose_estimate':pose,'simulator_pose_preflight_only':physical,'swept_costs':costs,'endpoints':endpoints,'action_server_ready':True,'monotonic_s':time.monotonic(),'sim_s':s.clock}
 put(run/'runtime/pre-goal.json',pre);freeze_manifest()
 # Fresh gate immediately before the single dispatch, after metadata work.
 gate=states();assert all(v['id']==3 for v in gate.values())
 assert all(time.monotonic()-s.last.get(k,0)<1.0 for k in ['scan','odom','clock'])
 put(run/'runtime/dispatch-readiness.json',{'states':gate,'input_receive_age_wall_s':{k:time.monotonic()-s.last[k] for k in ['scan','odom','clock']},'sim_s':s.clock,'monotonic_s':time.monotonic(),'action_ready':client.server_is_ready()})
 assert observer.poll() is None
 cli=spawn('goal-client',['python3','-B',str(run/'artifacts/send_one_goal.py')]);deadline=time.monotonic()+170
 while cli.poll() is None and time.monotonic()<deadline:rclpy.spin_once(s.node,timeout_sec=.05)
 assert cli.poll()==0,('goal client',cli.poll())
 terminal=json.loads((run/'runtime/goal-terminal.json').read_text());emit('goal_terminal',terminal)
 end_sim=s.clock+8;deadline=time.monotonic()+30
 while s.clock<end_sim and time.monotonic()<deadline:rclpy.spin_once(s.node,timeout_sec=.03)
 assert s.clock>=end_sim,'post-goal clock did not advance'
 if observer.poll() is None:observer.send_signal(signal.SIGINT);observer.wait(timeout=10)
 emit('capture_complete',{'sim_s':s.clock,'observer_exit':observer.returncode})
except BaseException as e:
 failure={'type':type(e).__name__,'message':str(e),'traceback':traceback.format_exc()};put(run/'runtime/session-failure.json',failure);emit('session_failure',failure)
finally:
 # Lifecycle shutdown preserves localization/TF until navigation is closed.
 if observer and observer.poll() is None:
  observer.send_signal(signal.SIGINT)
  try:observer.wait(timeout=10)
  except subprocess.TimeoutExpired:observer.terminate();observer.wait(timeout=5)
 for label,proc in reversed(s.children):
  item={'label':label,'pid':proc.pid,'forced':False,'shutdown_service':None}
  if label in s.managers and proc.poll() is None:
   c=s.node.create_client(ManageLifecycleNodes,f'/lifecycle_manager_{label}/manage_nodes');req=ManageLifecycleNodes.Request();req.command=4
   response=s.call(c,req,15);item['shutdown_service']=response.success if response else None
  if proc.poll() is None:
   proc.send_signal(signal.SIGINT)
   try:proc.wait(timeout=20)
   except subprocess.TimeoutExpired:
    item['forced']=True;os.killpg(proc.pid,signal.SIGTERM)
    try:proc.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=5)
  item['exit']=proc.returncode;cleanup.append(item);emit('cleanup_process',item)
 for label,proc,ticks in reversed(owned):
  if label!='gazebo':continue
  item={'label':label,'pid':proc.pid,'forced':False}
  if proc.poll() is None:
   # Previous signal-only shutdown returned SIGSEGV. Request the existing
   # server's normal stop service first; retain any abnormal exit honestly.
   try:
    out=subprocess.run(['gz','service','-s','/server_control','--reqtype','gz.msgs.ServerControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','stop: true'],env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=8)
    put(run/'logs/gazebo-stop-service.out',out.stdout);put(run/'logs/gazebo-stop-service.err',out.stderr);put(run/'logs/gazebo-stop-service.exit',str(out.returncode)+'\n');item['stop_service_exit']=out.returncode;item['stop_service_response']=out.stdout
   except Exception as e:item['stop_service_error']=str(e)
   try:proc.wait(timeout=15)
   except subprocess.TimeoutExpired:
    item['signal_fallback']=True;proc.send_signal(signal.SIGINT)
    try:proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
     item['forced']=True;os.killpg(proc.pid,signal.SIGTERM)
     try:proc.wait(timeout=5)
     except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=5)
  item['exit']=proc.returncode;cleanup.append(item);emit('cleanup_process',item)
 spin(.5)
 after=s.node.get_node_names();s.node.destroy_node();rclpy.shutdown()
 remaining=[]
 groups={proc.pid for _,proc,_ in owned}
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   if p.stat().st_uid!=os.getuid():continue
   if os.getpgid(int(p.name)) in groups:remaining.append({'pid':int(p.name),'cmd':(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')})
  except (PermissionError,ProcessLookupError,FileNotFoundError):pass
 put(run/'runtime/cleanup-evidence.json',{'processes':cleanup,'remaining_owned_groups':remaining,'nodes_after':after,'monotonic_s':time.monotonic(),'scope':'Owned navigation then localization, bridge/RSP, then Gazebo. No global kill.'})
 put(run/'runtime/session-ended.json',{'utc':datetime.now(timezone.utc).isoformat(),'monotonic_s':time.monotonic(),'elapsed_monotonic_s':time.monotonic()-start,'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'same_boot':boot==Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'scope':'Whole commissioning session including startup and cleanup; not controller performance.'})
 for f in files:f.close()
 events.close()
 print('SESSION_CLOSED',run,flush=True)
 if failure:sys.exit(1)
