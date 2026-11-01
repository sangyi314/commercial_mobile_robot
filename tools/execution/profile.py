# SPDX-License-Identifier: MIT
"""Derived routing contract; validates unchanged P0 parent before any run."""
import copy,json,re,xml.etree.ElementTree as ET
from pathlib import Path
import yaml
from tools.runner.runner_core import digest,write_json
CASES={'G01':('mppi',None),'G02':('dwb',None),'G03':('mppi','revoke'),'G04':('mppi','owner-loss'),'G05':('mppi','candidate-stop'),'G06':('mppi','clock-pause'),'G07':('mppi','owner-loss-load')}
NODES=['controller_server','smoother_server','planner_server','route_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother','collision_monitor']
def derive_config(base,method,ns):
 c=copy.deepcopy(base)
 for n in ['controller_server','behavior_server','velocity_smoother','collision_monitor']:c[n]['ros__parameters']['enable_stamped_cmd_vel']=True
 c['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance']=.18
 c['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance']=.20
 fp=c['controller_server']['ros__parameters']['FollowPath']
 if method=='mppi':fp.update(vx_max=.18,wz_max=.8)
 else:fp.update(max_vel_x=.18,max_speed_xy=.18,max_vel_theta=.8,xy_goal_tolerance=.18)
 c['behavior_server']['ros__parameters']['max_rotational_vel']=.8
 c['velocity_smoother']['ros__parameters']['max_velocity']=[.18,0.,.8]
 c['velocity_smoother']['ros__parameters']['min_velocity']=[-.1,0.,-.8]
 c['collision_monitor']['ros__parameters']['cmd_vel_out_topic']=ns+'/candidate'
 c.pop('docking_server',None)
 return c
def bridge(ns):
 b=yaml.safe_load(Path('/opt/ros/jazzy/share/nav2_minimal_tb3_sim/configs/turtlebot3_waffle_bridge.yaml').read_text());out=[]
 for v in b:
  t='/'+v['topic_name'].lstrip('/')
  if t=='/cmd_vel':continue
  out.append({**{k:x for k,x in v.items() if k!='topic_name'},'ros_topic_name':t,'gz_topic_name':t,'subscriber_queue':1,'publisher_queue':1})
 out.append({'ros_topic_name':ns+'/final','gz_topic_name':ns+'/drive','ros_type_name':'geometry_msgs/msg/Twist','gz_type_name':'gz.msgs.Twist','direction':'ROS_TO_GZ','subscriber_queue':1,'publisher_queue':1})
 return out
def validate(s,p,method,legacy,identity):
 e=p['execution_v1'];base,scene=legacy(e['base_scenario'],e['base_profile'],method,identity)
 if p['profile_version']!=3 or p['scope']!='simulation_only' or e['case'] not in CASES or CASES[e['case']]!=(method,e['fault']):raise ValueError('execution case identity')
 if e['run_id']!=identity or e['namespace']!='/cmr_execution/'+identity or not re.fullmatch('[a-z][a-z0-9_]{1,70}',identity):raise ValueError('execution namespace')
 if s['task']!={**scene['task'],'common_constraints_ref':s['task']['common_constraints_ref']} or s['dynamic_events']:raise ValueError('unreviewed physical task')
 ns=e['namespace'];original=ET.parse(base['files']['robot']['path']);robot=ET.parse(p['files']['robot']['path'])
 original.find('.//plugin[@name="gz::sim::systems::DiffDrive"]/topic').text=ns+'/drive'
 if ET.tostring(original.getroot())!=ET.tostring(robot.getroot()):raise ValueError('robot change beyond DiffDrive routing')
 world=ET.parse(base['files']['world']['path']);w=world.find('world')
 if not w.findall('plugin[@name="gz::sim::systems::SceneBroadcaster"]'):ET.SubElement(w,'plugin',{'filename':'gz-sim-scene-broadcaster-system','name':'gz::sim::systems::SceneBroadcaster'})
 if ET.tostring(world.getroot())!=ET.tostring(ET.parse(p['files']['world']['path']).getroot()):raise ValueError('world change beyond read-only SceneBroadcaster')
 for m in ['mppi','dwb']:
  expected=derive_config(yaml.safe_load(Path(base['methods'][m]['config']).read_text()),m,ns)
  if yaml.safe_load(Path(p['methods'][m]['config']).read_text())!=expected:raise ValueError('config differs from reviewed derived settings')
 for k,v in p['files'].items():
  if digest(v['path'])!=v['sha256']:raise ValueError('derived resource identity '+k)
 for v in p['methods'].values():
  if digest(v['config'])!=v['sha256']:raise ValueError('derived config identity')
 for role,v in e['binaries'].items():
  if digest(v['path'])!=v['sha256']:raise ValueError('binary changed '+role)
 if yaml.safe_load(Path(e['bridge']).read_text())!=bridge(ns):raise ValueError('bridge bypass/unreviewed configuration')
 if p['launch']!={**base['launch'],'navigation':'navigation_gate.launch.py','navigation_package':'cmr_execution'}:raise ValueError('derived launch')
 if e.get('capture_scope')!='robot_model_all_samples+world_stats':raise ValueError('unreviewed observation scope')
 if e.get('arrival_limits')!={'position_m':.25,'yaw_rad':.25} or e.get('control_profile_version')!='p103-routing-v4-direct-cm':raise ValueError('unreviewed arrival acceptance')
 if p['environment']!=base['environment'] or p['budgets']!=base['budgets']:raise ValueError('unreviewed environment/budget')
 return p,s
def prepare(base_path,prefix,out,suffix):
 from tools.runner.trial import load_inputs
 base=yaml.safe_load(Path(base_path).read_text());scenario=str(Path(base['pair'])/'inputs/scenario-template.yaml')
 load_inputs(scenario,base_path,'mppi','p103_preflight');out=Path(out);out.mkdir(parents=True,exist_ok=False);plans=[]
 for case,(method,fault) in CASES.items():
  rid='p103_20261101_'+case.lower()+'_'+suffix;ns='/cmr_execution/'+rid;d=out/case;d.mkdir();p=copy.deepcopy(base)
  world=ET.parse(base['files']['world']['path']);w=world.find('world')
  if not w.findall('plugin[@name="gz::sim::systems::SceneBroadcaster"]'):ET.SubElement(w,'plugin',{'filename':'gz-sim-scene-broadcaster-system','name':'gz::sim::systems::SceneBroadcaster'})
  world.write(d/'world.sdf',encoding='utf-8',xml_declaration=True)
  robot=ET.parse(base['files']['robot']['path']);robot.find('.//plugin[@name="gz::sim::systems::DiffDrive"]/topic').text=ns+'/drive';robot.write(d/'robot.sdf',encoding='utf-8',xml_declaration=True)
  for k in ['world','robot']:p['files'][k]={'path':str(d/(k+'.sdf')),'sha256':digest(d/(k+'.sdf'))}
  for m in ['mppi','dwb']:
   (d/(m+'.yaml')).write_text(yaml.safe_dump(derive_config(yaml.safe_load(Path(base['methods'][m]['config']).read_text()),m,ns)));p['methods'][m].update(config=str(d/(m+'.yaml')),sha256=digest(d/(m+'.yaml')))
  constraints=json.loads(Path(base['files']['constraints']['path']).read_text());constraints.update(scope='P1-03 conservative controlled gate trial; not P0 comparison',semantic_limits={**constraints['semantic_limits'],'vx_m_s':[-.1,.18],'abs_wz_rad_s':.8});constraints['common_explicit_config']={k:v for k,v in yaml.safe_load((d/'mppi.yaml').read_text()).items() if k!='controller_server'}
  write_json(d/'constraints.json',constraints);p['files']['constraints']={'path':str(d/'constraints.json'),'sha256':digest(d/'constraints.json')}
  write_json(d/'bridge.yaml',bridge(ns));p['launch'].update(navigation='navigation_gate.launch.py',navigation_package='cmr_execution');p['profile_version']=3;p['faults']=[]
  bins={n:{'path':str(Path(prefix)/'cmr_execution/lib/cmr_execution'/n),'sha256':digest(Path(prefix)/'cmr_execution/lib/cmr_execution'/n)} for n in ['execution_gate','execution_observer','candidate_relay']}
  p['execution_v1']={'case':case,'run_id':rid,'namespace':ns,'owner_epoch':rid+'_fixture','goal_owner':'original Runner cmr_worker','lease_owner':'explicit authorization fixture; no Mission/Action client','fault':fault,'base_profile':str(Path(base_path).resolve()),'base_scenario':scenario,'bridge':str(d/'bridge.yaml'),'binaries':bins,'nodes':NODES,'synthetic':False,'arrival_limits':{'position_m':.25,'yaw_rad':.25},'control_profile_version':'p103-routing-v4-direct-cm','capture_scope':'robot_model_all_samples+world_stats'}
  p['recording']['topics']=[t for t in base['recording']['topics'] if t!='/cmd_vel']+[ns+'/'+t for t in ['candidate','final','state','stop_report']]
  s=yaml.safe_load(Path(scenario).read_text());s['resources']['world']={**s['resources']['world'],**p['files']['world']};s['task']['common_constraints_ref']={**s['task']['common_constraints_ref'],**p['files']['constraints']};write_json(d/'scenario.yaml',s);write_json(d/'profile.yaml',p)
  plans.append({'case':case,'method':method,'profile':str(d/'profile.yaml'),'scenario':str(d/'scenario.yaml'),'run_id':rid})
 write_json(out/'plan.json',plans);return plans
