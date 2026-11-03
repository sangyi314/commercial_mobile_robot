# SPDX-License-Identifier: MIT
"""Reviewed P1-only deltas. --check uses validate, never prepare."""
import copy,json,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
import yaml
from tools.runner.runner_core import digest,write_json
from tools.execution.profile import prepare as prepare_execution,NODES
CASES={'G01':('mppi','human'),'G02':('dwb','synthetic'),'G03':('mppi','cancel'),'G04':('mppi','timeout'),'G05':('mppi','fifo'),'G06':('mppi','owner-exit'),'G01D':('mppi','synthetic')}
BT='''<root BTCPP_format="4" main_tree_to_execute="MainTree">
<BehaviorTree ID="MainTree"><RecoveryNode number_of_retries="1" name="DeliveryRecovery">
<PipelineSequence name="NavigateWithReplanning"><RateController hz="1.0"><ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" error_code_id="{compute_path_error_code}"/></RateController><FollowPath path="{path}" controller_id="FollowPath" error_code_id="{follow_path_error_code}"/></PipelineSequence>
<Sequence name="OneBoundedRecovery"><ClearEntireCostmap name="ClearLocalOnce" service_name="local_costmap/clear_entirely_local_costmap"/><ClearEntireCostmap name="ClearGlobalOnce" service_name="global_costmap/clear_entirely_global_costmap"/><Wait wait_duration="2.0"/></Sequence>
</RecoveryNode></BehaviorTree></root>
'''
def body_feedback(ns):
 return {'version':1,'source':'gz-sim8-odometry-publisher-worldPose','gz_sim_commit':'446a44335a45b704b4d36dabcc5508ee34eeb3d8','topic':ns+'/body_odom','frame':'body_world','child_frame':'body_base','frequency_hz':50,'required':True,'navigation_input':False,'oracle_stop_assisted':True,'sensor_only_E2':False}
def body_robot(path,ns):
 tree=ET.parse(path);model=tree.find('model')
 if model is None or model.findall('plugin[@name="gz::sim::systems::OdometryPublisher"]'):raise ValueError('body observer source must be singular')
 plugin=ET.SubElement(model,'plugin',{'filename':'gz-sim-odometry-publisher-system','name':'gz::sim::systems::OdometryPublisher'})
 for key,value in {'dimensions':'2','odom_publish_frequency':'50','odom_frame':'body_world','robot_base_frame':'body_base','gaussian_noise':'0','odom_topic':ns+'/body_odom','odom_covariance_topic':ns+'/body_covariance','tf_topic':ns+'/body_tf_unused'}.items():ET.SubElement(plugin,key).text=value
 return tree

def body_bridge(base,ns):
 return yaml.safe_load(Path(base).read_text())+[{'ros_topic_name':ns+'/body_odom','gz_topic_name':ns+'/body_odom','ros_type_name':'nav_msgs/msg/Odometry','gz_type_name':'gz.msgs.Odometry','direction':'GZ_TO_ROS','subscriber_queue':1,'publisher_queue':1}]

def configs(base,bt):
 c=copy.deepcopy(base);n=c['bt_navigator']['ros__parameters'];n['default_nav_to_pose_bt_xml']=bt;n['navigators']=['navigate_to_pose'];c.pop('waypoint_follower',None)
 fp=c['controller_server']['ros__parameters']['FollowPath']
 if fp['plugin']=='nav2_mppi_controller::MPPIController':
  fp.update(plugin='nav2_rotation_shim_controller::RotationShimController',primary_controller='nav2_mppi_controller::MPPIController',rotate_to_goal_heading=True,rotate_to_heading_once=True,rotate_to_heading_angular_vel=.8,max_angular_accel=2.)
  fp.setdefault('GoalAngleCritic',{})['enabled']=False
  fp.setdefault('GoalCritic',{})['cost_weight']=20.
 return c

def validate(scene,p,method,legacy,identity):
 d=p['delivery_v1']
 if set(d)!={'version','case','run_id','synthetic_observations','handover_policy','base_profile','base_scenario','catalog','catalog_source','bt','binaries','policy','stop_feedback','controller_profile'}:raise ValueError('unknown delivery field')
 if p['profile_version']!=4 or d['version']!=1 or d['case'] not in CASES or CASES[d['case']][0]!=method or d['run_id']!=identity or d['synthetic_observations'] is not False:raise ValueError('delivery profile identity/layer')
 for ref in [d['base_profile'],d['base_scenario'],d['catalog'],d['catalog_source'],d['bt'],*d['binaries'].values()]:
  if digest(ref['path'])!=ref['sha256']:raise ValueError('delivery resource changed '+ref['path'])
 base,oldscene=legacy(d['base_scenario']['path'],d['base_profile']['path'],method,identity)
 expected=copy.deepcopy(base);expected['profile_version']=4;expected['delivery_v1']=d;expected['launch'].update(navigation='navigation_delivery.launch.py',navigation_package='cmr_mission');expected['execution_v1'].update(nodes=[n for n in NODES if n!='waypoint_follower'],owner_epoch=identity+'_mission',goal_owner='Mission NavigationAdapter only',lease_owner='real Mission ExecutionPort; no fixture')
 ns=p['execution_v1']['namespace']
 if d['stop_feedback']!=body_feedback(ns):raise ValueError('independent body feedback policy')
 if d['controller_profile']!='p104-mppi-short-leg-v4':raise ValueError('unreviewed terminal heading policy')
 if ET.tostring(body_robot(base['files']['robot']['path'],ns).getroot())!=ET.tostring(ET.parse(p['files']['robot']['path']).getroot()):raise ValueError('robot change beyond independent read-only body observer')
 if digest(p['files']['robot']['path'])!=p['files']['robot']['sha256']:raise ValueError('body robot hash')
 if yaml.safe_load(Path(p['execution_v1']['bridge']).read_text())!=body_bridge(base['execution_v1']['bridge'],ns):raise ValueError('body bridge route')
 expected['files']['robot']=p['files']['robot'];expected['execution_v1']['bridge']=p['execution_v1']['bridge'];expected['recording']['topics']=base['recording']['topics']+[ns+'/body_odom']
 for key in ['mppi','dwb']:
  if yaml.safe_load(Path(p['methods'][key]['config']).read_text())!=configs(yaml.safe_load(Path(base['methods'][key]['config']).read_text()),d['bt']['path']):raise ValueError('unreviewed delivery Nav2 parameters')
  expected['methods'][key].update(config=p['methods'][key]['config'],sha256=digest(p['methods'][key]['config']),plugin='nav2_rotation_shim_controller::RotationShimController' if key=='mppi' else base['methods'][key]['plugin'])
 expected['budgets']={**base['budgets'],'work':660,'startup':180,'cleanup':45,'finalize':45}
 if p!=expected or scene!=oldscene:raise ValueError('delivery delta beyond reviewed fields')
 cat=json.loads(Path(d['catalog']['path']).read_text());original=json.loads(Path(d['catalog_source']['path']).read_text());original['bt']={'path':d['bt']['path'],'sha256':d['bt']['sha256']}
 if cat!=original:raise ValueError('station snapshot differs from reviewed source')
 m=cat['map']
 if digest(m['yaml_path'])!=m['yaml_sha256'] or digest(m['image_path'])!=m['image_sha256'] or m['yaml_sha256']!=base['files']['map_yaml']['sha256'] or m['image_sha256']!=base['files']['map_image']['sha256']:raise ValueError('actual map identity')
 if Path(d['bt']['path']).read_text()!=BT:raise ValueError('unreviewed recovery BT')
 if d['policy']!={'task_ms':240000,'handover_ms':1500 if d['case']=='G04' else 120000,'queue_capacity':4,'shared_retry_limit':1,'deadline_reset':False}:raise ValueError('delivery policy')
 if d['handover_policy']!=CASES[d['case']][1]:raise ValueError('handover/fault scope')
 return p,scene

def prepare(base_profile,prefix,output,suffix,run_prefix='p104_20261103'):
 out=Path(output).resolve();out.mkdir(parents=True,exist_ok=False);prefix=Path(prefix).resolve();base_profile=Path(base_profile).resolve();templates=prepare_execution(str(base_profile),str(prefix),str(out/'execution-templates'),suffix)
 meta=out/'map-identity.json';loader=prefix/'cmr_mission/lib/cmr_mission/map_identity';base=yaml.safe_load(base_profile.read_text())
 with (out/'map-identity.log').open('x') as log:subprocess.run([str(loader),'--map',base['files']['map_yaml']['path'],'--output',str(meta)],stdout=log,stderr=subprocess.STDOUT,check=True)
 metadata=json.loads(meta.read_text());plans=[]
 from tools import PROJECT_ROOT
 catalog_source=PROJECT_ROOT/'experiments/mission/p1-delivery/stations-v1.json';source_catalog=json.loads(catalog_source.read_text())
 if source_catalog['map']['metadata']!=metadata:raise ValueError('installed map loader differs from reviewed station metadata')
 for case,(method,handover) in CASES.items():
  rid=run_prefix+'_'+case.lower()+'_'+suffix;ns='/cmr_execution/'+rid;root=out/case;root.mkdir();p3root=root/'execution-base';p3root.mkdir();template=templates[1 if method=='dwb' else 0];p3=json.loads(Path(template['profile']).read_text());old=p3['execution_v1']['namespace']
  def replace(value):
   if isinstance(value,str):return value.replace(old,ns)
   if isinstance(value,dict):return {k:replace(v) for k,v in value.items()}
   if isinstance(value,list):return [replace(v) for v in value]
   return value
  p3=replace(p3);p3['execution_v1'].update(run_id=rid,namespace=ns,owner_epoch=rid+'_fixture')
  for k in ['world','robot','constraints']:
   src=Path(p3['files'][k]['path']);target=p3root/src.name;target.write_text(src.read_text().replace(old,ns));p3['files'][k]={'path':str(target),'sha256':digest(target)}
  for k in ['mppi','dwb']:
   target=p3root/(k+'.yaml');target.write_text(Path(p3['methods'][k]['config']).read_text().replace(old,ns));p3['methods'][k].update(config=str(target),sha256=digest(target))
  bridge=p3root/'bridge.yaml';bridge.write_text(Path(p3['execution_v1']['bridge']).read_text().replace(old,ns));p3['execution_v1']['bridge']=str(bridge)
  scene=json.loads(Path(template['scenario']).read_text());scene['resources']['world']={**scene['resources']['world'],**p3['files']['world']};scene['task']['common_constraints_ref']={**scene['task']['common_constraints_ref'],**p3['files']['constraints']};write_json(p3root/'scenario.json',scene);write_json(p3root/'profile.json',p3)
  bt=root/'delivery.bt.xml';bt.write_text(BT);source_goal=json.loads(Path(base['files']['goal']['path']).read_text());source_home=json.loads(Path(base['files']['initialpose']['path']).read_text())
  def pose(p):return {'frame':'map','position':p['position'],'orientation':p['orientation']}
  target=pose(source_goal['pose']['pose']);home=pose(source_home['pose']['pose']);middle=copy.deepcopy(home);middle['position']['x']=(target['position']['x']+home['position']['x'])/2;middle['position']['y']=(target['position']['y']+home['position']['y'])/2
  map_version='tb3map_'+base['files']['map_yaml']['sha256'][:12];station_version='p104_stations_v1';origin=metadata
  catalog=copy.deepcopy(source_catalog);catalog['bt']={'path':str(bt),'sha256':digest(bt)};write_json(root/'stations.v1.json',catalog)
  p4=copy.deepcopy(p3);p4['profile_version']=4;p4['launch'].update(navigation='navigation_delivery.launch.py',navigation_package='cmr_mission');p4['execution_v1'].update(nodes=[n for n in NODES if n!='waypoint_follower'],owner_epoch=rid+'_mission',goal_owner='Mission NavigationAdapter only',lease_owner='real Mission ExecutionPort; no fixture')
  for k in ['mppi','dwb']:
   config=root/(k+'.yaml');config.write_text(yaml.safe_dump(configs(yaml.safe_load(Path(p3['methods'][k]['config']).read_text()),str(bt))));p4['methods'][k].update(config=str(config),sha256=digest(config),plugin='nav2_rotation_shim_controller::RotationShimController' if k=='mppi' else p3['methods'][k]['plugin'])
  ref=lambda path:{'path':str(path),'sha256':digest(path)}
  p4['delivery_v1']={'version':1,'case':case,'run_id':rid,'synthetic_observations':False,'handover_policy':handover,'base_profile':ref(p3root/'profile.json'),'base_scenario':ref(p3root/'scenario.json'),'catalog':ref(root/'stations.v1.json'),'catalog_source':ref(catalog_source),'bt':ref(bt),'binaries':{n:ref(prefix/'cmr_mission/lib/cmr_mission'/n) for n in ['mission_node','map_identity']},'policy':{'task_ms':240000,'handover_ms':1500 if case=='G04' else 120000,'queue_capacity':4,'shared_retry_limit':1,'deadline_reset':False}}
  body=body_robot(p3['files']['robot']['path'],ns);bodypath=root/'body-robot.sdf';body.write(bodypath,encoding='utf-8',xml_declaration=True);p4['files']['robot']=ref(bodypath)
  bridgepath=root/'body-bridge.yaml';write_json(bridgepath,body_bridge(p3['execution_v1']['bridge'],ns));p4['execution_v1']['bridge']=str(bridgepath);p4['recording']['topics'].append(ns+'/body_odom');p4['delivery_v1']['stop_feedback']=body_feedback(ns);p4['delivery_v1']['controller_profile']='p104-mppi-short-leg-v4'
  p4['budgets'].update(work=660,startup=180,cleanup=45,finalize=45);write_json(root/'profile.json',p4);write_json(root/'scenario.json',scene);plans.append({'case':case,'method':method,'run_id':rid,'profile':str(root/'profile.json'),'scenario':str(root/'scenario.json')})
 write_json(out/'plan.json',plans);return plans
