"""Read-only independent recomputation and raw evidence entrances for one closed run."""
from pathlib import Path
import argparse,json,sys,math,hashlib,collections,bisect
import numpy as np
ROOT=Path('/home/robot/commercial_mobile_robot');sys.path.insert(0,str(ROOT/'tools'))
import validate_run as v
from inspect_dynamic_recording import raw_rows,actual_poses
from runner_analysis import analyze
from dynamic_geometry import PoseSeries,rotate
from dynamic_contract import fingerprint
def h(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def review(run):
 run=Path(run);contract=v.validate_run_dir(run)
 if contract['status']!='VALID':raise ValueError(contract)
 profile=v.load_strict(run/'artifacts/profile.json');spec=profile['dynamic_spec'];d=spec['dynamic'];manifest=v.load_strict(run/'manifest.json');result=v.load_strict(run/'result.json');exitinfo=v.load_strict(run/'runtime/runner-exit.json');cleanup=v.load_strict(run/'runtime/cleanup.json');dynamic=v.load_strict(run/'runtime/dynamic-report.json');bag=v.load_strict(run/'runtime/bag-check.json');ready=v.load_strict(run/'runtime/dynamic-readiness.json')
 raw=raw_rows(run/'runtime/gazebo-raw.jsonl');mapping,samples=actual_poses(raw,run.name,run.name+':sim0');commands=raw_rows(run/'runtime/motion-commands.jsonl');actions=raw_rows(run/'logs/action-client.jsonl');obs=raw_rows(run/'runtime/observations.jsonl')
 out={'run':str(run),'record_contract':contract,'purpose':profile['mode'],'result_status':result['status'],'runner_exit':exitinfo,'actual_plugin':result['runtime']['actual_plugin'],'dynamic_valid':dynamic['dynamic_recording_valid'],'probe_pass':dynamic['probe_pass'],'bag_valid':bag['valid'],'cleanup_ok':cleanup['ok'],'remaining':cleanup['remaining'],'forced_processes':[x['role'] for x in cleanup['processes'] if x['forced']],'input_fingerprint':fingerprint(spec),'source_goal':v.load_strict(run/'artifacts/goal.json'),'actual_entity_mapping':mapping,'clock_alignment':dynamic.get('clock_alignment'),'collision':result['evaluation']['collision'],'truth_leak_detected':ready['input_isolation']['truth_leak_detected'],'raw_sources':{n:{'path':str(run/n),'sha256':h(run/n)} for n in ['logs/action-client.jsonl','logs/events.jsonl','runtime/observations.jsonl','runtime/gazebo-raw.jsonl','runtime/motion-commands.jsonl','bag/metadata.yaml']},'human_visual_confirmation':None,'human_understanding_confirmation':None}
 actor=samples['scripted_obstacle'];robot=samples['turtlebot3_waffle'];motion=v.load_strict(run/'artifacts/dynamic_motion.json');arm=next(x for x in commands if x['kind']=='arm');release=arm['release_ns'];end=release+motion['knots'][-1]['t_ns']+motion['hold_ns'];seq=[x for x in actor if release<=x['t_ns']<=end]
 times=np.array([x['t_ns']-release for x in seq],dtype=np.float64);kt=np.array([x['t_ns'] for x in motion['knots']],dtype=np.float64);kp=np.array([x['xyz'] for x in motion['knots']]);wanted=np.column_stack([np.interp(times,kt,kp[:,j]) for j in range(3)]);actual=np.array([x['xyz'] for x in seq]);errors=np.linalg.norm(actual-wanted,axis=1)
 out['independent_trajectory']={'method':'NumPy piecewise interpolation of desired knots at actual source times; no shift of time axis','maximum_m':float(errors.max()),'p95_m':float(np.quantile(errors,.95,method='higher')),'rms_m':float(np.sqrt(np.mean(errors**2))),'sample_count':len(seq),'first_last_raw_lines':[seq[0]['raw_line'],seq[-1]['raw_line']],'motion_source_window_ns':[release,end]}
 assert abs(out['independent_trajectory']['maximum_m']-dynamic['trajectory']['maximum_m'])<1e-9
 assert out['independent_trajectory']['maximum_m']<=d['position_tolerance_m']
 initial=motion['knots'][0]['xyz'];first=next((x for x in actor if x['t_ns']>=release and math.dist(x['xyz'],initial)>1e-6),None);out['first_actual_actor_motion']={'source_ns':first['t_ns'],'raw_line':first['raw_line'],'after_release_s':(first['t_ns']-release)*1e-9} if first else None
 out['trigger']={'raw_command_line':arm['_line'],'event':arm['event'],'release_ns':release}
 out['actor_yaw_evidence']='raw upright quaternion; trajectory is translational, desired yaw zero'
 if profile['mode']=='navigation_recording':
  params=v.load_strict(run/'runtime/all-parameters.json')['controller_server'];behavior=analyze(obs,actions,v.load_strict(run/'artifacts/goal.json'),params)
  out['independent_behavior']=behavior
  accepted=[x for x in actions if x['kind']=='response' and x['data']['accepted']];terminal=[x for x in actions if x['kind']=='terminal'];assert len(accepted)==len(terminal)==1 and accepted[0]['data']['uuid']==terminal[0]['data']['uuid']
  out['action_evidence']={'accepted_line':accepted[0]['line'],'accepted_sim_s':accepted[0]['ros_s'],'terminal_line':terminal[0]['line'],'terminal_sim_s':terminal[0]['ros_s'],'uuid':accepted[0]['data']['uuid'],'terminal':terminal[0]['data']['action']}
  moving=[x for x in robot if round(accepted[0]['ros_s']*1e9)<=x['t_ns']<=round(terminal[0]['ros_s']*1e9)]
  out['actual_robot_world_motion']={'maximum_displacement_m':max(math.dist(moving[0]['xyz'][:2],x['xyz'][:2]) for x in moving),'raw_line_range':[moving[0]['raw_line'],moving[-1]['raw_line']],'scope':'independent Gazebo top-model state, separate from wheel odom and AMCL'}
  out['goal_checker']={k:params[k] for k in ['general_goal_checker.xy_goal_tolerance','general_goal_checker.yaw_goal_tolerance']}
  out['commands_nonzero_during_or_after_stop_window']={topic:sum(abs(x['data']['linear'][0])>.001 or abs(x['data']['angular'][2])>.001 for x in obs if x['kind']==topic and x['ros_now_s']>=behavior['stop']['start_sim_s']) for topic in ['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel']}
  assert all(behavior['checks'].values()) and terminal[0]['data']['action']=='SUCCEEDED'
 else:
  out['actual_stationary_robot_xy_drift_m']=max(math.dist(robot[0]['xyz'][:2],x['xyz'][:2]) for x in robot);assert out['actual_stationary_robot_xy_drift_m']<=.02
  assert not actions
 stages=[]
 for frame in dynamic['frames']:
  if not stages or stages[-1]['label']!=frame['label']:stages.append({'label':frame['label'],'frames':[frame]})
  else:stages[-1]['frames'].append(frame)
 out['observation_stages']=[];out['example_frames']=[];geometry=v.load_strict(run/'artifacts/dynamic_geometry.json');series={name:PoseSeries(seq,run.name+':sim0',mapping[name],d['truth_gap_ns']) for name,seq in samples.items()}
 for stage in stages:
  frames=stage['frames'];span=(frames[-1]['t_ns']-frames[0]['t_ns'])*1e-9;qualified=len(frames)>=5 and span>=1. and all(0<b['t_ns']-a['t_ns']<=300000000 for a,b in zip(frames,frames[1:]));out['observation_stages'].append({'label':stage['label'],'frames':len(frames),'start_ns':frames[0]['t_ns'],'end_ns':frames[-1]['t_ns'],'span_s':span,'qualified':qualified,'bag_message_index_range':[frames[0]['bag_message_index'],frames[-1]['bag_message_index']]})
  if qualified and stage['label'] in ['visible','occluded']:
   frame=frames[len(frames)//2];t=frame['t_ns'];brackets={}
   for name,ss in samples.items():
    idx=bisect.bisect_left([x['t_ns'] for x in ss],t);indices=[idx] if idx<len(ss) and ss[idx]['t_ns']==t else [idx-1,idx];brackets[name]=[{'line':ss[i]['raw_line'],'source_ns':ss[i]['t_ns'],'xyz':ss[i]['xyz'],'quaternion_xyzw':ss[i]['quaternion_xyzw']} for i in indices]
   xyz=series['turtlebot3_waffle'].xyz_at(t);q=series['turtlebot3_waffle'].quat_at(t);offset=rotate(q,geometry['laser_extrinsic']['xyz']);out['example_frames'].append({**frame,'source_pose_brackets':brackets,'laser_world_origin':[xyz[i]+offset[i] for i in range(3)],'actual_robot_xyyaw':series['turtlebot3_waffle'].at(t),'actual_actor_xyz':series['scripted_obstacle'].xyz_at(t)})
 out['crossing']=dynamic.get('crossing');out['unknown_frames']=sum(x['label'] is None for x in dynamic['frames']);out['partial_frames']=sum(x['label']=='partial' for x in dynamic['frames'])
 assert exitinfo['exit']==0 and cleanup['ok'] and not cleanup['remaining'] and not out['forced_processes'] and bag['valid'] and not out['truth_leak_detected'] and out['collision'] is None
 return out
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--output',required=True);a=p.parse_args();r=review(a.run);Path(a.output).open('x').write(json.dumps(r,indent=2)+'\n');print(json.dumps({k:r[k] for k in ['run','record_contract','dynamic_valid','probe_pass','bag_valid','cleanup_ok','collision']}))
