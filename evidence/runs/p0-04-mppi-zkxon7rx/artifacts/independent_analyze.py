"""Offline independent recomputation from raw observations; no ROS imports."""
from pathlib import Path
import bisect,hashlib,json,math,sys
run=Path(sys.argv[1])
def read(name):return json.loads((run/name).read_text())
def finite(v):return type(v) in (int,float) and math.isfinite(v)
def vec(v,n):return isinstance(v,list) and len(v)==n and all(finite(x) for x in v)
def norm(v):return math.sqrt(sum(x*x for x in v))
def dist(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])
def yaw(q):
 assert vec(q,4) and abs(norm(q)-1)<1e-4,'invalid quaternion'
 x,y,z,w=q;return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def angle(a,b):return abs(math.atan2(math.sin(a-b),math.cos(a-b)))
rows=[]
for i,line in enumerate((run/'runtime/observations.jsonl').read_text().splitlines(),1):
 r=json.loads(line);r['source_line']=i;rows.append(r)
actions=[]
for i,line in enumerate((run/'logs/action-client.jsonl').read_text().splitlines(),1):
 r=json.loads(line);r['source_line']=i;actions.append(r)
dispatches=[r for r in actions if r['kind']=='dispatch'];responses=[r for r in actions if r['kind']=='response'];terminals=[r for r in actions if r['kind']=='terminal']
assert len(dispatches)==len(responses)==len(terminals)==1,'not unique action'
dispatch,accepted,terminal=dispatches[0],responses[0],terminals[0];uid=dispatch['data']['uuid']
assert uid==accepted['data']['uuid']==terminal['data']['uuid'] and int(uid,16)!=0
goal=read('artifacts/goal.yaml');params=read('runtime/all-parameters.json')['controller_server']
gp=goal['pose']['pose']['position'];gq=goal['pose']['pose']['orientation'];gyaw=yaw([gq[k] for k in ['x','y','z','w']])
odom_all=[r for r in rows if r['kind']=='odom']
def good_odom(r):
 d=r['data'];return all(vec(d.get(k),n) for k,n in [('position',3),('quaternion_xyzw',4),('linear',3),('angular',3)]) and finite(d.get('stamp_s')) and finite(r.get('ros_now_s')) and abs(norm(d['quaternion_xyzw'])-1)<1e-4
odom=[r for r in odom_all if good_odom(r)];bad_odom=[r['source_line'] for r in odom_all if not good_odom(r)]
assert odom,'No valid odometry'
before=[r for r in odom if r['receive_monotonic_s']<=dispatch['monotonic_s']];assert before
o0=before[-1];assert dispatch['monotonic_s']-o0['receive_monotonic_s']<=.5
motion=[r for r in odom if dispatch['monotonic_s']<=r['receive_monotonic_s']<=terminal['monotonic_s']+.3];assert motion
path=[o0]+motion
movement={'start_line':o0['source_line'],'end_line':motion[-1]['source_line'],'start_position':o0['data']['position'],'end_position':motion[-1]['data']['position'],'net_displacement_m':dist(path[0]['data']['position'],path[-1]['data']['position']),'max_offset_m':max(dist(path[0]['data']['position'],r['data']['position']) for r in path),'path_length_m':sum(dist(a['data']['position'],b['data']['position']) for a,b in zip(path,path[1:])),'samples':len(motion),'scope':'Odometry motion evidence, not independent physical truth.'}
commands={}
bad_commands=[]
for topic in ['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel']:
 allcmd=[r for r in rows if r['kind']==topic]
 cmd=[r for r in allcmd if vec(r['data'].get('linear'),3) and vec(r['data'].get('angular'),3)]
 bad_commands += [r['source_line'] for r in allcmd if r not in cmd]
 assert cmd,'Missing commands, not zero: '+topic
 last=cmd[-1];nonzero=[r for r in cmd if abs(r['data']['linear'][0])>.001 or abs(r['data']['angular'][2])>.001]
 commands[topic]={'count':len(cmd),'last':last,'last_nonzero_line':nonzero[-1]['source_line'] if nonzero else None,'last_vx':last['data']['linear'][0],'last_wz':last['data']['angular'][2],'last_near_zero':abs(last['data']['linear'][0])<=.001 and abs(last['data']['angular'][2])<=.001,'nonzero_after_last':False,'observation_tail_wall_s':rows[-1]['receive_monotonic_s']-last['receive_monotonic_s']}
lastcmd=commands['/cmd_vel']['last']
stopstart=max(terminal['ros_s']+.5,lastcmd['ros_now_s']+.5)
window=[r for r in odom if r['receive_monotonic_s']>=terminal['monotonic_s'] and stopstart<=r['data']['stamp_s']<=stopstart+5]
assert len(window)>=2,'Insufficient stop samples'
stamps=[r['data']['stamp_s'] for r in window];gaps=[b-a for a,b in zip(stamps,stamps[1:])]
clocks=[r for r in rows if r['kind']=='clock'];ctimes=[r['receive_monotonic_s'] for r in clocks];assert clocks
nearest_errors=[]
for r in window:
 i=bisect.bisect_left(ctimes,r['receive_monotonic_s']);near=clocks[max(0,i-1):min(len(clocks),i+1)]
 c=min(near,key=lambda x:abs(x['receive_monotonic_s']-r['receive_monotonic_s']))
 nearest_errors.append(abs(c['data']['sim_s']-r['data']['stamp_s']))
clock_window=[r for r in clocks if window[0]['receive_monotonic_s']-.3<=r['receive_monotonic_s']<=window[-1]['receive_monotonic_s']+.3]
clock_deltas=[b['data']['sim_s']-a['data']['sim_s'] for a,b in zip(clock_window,clock_window[1:])]
stop={'first_line':window[0]['source_line'],'last_line':window[-1]['source_line'],'start_sim_s':stamps[0],'end_sim_s':stamps[-1],'span_sim_s':stamps[-1]-stamps[0],'samples':len(window),'min_gap_sim_s':min(gaps),'max_gap_sim_s':max(gaps),'max_linear_m_s':max(math.hypot(*r['data']['linear'][:2]) for r in window),'max_abs_wz_rad_s':max(abs(r['data']['angular'][2]) for r in window),'max_drift_from_first_m':max(dist(window[0]['data']['position'],r['data']['position']) for r in window),'max_pairwise_drift_m':max(dist(a['data']['position'],b['data']['position']) for a in window for b in window),'max_odom_stamp_age_sim_s':max(abs(r['ros_now_s']-r['data']['stamp_s']) for r in window),'max_nearby_clock_stamp_error_s':max(nearest_errors),'clock_nonadvancing_samples':sum(v<=0 for v in clock_deltas),'invalid_odom_in_window':[r['source_line'] for r in odom_all if not good_odom(r) and window[0]['receive_monotonic_s']<=r['receive_monotonic_s']<=window[-1]['receive_monotonic_s']]}
maprows=[]
for r in rows:
 if r['kind']!='map_base' or r['receive_monotonic_s']<terminal['monotonic_s']:continue
 d=r['data']
 if vec(d.get('position'),3) and vec(d.get('quaternion_xyzw'),4) and finite(d.get('stamp_s')) and abs(norm(d['quaternion_xyzw'])-1)<1e-4 and abs(d['stamp_s']-r['ros_now_s'])<=.5:maprows.append(r)
assert maprows,'No fresh post-terminal localization'
def err(r):return {'position_m':math.hypot(r['data']['position'][0]-gp['x'],r['data']['position'][1]-gp['y']),'yaw_rad':angle(yaw(r['data']['quaternion_xyzw']),gyaw)}
arrival={'first_line':maprows[0]['source_line'],'last_line':maprows[-1]['source_line'],'first_error':err(maprows[0]),'last_error':err(maprows[-1]),'last_pose':maprows[-1],'fresh_samples':len(maprows),'xy_tolerance_m':params['general_goal_checker.xy_goal_tolerance'],'yaw_tolerance_rad':params['general_goal_checker.yaw_goal_tolerance'],'stateful':params['general_goal_checker.stateful'],'scope':'Fresh map->base AMCL/TF localization estimate; no independent arrival truth.'}
matched=[{'line':r['source_line'],**x} for r in rows if r['kind']=='action_status' for x in r['data'] if x['goal_uuid']==uid]
jumps=[r for r in rows if r['kind']=='clock_jump']
checks={'unique_accepted_succeeded':accepted['data']['accepted'] and terminal['data']['action']=='SUCCEEDED' and terminal['data'].get('status')==4 and any(x['status']==4 for x in matched),'movement':movement['max_offset_m']>=.1,'arrival':arrival['last_error']['position_m']<=arrival['xy_tolerance_m'] and arrival['last_error']['yaw_rad']<=arrival['yaw_tolerance_rad'],'final_command':all(c['last_near_zero'] for c in commands.values()) and not bad_commands,'stop':stop['span_sim_s']>=3 and stop['samples']>=5 and 0<stop['min_gap_sim_s']<=stop['max_gap_sim_s']<=.5 and stop['max_linear_m_s']<=.01 and stop['max_abs_wz_rad_s']<=.03 and stop['max_pairwise_drift_m']<=.02 and stop['max_odom_stamp_age_sim_s']<=.5 and stop['max_nearby_clock_stamp_error_s']<=.5 and stop['clock_nonadvancing_samples']==0 and not stop['invalid_odom_in_window'] and not jumps}
summary={'run_id':run.name,'uuid':uid,'checks':checks,'action':{'dispatch':dispatch,'response':accepted,'terminal':terminal,'status_evidence':matched},'movement':movement,'arrival':arrival,'commands':commands,'stop':stop,'invalid_odom_lines':bad_odom,'invalid_command_lines':bad_commands,'clock_jumps':jumps,'raw_sha256':{name:hashlib.sha256((run/name).read_bytes()).hexdigest() for name in ['runtime/observations.jsonl','logs/action-client.jsonl','runtime/all-parameters.json','artifacts/goal.yaml']}}
with (run/'runtime/independent-offline-evidence.json').open('x') as f:json.dump(summary,f,indent=2,allow_nan=False)
print(json.dumps({'run_id':run.name,'uuid':uid,'checks':checks,'movement':movement,'arrival_error':arrival['last_error'],'stop':stop},allow_nan=False))
sys.exit(0 if all(checks.values()) else 1)
