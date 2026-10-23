"""Offline calculations from this run's raw records. No ROS, no writes to inputs."""
from pathlib import Path
import json,math,statistics,sys,bisect,hashlib
run=Path(sys.argv[1])
rows=[json.loads(l) for l in (run/'runtime/observations-goal.jsonl').read_text().splitlines()]
client=[json.loads(l) for l in (run/'logs/action-client.jsonl').read_text().splitlines()]
dispatch=json.loads((run/'runtime/goal-dispatch.json').read_text());terminal=json.loads((run/'runtime/goal-terminal.json').read_text())
goal=json.loads((run/'artifacts/goal.yaml').read_text());audit=json.loads((run/'runtime/pre-goal-audit-v2.json').read_text())
def finite(v):return isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
def valid_vector(v,size):return isinstance(v,list) and len(v)==size and all(finite(x) for x in v)
def vnorm(v):return math.sqrt(sum(x*x for x in v))
def yaw(q):
 assert valid_vector(q,4) and abs(vnorm(q)-1)<1e-5
 x,y,z,w=q;return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def angle(a,b):return abs(math.atan2(math.sin(a-b),math.cos(a-b)))
def xyzdist(a,b):return vnorm([x-y for x,y in zip(a,b)])
odom=[r for r in rows if r['kind']=='odom' and valid_vector(r['data']['position'],3) and valid_vector(r['data']['linear'],3) and valid_vector(r['data']['angular'],3) and finite(r['data']['stamp_s'])]
clocks=[r for r in rows if r['kind']=='clock'];ts=[r['receive_monotonic_s'] for r in clocks]
assert clocks and odom
for r in odom:
 i=bisect.bisect_left(ts,r['receive_monotonic_s']);neighbours=clocks[max(0,i-1):min(len(clocks),i+1)]
 c=min(neighbours,key=lambda x:abs(x['receive_monotonic_s']-r['receive_monotonic_s']))
 r['nearest_clock_stamp_error_s']=abs(c['data']['sim_s']-r['data']['stamp_s'])
 r['nearest_clock_receive_gap_s']=abs(c['receive_monotonic_s']-r['receive_monotonic_s'])
before=[r for r in odom if r['receive_monotonic_s']<=dispatch['monotonic_s']]
assert before and dispatch['monotonic_s']-before[-1]['receive_monotonic_s']<1
o0=before[-1];motion=[r for r in odom if dispatch['monotonic_s']<=r['receive_monotonic_s']<=terminal['monotonic_s']+.5]
assert motion
movement={'start':o0,'end':motion[-1],'net_displacement_m':xyzdist(o0['data']['position'],motion[-1]['data']['position']),'path_length_m':sum(xyzdist(a['data']['position'],b['data']['position']) for a,b in zip([o0]+motion,motion)),'sample_count':len(motion),'max_linear_m_s':max(vnorm(r['data']['linear']) for r in motion),'max_angular_rad_s':max(vnorm(r['data']['angular']) for r in motion)}
commands={}
for topic in ['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel']:
 cmd=[r for r in rows if r['kind']==topic and valid_vector(r['data']['linear'],3) and valid_vector(r['data']['angular'],3)]
 assert cmd,'No command samples: '+topic
 commands[topic]={'count':len(cmd),'last':cmd[-1],'last_linear_m_s':vnorm(cmd[-1]['data']['linear']),'last_angular_rad_s':vnorm(cmd[-1]['data']['angular'])}
lastcmd=commands['/cmd_vel']['last'];assert lastcmd['receive_monotonic_s']>=terminal['monotonic_s']-1
assert commands['/cmd_vel']['last_linear_m_s']<=.01 and commands['/cmd_vel']['last_angular_rad_s']<=.02
stopstart=max(lastcmd['ros_now_s']+.5,terminal['ros_s']+.5)
window=[r for r in odom if stopstart<=r['data']['stamp_s']<=stopstart+5]
assert len(window)>2
stamps=[r['data']['stamp_s'] for r in window]
gaps=[b-a for a,b in zip(stamps,stamps[1:])]
drift=max(xyzdist(window[0]['data']['position'],r['data']['position']) for r in window)
stop={'thresholds':{'min_span_sim_s':3,'max_gap_sim_s':.2,'max_stamp_ros_error_s':.3,'max_linear_m_s':.01,'max_angular_rad_s':.02,'max_drift_m':.02},'start_sim_s':stamps[0],'end_sim_s':stamps[-1],'span_sim_s':stamps[-1]-stamps[0],'count':len(window),'max_gap_sim_s':max(gaps),'stamp_backjumps':sum(x<0 for x in gaps),'max_stamp_ros_error_s':max(abs(r['data']['stamp_s']-r['ros_now_s']) for r in window),'max_nearest_clock_error_s':max(r['nearest_clock_stamp_error_s'] for r in window),'max_nearest_clock_receive_gap_s':max(r['nearest_clock_receive_gap_s'] for r in window),'max_linear_m_s':max(vnorm(r['data']['linear']) for r in window),'max_angular_rad_s':max(vnorm(r['data']['angular']) for r in window),'max_drift_m':drift,'first':window[0],'last':window[-1]}
gps=goal['pose']['pose']['position'];q=goal['pose']['pose']['orientation'];gyaw=yaw([q[k] for k in ['x','y','z','w']])
maprows=[r for r in rows if r['kind']=='map_base' and r['receive_monotonic_s']>=terminal['monotonic_s'] and valid_vector(r['data']['position'],3) and valid_vector(r['data']['quaternion_xyzw'],4) and abs(r['data']['stamp_s']-r['ros_now_s'])<=.5]
assert maprows,'No fresh post-terminal localization'
def error(r):
 d=r['data'];return {'position_m':math.hypot(d['position'][0]-gps['x'],d['position'][1]-gps['y']),'yaw_rad':angle(yaw(d['quaternion_xyzw']),gyaw)}
arrival={'first_fresh_post_terminal':maprows[0],'first_error':error(maprows[0]),'last_fresh_post_terminal':maprows[-1],'last_error':error(maprows[-1]),'fresh_sample_count':len(maprows),'goal_checker':{k:audit['parameters']['controller_server'][k] for k in ['general_goal_checker.xy_goal_tolerance','general_goal_checker.yaw_goal_tolerance','general_goal_checker.stateful']},'scope':'AMCL/TF estimate; not independent physical truth'}
statusrows=[r for r in rows if r['kind']=='action_status']
uid=dispatch['data']['uuid'];matched=[s for r in statusrows for s in r['data'] if s['goal_uuid']==uid]
summary={'run_id':run.name,'uuid':uid,'action':terminal['data']['action'],'status_array_codes':sorted({s['status'] for s in matched}),'dispatch':dispatch,'terminal':terminal,'action_wall_s':terminal['monotonic_s']-dispatch['monotonic_s'],'feedback_count':sum(r['kind']=='feedback' for r in client),'movement':movement,'commands':commands,'arrival':arrival,'stop':stop,'clock_jump_events':[r for r in rows if r['kind']=='clock_jump'],'capture':{'events':len(rows),'first_receive_monotonic_s':rows[0]['receive_monotonic_s'],'last_receive_monotonic_s':rows[-1]['receive_monotonic_s'],'clock_first_s':clocks[0]['data']['sim_s'],'clock_last_s':clocks[-1]['data']['sim_s']},'checks':{}}
summary['checks']={'terminal_succeeded':terminal['data']['action']=='SUCCEEDED' and any(s['status']==4 for s in matched),'movement':movement['net_displacement_m']>.1,'arrival':arrival['last_error']['position_m']<=arrival['goal_checker']['general_goal_checker.xy_goal_tolerance'] and arrival['last_error']['yaw_rad']<=arrival['goal_checker']['general_goal_checker.yaw_goal_tolerance'],'stop':stop['span_sim_s']>=3 and 0<=min(gaps) and max(gaps)<=.2 and stop['max_stamp_ros_error_s']<=.3 and stop['max_nearest_clock_error_s']<=.3 and stop['max_linear_m_s']<=.01 and stop['max_angular_rad_s']<=.02 and drift<=.02 and not summary['clock_jump_events']}
with (run/'runtime/navigation-evidence.json').open('x') as f:json.dump(summary,f,indent=2,allow_nan=False);f.write('\n')
print(json.dumps({k:summary[k] for k in ['run_id','uuid','action','status_array_codes','action_wall_s','feedback_count','checks']}))
print('MOVEMENT',movement['net_displacement_m'],movement['path_length_m'])
print('ARRIVAL',json.dumps(arrival['last_error']))
print('STOP',json.dumps({k:v for k,v in stop.items() if k not in ['first','last']}))
assert all(summary['checks'].values()),'Evidence checks did not all pass'
