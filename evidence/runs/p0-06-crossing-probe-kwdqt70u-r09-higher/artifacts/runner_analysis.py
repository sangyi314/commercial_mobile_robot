"""Offline behavior calculations shared by sidecar and independent bag samples."""
import math

def finite(x):return type(x) in (int,float) and math.isfinite(x)
def vector(x,n):return isinstance(x,list) and len(x)==n and all(finite(v) for v in x)
def distance(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])
def yaw(q):
 if not vector(q,4) or abs(sum(v*v for v in q)-1)>.001:raise ValueError('quaternion')
 x,y,z,w=q;return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def analyze(rows,actions,goal,parameters,axis="monotonic"):
 recv=lambda r:r["receive_monotonic_s"] if axis=="monotonic" else r["ros_now_s"]
 at=lambda r:r["monotonic_s"] if axis=="monotonic" else r["ros_s"]
 dispatch=[r for r in actions if r['kind']=='dispatch'];response=[r for r in actions if r['kind']=='response'];terminal=[r for r in actions if r['kind']=='terminal']
 if len(dispatch)!=1 or len(response)!=1 or len(terminal)!=1:raise ValueError('unique action evidence absent')
 d,a,t=dispatch[0],response[0],terminal[0];uid=d['data']['uuid']
 if uid!=a['data']['uuid'] or uid!=t['data']['uuid']:raise ValueError('UUID mismatch')
 odall=[r for r in rows if r['kind']=='odom']
 def valid(r):return finite(r['data'].get('stamp_s')) and all(vector(r['data'].get(k),n) for k,n in [('position',3),('linear',3),('angular',3),('quaternion_xyzw',4)])
 od=[r for r in odall if valid(r)]
 before=[r for r in od if recv(r)<=at(d)]
 moving=[r for r in od if at(d)<=recv(r)<=at(t)+.3]
 if not before or not moving:raise ValueError('missing motion samples')
 origin=before[-1];move=max(distance(origin['data']['position'],r['data']['position']) for r in moving)
 commands={}
 for topic in ['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel']:
  allc=[r for r in rows if r['kind']==topic]
  if not allc or any(not vector(r['data'].get('linear'),3) or not vector(r['data'].get('angular'),3) for r in allc):raise ValueError('missing/invalid commands '+topic)
  last=allc[-1];commands[topic]={'last':last,'ok':abs(last['data']['linear'][0])<=.001 and abs(last['data']['angular'][2])<=.001,'samples':len(allc)}
 start=max(t['ros_s']+.5,commands['/cmd_vel']['last']['ros_now_s']+.5)
 win=[r for r in od if start<=r['data']['stamp_s']<=start+5 and recv(r)>=at(t)]
 if len(win)<2:raise ValueError('missing stop window')
 gaps=[b['data']['stamp_s']-a['data']['stamp_s'] for a,b in zip(win,win[1:])]
 drift=max(distance(a['data']['position'],b['data']['position']) for a in win for b in win)
 stop={'start_sim_s':win[0]['data']['stamp_s'],'end_sim_s':win[-1]['data']['stamp_s'],'span_s':win[-1]['data']['stamp_s']-win[0]['data']['stamp_s'],'samples':len(win),'max_gap_s':max(gaps),'min_gap_s':min(gaps),'max_linear':max(math.hypot(*r['data']['linear'][:2]) for r in win),'max_wz':max(abs(r['data']['angular'][2]) for r in win),'max_drift_m':drift,'max_age_s':max(abs(r['ros_now_s']-r['data']['stamp_s']) for r in win),'line_range':[win[0].get('line'),win[-1].get('line')]}
 maps=[r for r in rows if r['kind']=='map_base' and recv(r)>=at(t) and vector(r['data'].get('position'),3) and vector(r['data'].get('quaternion_xyzw'),4) and abs(r['ros_now_s']-r['data']['stamp_s'])<=.5]
 if not maps:raise ValueError('no fresh post-terminal localization')
 p=maps[-1]['data'];gp=goal['pose']['pose'];gq=gp['orientation'];gy=yaw([gq[k] for k in ['x','y','z','w']]);py=yaw(p['quaternion_xyzw'])
 arrival={'position_m':distance(p['position'],[gp['position']['x'],gp['position']['y']]),'yaw_rad':abs(math.atan2(math.sin(py-gy),math.cos(py-gy))),'line':maps[-1].get('line'),'stamp_s':p['stamp_s'],'scope':'AMCL/TF estimate, not independent physical truth'}
 matched=[x for r in rows if r['kind']=='action_status' for x in r['data'] if x['goal_uuid']==uid]
 badwindow=any(not valid(r) and recv(win[0])<=recv(r)<=recv(win[-1]) for r in odall)
 clocks=[r['data']['sim_s'] for r in rows if r['kind']=='clock']
 clockwindow=[r['data']['sim_s'] for r in rows if r['kind']=='clock' and recv(win[0])<=recv(r)<=recv(win[-1])]
 clockok=len(clocks)>2 and len(clockwindow)>2 and max(clockwindow)-min(clockwindow)>=3 and all(b>=a for a,b in zip(clocks,clocks[1:])) and not any(r['kind']=='clock_jump' for r in rows)
 checks={'action':a['data']['accepted'] and t['data']['action']=='SUCCEEDED' and any(x['status']==4 for x in matched),'movement':move>=.1,'arrival':arrival['position_m']<=parameters['general_goal_checker.xy_goal_tolerance'] and arrival['yaw_rad']<=parameters['general_goal_checker.yaw_goal_tolerance'],'commands':all(v['ok'] and recv(v['last'])>=at(t)-.5 for v in commands.values()),'stop':stop['span_s']>=3 and len(win)>=5 and 0<min(gaps)<=max(gaps)<=.5 and stop['max_linear']<=.01 and stop['max_wz']<=.03 and drift<=.02 and stop['max_age_s']<=.5 and not badwindow and clockok}
 return {'uuid':uid,'checks':checks,'max_displacement_m':move,'net_displacement_m':distance(origin['data']['position'],moving[-1]['data']['position']),'movement_lines':[origin.get('line'),moving[-1].get('line')],'arrival':arrival,'stop':stop,'commands':commands}
