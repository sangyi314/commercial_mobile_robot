# SPDX-License-Identifier: MIT
"""Independent raw-observation oracle. Does not call the C++ StopWindow."""
import math
def yaw(q):
 if len(q)!=4 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in q) or abs(sum(x*x for x in q)-1)>.001:raise ValueError('invalid quaternion')
 return math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
def evaluate(rows,after,now=None):
 result={'checks':{'A':False,'B':False,'D':False},'C':'CONSUMER_ACK_UNKNOWN','synthetic':False,'reason':'INSUFFICIENT_EVIDENCE'}
 states=[r for r in rows if r['kind']=='gate_state' and r['receive_monotonic_s']>=after]
 if not states:return result
 state=states[-1]['data'];gen=state['generation'];epoch=state['clock_epoch'];who=state['identity']
 a=[r for r in states if r['data']['generation']==gen and not r['data']['allowed']]
 if not a:return result
 result['checks']['A']=not state['allowed'] and not state['synthetic'];start=max(after,a[0]['receive_monotonic_s'])
 od=[r for r in rows if r['kind']=='odom' and r['receive_monotonic_s']>=start]
 if len(od)<5:return result
 end=od[-1]['data']['stamp_s'];win=[r for r in od if end-3.2<=r['data']['stamp_s']<=end]
 if len(win)<5:return result
 stamps=[r['data']['stamp_s'] for r in win];gaps=[b-a for a,b in zip(stamps,stamps[1:])];wallgaps=[b['receive_monotonic_s']-a['receive_monotonic_s'] for a,b in zip(win,win[1:])]
 try:
  values=[x for r in win for k in ['position','quaternion_xyzw','linear','angular'] for x in r['data'][k]]
  if not all(isinstance(x,(float,int)) and math.isfinite(x) for x in values):return result
  drift=max(math.dist(x['data']['position'][:2],win[0]['data']['position'][:2]) for x in win)
  y0=yaw(win[0]['data']['quaternion_xyzw']);angle=max(abs(math.atan2(math.sin(yaw(x['data']['quaternion_xyzw'])-y0),math.cos(yaw(x['data']['quaternion_xyzw'])-y0))) for x in win)
  v=max(math.sqrt(sum(x*x for x in r['data']['linear'])) for r in win);w=max(math.sqrt(sum(x*x for x in r['data']['angular'])) for r in win)
 except (TypeError,ValueError):return result
 cmds=[r for r in rows if r['kind']=='/cmd_vel' and r['receive_monotonic_s']>=win[0]['receive_monotonic_s']]
 zero=bool(cmds) and all(all(abs(x)<=.001 for k in ['linear','angular'] for x in r['data'][k]) for r in cmds)
 fresh=now is None or (now-od[-1]['receive_monotonic_s']<.2 and now-states[-1]['receive_monotonic_s']<.2 and cmds and now-cmds[-1]['receive_monotonic_s']<.2)
 reports=[r for r in rows if r['kind']=='stop_report' and r['receive_monotonic_s']>=start]
 cpp=bool(reports and reports[-1]['data']['verdict']=='STOPPED' and not reports[-1]['data']['synthetic'] and reports[-1]['data']['generation']==gen and reports[-1]['data']['clock_epoch']==epoch and reports[-1]['data']['identity']==who and reports[-1]['data']['from_ros_ns']>=int(win[0]['data']['stamp_s']*1e9)-1000000000)
 result.update(window={'from_s':stamps[0],'to_s':stamps[-1],'samples':len(win),'max_gap_s':max(gaps),'max_receive_gap_s':max(wallgaps),'drift_m':drift,'yaw_drift_rad':angle,'max_v':v,'max_w':w,'line_range':[win[0].get('line'),win[-1].get('line')]},identity=who,clock_epoch=epoch,generation=gen,cpp_report_qualified=cpp,first_forbidden_receive_s=a[0]['receive_monotonic_s'],first_forbidden_gate_ms=a[0]['data'].get('gate_steady_ms'),gate_reason=a[0]['data'].get('reason'))
 result['checks']['B']=zero and fresh and cmds[-1]['receive_monotonic_s']-cmds[0]['receive_monotonic_s']>=3
 result['checks']['D']=fresh and cpp and stamps[-1]-stamps[0]>=3 and min(gaps)>0 and max(gaps)<=.5 and max(wallgaps)<.2 and v<=.01 and w<=.03 and drift<=.02 and angle<=.03 and all(abs(r['ros_now_s']-r['data']['stamp_s'])<.5 for r in win)
 result['reason']=None if all(result['checks'].values()) else 'STOP_NOT_ESTABLISHED'
 return result


def normal_behavior(rows,actions,goal,parameters,limits):
 """Derived chain: periodic zeros cannot use P0 last-publication+0.5s rule."""
 terminal=[a for a in actions if a['kind']=='terminal'][-1];sent=[a for a in actions if a['kind']=='dispatch'];accepted=[a for a in actions if a['kind']=='response'];t=terminal['monotonic_s'];sim=terminal['ros_s']
 markers=[r for r in rows if r['kind']=='execution_marker' and r['data']['name']=='EXECUTION_NORMAL_REVOKED']
 if not markers:raise ValueError('normal revoke marker absent')
 proof=evaluate(rows,markers[-1]['receive_monotonic_s'],rows[-1]['receive_monotonic_s'])
 maps=[r for r in rows if r['kind']=='map_base' and r['receive_monotonic_s']>=t and sim<=r['data']['stamp_s']<=sim+.5 and abs(r['ros_now_s']-r['data']['stamp_s'])<=.5]
 if not maps:raise ValueError('no fresh fixed-window arrival evidence')
 pose=min(maps,key=lambda r:(r['data']['stamp_s'],r['receive_monotonic_s']));gp=goal['pose']['pose'];q=gp['orientation'];gy=yaw([q[k] for k in ['x','y','z','w']]);py=yaw(pose['data']['quaternion_xyzw']);pos=math.dist(pose['data']['position'][:2],[gp['position']['x'],gp['position']['y']]);angle=abs(math.atan2(math.sin(py-gy),math.cos(py-gy)))
 od=[r for r in rows if r['kind']=='odom'];before=[r for r in od if r['receive_monotonic_s']<=sent[0]['monotonic_s']];motion=[r for r in od if sent[0]['monotonic_s']<=r['receive_monotonic_s']<=t+.3]
 movement=max(math.dist(before[-1]['data']['position'][:2],r['data']['position'][:2]) for r in motion)
 cmds={k:[r for r in rows if r['kind']==k][-1] for k in ['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel']}
 checks={'action':len(sent)==len(accepted)==1 and accepted[0]['data']['accepted'] and terminal['data']['action']=='SUCCEEDED','movement':movement>=.1,'arrival':pos<=limits['position_m'] and angle<=limits['yaw_rad'],'commands':all(all(abs(x)<=.001 for key in ['linear','angular'] for x in v['data'][key]) for v in cmds.values()),'stop':all(proof['checks'].values())}
 return {'checks':checks,'arrival':{'position_m':pos,'yaw_rad':angle,'line':pose['line'],'scope':'AMCL/TF estimate; fixed fresh post-terminal 0.5s, not physical truth'},'stop':proof,'max_displacement_m':movement,'uuid':terminal['data']['uuid']}
