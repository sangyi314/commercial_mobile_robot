"""Pure abnormal-stop observation; no arrival requirement and no missing-as-zero."""
import math
from tools.runner.runner_analysis import finite, vector, distance

TOPICS=['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel']
def observe_stop(rows,after,command_after,limits,axis='monotonic'):
 at=lambda r:r['receive_monotonic_s'] if axis=='monotonic' else r['ros_now_s']
 odall=[r for r in rows if r['kind']=='odom' and at(r)>=after]
 valid=lambda r:finite(r['data'].get('stamp_s')) and all(vector(r['data'].get(k),n) for k,n in [('position',3),('linear',3),('angular',3),('quaternion_xyzw',4)])
 od=[r for r in odall if valid(r)]
 commands={}
 for topic in TOPICS:
  cs=[r for r in rows if r['kind']==topic]
  good=bool(cs) and all(vector(r['data'].get(k),3) for r in cs for k in ['linear','angular'])
  last=cs[-1] if cs else None
  commands[topic]={'last':last,'valid':bool(good),'ok':bool(good and at(last)>=command_after and abs(last['data']['linear'][0])<=limits['max_cmd'] and abs(last['data']['angular'][2])<=limits['max_cmd'])}
 result={'checks':{'commands':all(v['ok'] for v in commands.values()),'stop':False,'clock':False},'commands':commands,'stop':None,'time_axis':axis,'after':after,'command_after':command_after,'reason':'INSUFFICIENT_SAMPLES'}
 if len(od)<limits['min_samples'] or not odall or not valid(odall[-1]):return result
 end=od[-1]['data']['stamp_s'];win=[r for r in od if end-(limits['min_span_s']+.5)<=r['data']['stamp_s']<=end]
 if len(win)<2:return result
 stamps=[r['data']['stamp_s'] for r in win];gaps=[b-a for a,b in zip(stamps,stamps[1:])]
 invalid=any(not valid(r) and at(win[0])<=at(r) for r in odall)
 clocks=[r['data']['sim_s'] for r in rows if r['kind']=='clock' and at(win[0])<=at(r)<=at(win[-1])]
 clockok=len(clocks)>=2 and all(finite(t) for t in clocks) and all(b>=a for a,b in zip(clocks,clocks[1:])) and clocks[-1]-clocks[0]>=limits['min_span_s'] and not any(r['kind']=='clock_jump' for r in rows)
 drift=max(distance(a['data']['position'],b['data']['position']) for a in win for b in win)
 stop={'start_sim_s':stamps[0],'end_sim_s':stamps[-1],'span_s':stamps[-1]-stamps[0],'samples':len(win),'min_gap_s':min(gaps),'max_gap_s':max(gaps),'max_linear':max(math.hypot(*r['data']['linear'][:2]) for r in win),'max_wz':max(abs(r['data']['angular'][2]) for r in win),'max_drift_m':drift,'max_age_s':max(abs(r['ros_now_s']-r['data']['stamp_s']) for r in win),'line_range':[win[0].get('line'),win[-1].get('line')]}
 ok=not invalid and len(win)>=limits['min_samples'] and stop['span_s']>=limits['min_span_s'] and 0<min(gaps)<=max(gaps)<=limits['max_gap_s'] and stop['max_linear']<=limits['max_linear'] and stop['max_wz']<=limits['max_wz'] and drift<=limits['max_drift'] and stop['max_age_s']<=.5
 result.update(stop=stop,reason=None if ok and clockok and result['checks']['commands'] else 'STOP_NOT_ESTABLISHED');result['checks'].update(stop=ok,clock=clockok)
 return result

def shutdown_transition(state):
 if type(state) is not int or state not in [1,2,3,4]:raise ValueError('transitional or unknown lifecycle state')
 return {1:5,2:6,3:7,4:None}[state]
