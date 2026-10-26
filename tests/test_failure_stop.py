import sys,copy,math
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from failure_stop import *
from test_runner_lifecycle import samples
LIMIT={'min_span_s':3.,'min_samples':5,'max_gap_s':.5,'max_linear':.01,'max_wz':.03,'max_drift':.02,'max_cmd':.001}

def test_stop_without_success_or_arrival():
 rows,*_=samples();r=observe_stop(rows,12.,10.,LIMIT);assert all(r['checks'].values());assert r['stop']['samples']>=5
@pytest.mark.parametrize('fault',['no_odom','no_cmd','paused','reversed','gap','nan','drift','moving','rotation','stale','late_nonzero','cached','wrong_age'])
def test_reject_missing_or_bad(fault):
 rows,*_=samples()
 if fault=='no_odom':rows=[r for r in rows if r['kind']!='odom']
 elif fault=='no_cmd':rows=[r for r in rows if r['kind']!='/cmd_vel']
 elif fault=='gap':rows=[r for r in rows if not(r['kind']=='odom' and 18<r['ros_now_s']<19)]
 elif fault=='reversed':rows=list(reversed(rows))
 elif fault=='late_nonzero':rows.append({'kind':'/cmd_vel','ros_now_s':21,'receive_monotonic_s':21,'data':{'linear':[.02,0,0],'angular':[0,0,0]}})
 else:
  for r in rows:
   if r['kind']=='clock' and fault=='paused':r['data']['sim_s']=5.
   if r['kind']=='odom' and r['ros_now_s']>=16:
    if fault=='nan':r['data']['linear'][0]=float('nan')
    if fault=='drift':r['data']['position'][0]+=.1*math.sin(r['ros_now_s'])
    if fault=='moving':r['data']['linear'][0]=.011
    if fault=='rotation':r['data']['angular'][2]=.031
    if fault in ['stale','wrong_age']:r['ros_now_s']+=2
    if fault=='cached':r['data']['stamp_s']=17.
 assert not all(observe_stop(rows,12.,10.,LIMIT)['checks'].values())
@pytest.mark.parametrize('state,transition',[(1,5),(2,6),(3,7),(4,None)])
def test_actual_shutdown(state,transition):assert shutdown_transition(state)==transition
@pytest.mark.parametrize('state',[0,10,11,12,99,True,None])
def test_unknown_shutdown_rejected(state):
 with pytest.raises(ValueError):shutdown_transition(state)

def test_fixed_arrival_policy_not_best_error_and_legacy_unchanged():
 from runner_analysis import analyze
 rows,actions,goal,params=samples()
 for x in rows:
  if x['kind']=='map_base' and x['ros_now_s']>11:x['data']['position'][0]=1.3
 assert not analyze(rows,actions,goal,params)['checks']['arrival']
 new=analyze(rows,actions,goal,params,arrival_policy='first_fresh_post_terminal_0.5s')
 assert new['checks']['arrival'] and new['arrival']['late_position_m']>.25
 for x in rows:
  if x['kind']=='map_base' and x['ros_now_s']==10:x['data']['position'][0]=1.4
 assert not analyze(rows,actions,goal,params,arrival_policy='first_fresh_post_terminal_0.5s')['checks']['arrival']
 rows=[x for x in rows if not(x['kind']=='map_base' and 10<=x['ros_now_s']<=10.5)]
 with pytest.raises(ValueError,match='fixed'):analyze(rows,actions,goal,params,arrival_policy='first_fresh_post_terminal_0.5s')
