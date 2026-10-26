import copy,json,math,sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from runner_core import *
from runner_analysis import analyze

def test_budget_nested_monotonic():
 now=[10.];clock=lambda:now[0];p=DeadlineBudget.after(5,clock);c=p.child(20);assert c.end==15
 now[0]=14;assert c.remaining()==1;now[0]=16;assert c.remaining()==0
 with pytest.raises(TimeoutError):c.require()
@pytest.mark.parametrize('bad',[float('nan'),float('inf'),-1,True])
def test_invalid_budget(bad):
 with pytest.raises(ValueError):DeadlineBudget.after(bad)
@pytest.mark.parametrize('action',['SUCCEEDED','REJECTED','ABORTED','CANCELED','UNKNOWN'])
def test_terminal_preserved(action):
 s,_=reduce_event(TrialState(),{'type':'READY'},0);s,_=reduce_event(s,{'type':'DISPATCH','uuid':'1'*32},1);s,_=reduce_event(s,{'type':'TERMINAL','uuid':'1'*32,'action':action,'deadline':5},2);assert s.action==action and s.sends==1
 with pytest.raises(ValueError):reduce_event(s,{'type':'DISPATCH','uuid':'2'*32},3)
@pytest.mark.parametrize('received,expected',[(4.999,'COMPLETED'),(5.,'INTERRUPTED'),(5.001,'INTERRUPTED')])
def test_strict_deadline_race(received,expected):
 s,_=reduce_event(TrialState(),{'type':'READY'},0);s,_=reduce_event(s,{'type':'DISPATCH','uuid':'1'*32},1);s,_=reduce_event(s,{'type':'TERMINAL','uuid':'1'*32,'action':'SUCCEEDED','deadline':5.},received);assert s.action=='SUCCEEDED' and s.harness==expected

def test_timeout_cancel_ack_is_not_terminal():
 s,_=reduce_event(TrialState(),{'type':'READY'},0);s,_=reduce_event(s,{'type':'DISPATCH','uuid':'1'*32},1);s,intents=reduce_event(s,{'type':'TIMEOUT'},2);assert s.action=='UNKNOWN' and intents==['CANCEL_OWN_UUID']
 s,_=reduce_event(s,{'type':'TERMINAL','uuid':'1'*32,'action':'CANCELED'},3);assert s.action=='CANCELED' and s.harness=='NAV_TIMEOUT'
 s,_=reduce_event(s,{'type':'CLEANUP','ok':False},4);assert s.ledger.first['code']=='GOAL_DEADLINE' and s.ledger.later[-1]['code']=='CLEANUP_FAILED'
@pytest.mark.parametrize('name',['../x','a/b','','a b','x;touch bad','x'*97])
def test_unsafe_id(name):
 with pytest.raises(ValueError):run_id(name)
def test_atomic_no_overwrite_and_nan(tmp_path):
 p=tmp_path/'result.json';write_json(p,{'a':None});assert json.loads(p.read_text())=={'a':None}
 with pytest.raises(FileExistsError):write_json(p,{'a':True})
 assert json.loads(p.read_text())['a'] is None
 with pytest.raises(ValueError):write_json(tmp_path/'bad.json',{'x':float('nan')})
 assert not (tmp_path/'bad.json').exists()
def test_atomic_interrupted_before_publish(tmp_path,monkeypatch):
 monkeypatch.setattr(os,'link',lambda *a:(_ for _ in ()).throw(OSError('interruption')))
 with pytest.raises(OSError):atomic_write_new(tmp_path/'result.json',b'{}')
 assert list(tmp_path.iterdir())==[]
def test_strict_protocol_and_path(tmp_path):
 with pytest.raises(ValueError):strict_json('{"x":1,"x":2}')
 with pytest.raises(ValueError):within(tmp_path,tmp_path/'../escape')
 for changed in [{'run_id':'wrong'},{'seq':0},{'protocol_version':2}]:
  e={'run_id':'trial','seq':1,'protocol_version':1,'type':'RESULT','request_id':'1'};e.update(changed)
  with pytest.raises(ValueError):validate_envelope(e,'trial',0)

def samples():
 uid='1'*32;rows=[]
 for i in range(201):
  t=i*.1;p=min(1.,max(0.,(t-1)/8));v=.125 if 1<t<9 else 0.
  rows+=[{'kind':'clock','ros_now_s':t,'receive_monotonic_s':t,'data':{'sim_s':t}}, {'kind':'odom','ros_now_s':t,'receive_monotonic_s':t,'data':{'stamp_s':t,'position':[p,0.,0.],'quaternion_xyzw':[0.,0.,0.,1.],'linear':[v,0.,0.],'angular':[0.,0.,0.]}},{'kind':'map_base','ros_now_s':t,'receive_monotonic_s':t,'data':{'stamp_s':t,'position':[p,0.,0.],'quaternion_xyzw':[0.,0.,0.,1.]}}]
  if t<=11:
   for topic in ['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel']:rows.append({'kind':topic,'ros_now_s':t,'receive_monotonic_s':t,'data':{'linear':[.1 if t<10 else 0.,0.,0.],'angular':[0.,0.,0.]}})
 rows.append({'kind':'action_status','ros_now_s':10,'receive_monotonic_s':10,'data':[{'goal_uuid':uid,'status':4}]})
 actions=[{'kind':'dispatch','monotonic_s':1.,'ros_s':1.,'data':{'uuid':uid}},{'kind':'response','monotonic_s':1.1,'ros_s':1.1,'data':{'uuid':uid,'accepted':True}},{'kind':'terminal','monotonic_s':10.,'ros_s':10.,'data':{'uuid':uid,'action':'SUCCEEDED'}}]
 goal={'pose':{'pose':{'position':{'x':1.,'y':0.},'orientation':{'x':0.,'y':0.,'z':0.,'w':1.}}}};params={'general_goal_checker.xy_goal_tolerance':.25,'general_goal_checker.yaw_goal_tolerance':.25}
 return rows,actions,goal,params

def test_independent_stop_success():assert all(analyze(*samples())['checks'].values())
@pytest.mark.parametrize('case',['missing_odom','missing_cmd','moving','stale','clock_paused','clock_backjump','gap','invalid'])
def test_stop_bad_evidence_rejected(case):
 rows,actions,goal,p=samples()
 if case=='missing_odom':rows=[r for r in rows if r['kind']!='odom']
 elif case=='missing_cmd':rows=[r for r in rows if r['kind']!='/cmd_vel']
 elif case=='clock_backjump':rows.append({'kind':'clock_jump'})
 elif case=='clock_paused':
  for r in rows:
   if r['kind']=='clock':r['data']['sim_s']=1.
 elif case=='gap':rows=[r for r in rows if not(r['kind']=='odom' and 13<r['data']['stamp_s']<14)]
 else:
  for r in rows:
   if r['kind']=='odom' and r['ros_now_s']>11:
    if case=='moving':r['data']['linear'][0]=.05
    elif case=='stale':r['ros_now_s']+=3
    elif case=='invalid':r['data']['linear'][0]=None
 try:report=analyze(rows,actions,goal,p);assert not all(report['checks'].values())
 except ValueError:pass


def test_finalize_unknowns_and_runtime_gate():
 m={'run_id':'fixture','record_origin':'fixture','baseline':{'method':'mppi'},'clock':{'clock_epoch':'fixture:0'}}
 s=TrialState();s,_=reduce_event(s,{'type':'FAIL','code':'NO_CLOCK','detail':'no samples'},1);s,_=reduce_event(s,{'type':'CLEANUP','ok':True},2)
 r=finalize_record(m,'a'*64,s,3,[]);assert r['metrics']['action_success'] is None and r['metrics']['stop_observed'] is None and r['evaluation']['collision'] is None
 s,_=reduce_event(s,{'type':'READY'},4)
 with pytest.raises(ValueError,match='runtime'):finalize_record(m,'a'*64,s,4,[],PLUGINS['mppi'])
 s,_=reduce_event(s,{'type':'DISPATCH','uuid':'1'*32},5);s,_=reduce_event(s,{'type':'TERMINAL','uuid':'1'*32,'action':'SUCCEEDED'},6);s,_=reduce_event(s,{'type':'CLEANUP','ok':False},7)
 r=finalize_record(m,'a'*64,s,7,[{'id':'runtime_parameters','kind':'runtime_parameters'}],PLUGINS['mppi']);assert r['metrics']['action_success'] is True and r['status']['cleanup']=='FAILED'

def test_preflight_no_process_or_run_creation(monkeypatch):
 import run_trial,subprocess
 root=Path(__file__).resolve().parents[1];before=set((root/'evidence/runs').iterdir())
 monkeypatch.setattr(subprocess,'Popen',lambda *a,**k:(_ for _ in ()).throw(AssertionError('preflight spawned process')))
 monkeypatch.setattr(sys,'argv',['run_trial','--scenario',str(root/'experiments/scenarios/p0_05_official_static.yaml'),'--profile',str(root/'experiments/runner_profile.yaml'),'--method','mppi','--run-id','p0-05-unit-check-only','--check'])
 assert run_trial.main()==0 and before==set((root/'evidence/runs').iterdir())


def test_raw_xacro_is_not_execution_sdf(tmp_path):
 import run_trial
 root=Path(__file__).resolve().parents[1];profile=json.loads((root/'experiments/runner_profile.yaml').read_text());profile['files']['robot']=profile['files']['robot_source'];bad=tmp_path/'bad-profile.yaml';bad.write_text(json.dumps(profile))
 with pytest.raises(ValueError,match='expanded robot'):run_trial.load_inputs(root/'experiments/scenarios/p0_05_official_static.yaml',bad,'mppi','p0-05-test-bad-robot')


def test_broken_console_pipe_cannot_abort_cleanup_path(monkeypatch):
 import run_trial,io
 class Broken(io.StringIO):
  def write(self,s):raise BrokenPipeError('consumer gone')
 monkeypatch.setattr(sys,'stdout',Broken());run_trial.safe_print('PHASE','SHUTDOWN');assert sys.stdout is not None


def test_recorder_failure_keeps_actual_action_terminal():
 state=TrialState()
 for event in [{'type':'READY'},{'type':'DISPATCH','uuid':'9'*32},{'type':'TERMINAL','uuid':'9'*32,'action':'SUCCEEDED'},{'type':'FAIL','code':'BAG_INVALID','detail':'missing metadata','harness':'RECORDER_FAILED'}]:
  state,_=reduce_event(state,event,1.)
 assert state.action=='SUCCEEDED' and state.harness=='RECORDER_FAILED' and state.ledger.first['code']=='BAG_INVALID'
