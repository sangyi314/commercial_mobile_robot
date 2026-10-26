"""No simulator, no ROS nodes. Assert ownership and dispatch boundaries."""
import ast,json,sys
from pathlib import Path
import pytest
import tools.runner.trial as run_trial
from tools.runner.runner_core import DeadlineBudget, TrialState, reduce_event, finalize_record, PLUGINS
from tools.dynamic.runner_dynamic import DynamicSession
ROOT=Path(__file__).resolve().parents[2]
@pytest.mark.parametrize('name',['probe','crossing','observed_occlusion'])
def test_dynamic_drafts_rejected_without_spawn_or_directory(monkeypatch,name,tmp_path):
 before=set((ROOT/'evidence/runs').iterdir())
 monkeypatch.setattr(run_trial.subprocess,'Popen',lambda *a,**k:pytest.fail('must not spawn'))
 scenario='p0_06_probe' if name=='probe' else name
 draft=run_trial.validate_run.load_strict(ROOT/('experiments/tool_layout_v2/scenarios/'+scenario+'.yaml'))
 draft['status']='draft';draft['dynamic']['preparation']={'review':None,'probe':None}
 source=tmp_path/'fixture_draft.json';source.write_text(json.dumps(draft))
 with pytest.raises(ValueError,match='draft|prepared'):
  run_trial.load_inputs(source,ROOT/('experiments/tool_layout_v2/profiles/p0_06_'+name+'.yaml'),'mppi','p0-06-unit-draft')
 assert before==set((ROOT/'evidence/runs').iterdir())
@pytest.mark.parametrize('method',['mppi','dwb'])
def test_old_preflight_keeps_no_side_effects(monkeypatch,method):
 monkeypatch.setattr(run_trial.subprocess,'Popen',lambda *a,**k:pytest.fail('no process'))
 p,s=run_trial.load_inputs(ROOT/'experiments/scenarios/p0_05_official_static.yaml',ROOT/'experiments/runner_profile.yaml',method,'p0-06-old-precheck')
 assert p['profile_version']==1 and s['kind']=='static_smoke'
def test_probe_cannot_bind_goal():
 x=DynamicSession.__new__(DynamicSession);x.profile={'mode':'probe'}
 with pytest.raises(ValueError,match='cannot accept'):x.arm(DeadlineBudget.after(1),{'kind':'response'})
def test_recording_needs_actual_acceptance():
 x=DynamicSession.__new__(DynamicSession);x.profile={'mode':'navigation_recording'}
 with pytest.raises(ValueError,match='NO_ACCEPTED'):x.arm(DeadlineBudget.after(1),None)
def test_no_second_lifecycle():
 source=(ROOT/'tools/dynamic/runner_dynamic.py').read_text();tree=ast.parse(source)
 assert not any(isinstance(x,ast.Attribute) and x.attr in ['Popen','kill','killpg','system'] for x in ast.walk(tree))
 assert 'supervisor.spawn' in source and 'supervisor.close' in source
def test_probe_result_v1_keeps_unknown_metrics():
 s=TrialState();s,_=reduce_event(s,{'type':'READY'},1);s,_=reduce_event(s,{'type':'CLEANUP','ok':True},2)
 manifest={'run_id':'fixture_probe','record_origin':'fixture','baseline':{'method':'mppi'},'clock':{'clock_epoch':'fixture_probe:sim0'}}
 r=finalize_record(manifest,'a'*64,s,3,[{'id':'runtime_parameters','kind':'runtime_parameters'}],PLUGINS['mppi'])
 assert r['status']['action']=='NOT_SENT' and r['metrics']['action_success'] is None and r['evaluation']['collision'] is None
def test_worker_probe_send_guard_precedes_action():
 text=(ROOT/'tools/navigation/nav_goal.py').read_text();start=text.index("if op=='send':");guard=text.index("probe forbids navigation goals",start);send=text.index('ac.send_goal_async',start)
 assert start<guard<send

def test_dynamic_failure_reason_is_preserved():
 from tools.dynamic.runner_dynamic import DynamicFailure
 x=DynamicSession.__new__(DynamicSession)
 with pytest.raises(DynamicFailure) as caught:x.event({'type':'FAILURE','data':{'code':'MOTION_SERVICE_TIMEOUT'}},0)
 assert caught.value.code=='MOTION_SERVICE_TIMEOUT'
def test_pause_cannot_escape_into_navigation():
 x=DynamicSession.__new__(DynamicSession);x.profile={'mode':'navigation_recording'}
 with pytest.raises(ValueError,match='PROBE_ONLY'):x.pause_check(DeadlineBudget.after(1))

def test_quiesce_drains_once_without_resending():
 x=DynamicSession.__new__(DynamicSession);calls=[];states=iter([{'inflight':True},{'inflight':False}])
 x.rpc=lambda role,op,args,budget:(calls.append(op) or {'inflight':True})
 x.status=lambda budget:next(states)
 class Worker:
  def pump(self):calls.append('pump')
 x.worker=Worker();x.quiesce(DeadlineBudget.after(1))
 assert calls==['quiesce','pump']

def test_probe_must_observe_stationary_robot_before_any_motion():
 x=DynamicSession.__new__(DynamicSession);x.profile={'mode':'probe'}
 class Worker:
  def rpc(self,*args):raise RuntimeError('robot moved')
 x.worker=Worker()
 with pytest.raises(RuntimeError,match='robot moved'):x.probe(DeadlineBudget.after(1))

def test_pause_has_no_parent_quiesce_round_trip():
 x=DynamicSession.__new__(DynamicSession);x.profile={'mode':'probe'};calls=[]
 class Worker:
  def rpc(self,op,args,budget):calls.append(op)
 x.worker=Worker()
 x.quiesce=lambda budget:pytest.fail('quiesce/pause must be atomic in driver')
 def rpc(role,op,args,budget):
  calls.append(op)
  if op=='pause':raise RuntimeError('stop at pause dispatch')
 x.rpc=rpc
 with pytest.raises(RuntimeError,match='stop at pause dispatch'):x.pause_check(DeadlineBudget.after(1))
 assert calls==['probe_pause','pause']

@pytest.mark.parametrize('recorded_finish_ns,outcome',[(3000000000,'pass'),(31000000000,'late'),(2000000000,'missing')])
def test_late_observation_uses_actual_completion_deadline(tmp_path,recorded_finish_ns,outcome):
 x=DynamicSession.__new__(DynamicSession);x.run=tmp_path;x.profile={'mode':'navigation_recording','dynamic_profile':{'window_s':30}};x.release=0;x.entity=1
 (tmp_path/'artifacts').mkdir();(tmp_path/'runtime').mkdir()
 (tmp_path/'artifacts/dynamic_motion.json').write_text(json.dumps({'knots':[{'t_ns':0,'xyz':[0,0,.3]},{'t_ns':1000000000,'xyz':[.1,0,.3]}],'hold_ns':2000000000}))
 (tmp_path/'runtime/motion-commands.jsonl').write_text(json.dumps({'kind':'request','entity_id':1,'sim_ns':recorded_finish_ns})+'\n')
 x.status=lambda budget:{'phase':'FINISHED','inflight':False,'sim_ns':100000000000,'actual_xyz':[.1,0,.3]};events=[];x.log=lambda *args:events.append(args)
 if outcome=='late':
  with pytest.raises(TimeoutError,match='SCENE_WINDOW_EXCEEDED'):x.wait_complete(DeadlineBudget.after(1))
 elif outcome=='missing':
  with pytest.raises(ValueError,match='FINISH_EVIDENCE_MISSING'):x.wait_complete(DeadlineBudget.after(1))
 else:
  x.wait_complete(DeadlineBudget.after(1));assert events[-1][1]['completion_request_sim_ns']==3000000000 and events[-1][1]['completion_observed_sim_ns']==100000000000
