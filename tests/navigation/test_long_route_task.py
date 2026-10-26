"""Version binding and capture-boundary counterexamples, no ROS or simulation."""
import copy,json,sys
from pathlib import Path
import pytest
import tools.runner.trial as run_trial, tools.geometry.geometry_inputs as geometry_inputs
from tools.contracts.task_contract import validate_task
from tools.runner.runner_core import digest, DeadlineBudget
from tools.dynamic.runner_dynamic import DynamicSession
R=Path(__file__).resolve().parents[2]
def load(p):return json.loads(Path(p).read_text())
@pytest.mark.parametrize('method',['mppi','dwb'])
@pytest.mark.parametrize('legacy',[True,False])
def test_actual_task_entries_no_process(method,legacy,monkeypatch):
 monkeypatch.setattr(run_trial.subprocess,'Popen',lambda *a,**k:pytest.fail('cannot spawn'))
 s=R/('experiments/scenarios/p0_05_official_static.yaml' if legacy else 'experiments/scenarios/p0_long_route_static_v1.yaml')
 p=R/('experiments/runner_profile.yaml' if legacy else 'experiments/profiles/p0_long_route_static_v1.yaml')
 profile,spec=run_trial.load_inputs(s,p,method,'long-task-fixture')
 assert spec['task']['goal_pose']['x_m']==(-1.9568372509483436 if legacy else 1.75)
@pytest.mark.parametrize('case',['scenario_goal','initial','hash','goal_request','nonunit','spawn','version'])
def test_task_rejects_inconsistent_frozen_inputs(case,tmp_path):
 s=load(R/'experiments/scenarios/p0_long_route_static_v1.yaml');p=load(R/'experiments/profiles/p0_long_route_static_v1.yaml')
 if case=='scenario_goal':s['task']['goal_pose']['x_m']=1.7
 if case=='initial':s['task']['initial_pose']['y_m']=0
 if case=='hash':p['task_inputs']['sha256']='a'*64
 if case=='spawn':p['spawn']['x']=-1.9
 if case=='version':p['task_version']='arbitrary'
 if case in ('goal_request','nonunit'):
  g=load(p['files']['goal']['path'])
  if case=='goal_request':g['pose']['pose']['position']['x']=1.7
  else:g['pose']['pose']['orientation']['w']=2
  f=tmp_path/'request.json';f.write_text(json.dumps(g));p['files']['goal']={'path':str(f),'sha256':digest(f)}
  # Even a self-consistent set of file hashes cannot authorize an arbitrary goal.
  frozen=load(p['task_inputs']['path']);frozen['files']['goal']=p['files']['goal'];b=tmp_path/'binding.json';b.write_text(json.dumps(frozen));p['task_inputs']={'path':str(b),'sha256':digest(b)}
 with pytest.raises(ValueError):validate_task(s,p)
def test_old_preparation_cannot_authorize_new_goal(tmp_path):
 import tools.contracts.validation as v
 from tools.dynamic.probe_preparation import prepare
 from tools.contracts.dynamic_contract import fingerprint
 s=load(R/'experiments/tool_layout_v2/scenarios/crossing.yaml');s['status']='draft';s['dynamic']['preparation']={'review':None,'probe':None};f=tmp_path/'draft.json';f.write_text(json.dumps(s))
 paths=prepare(f,R/'experiments/tool_layout_v2/profiles/p0_06_crossing.yaml',tmp_path/'new-prep',True,as_probe=True)
 s=load(paths['scenario']);old=R/'evidence/env/p0-06-live-kwdqt70u/crossing-r10-preparation/preparation.json'
 s['dynamic']['preparation']['review']={'path':str(old),'sha256':digest(old)}
 with pytest.raises(ValueError,match='preparation'):v.scenario_rules(s,'fixture')
@pytest.mark.parametrize('terminal_ns,motion_end_ns',[(60_000_000_000,20_000_000_000),(10_000_000_000,70_000_000_000)])
def test_long_capture_cannot_close_at_motion_or_terminal_only(tmp_path,monkeypatch,terminal_ns,motion_end_ns):
 required=max(terminal_ns+5_000_000_000,motion_end_ns)
 monkeypatch.setattr(geometry_inputs,'scope',lambda run:{'interval_ns':[1_000_000_000,required]})
 x=DynamicSession.__new__(DynamicSession);x.run=tmp_path;x.profile={'dynamic_spec':{'dynamic':{'truth_gap_ns':50_000_000}}}
 stamps=iter([terminal_ns,motion_end_ns,required-1,required,required+50_000_000]);pumps=[];logs=[]
 x.status=lambda budget:{'pose_stamp_ns':next(stamps)}
 class Worker:
  def pump(self):pumps.append(1)
 x.worker=Worker();x.log=lambda *v:logs.append(v)
 x.cover_geometry_scope(DeadlineBudget.after(1))
 assert len(pumps)>=2 and logs[-1][1]['actual_pose_source_ns']>=required+50_000_000
