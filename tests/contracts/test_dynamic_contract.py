"""Synthetic contract cases only; never create a real run."""
import copy,json,sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import validate_run as v
import run_trial
from dynamic_contract import motion_rules,resource,fingerprint
ROOT=Path(__file__).resolve().parents[1]
@pytest.fixture
def current_probe_inputs(tmp_path):
 # Production draft hashes intentionally preserve their historical build.
 # Bind only the isolated test copy to the binary under test.
 scenario=ROOT/'experiments/scenarios/p0_06_probe.yaml'
 profile=v.load_strict(ROOT/'experiments/profiles/p0_06_probe.yaml')
 for ref in profile['executables'].values():ref['sha256']=v.digest(Path(ref['path']))
 target=tmp_path/'fixture_current_profile.json';target.write_text(json.dumps(profile))
 return scenario,target
@pytest.fixture
def spec():
 s=v.load_strict(ROOT/'experiments/scenarios/crossing.yaml')
 s['status']='draft';s['dynamic']['preparation']={'review':None,'probe':None}
 return s
@pytest.mark.parametrize('name',['p0_06_probe','crossing','observed_occlusion'])
def test_new_drafts_valid_but_not_ready(name,tmp_path):
 s=v.load_strict(ROOT/('experiments/scenarios/'+name+'.yaml'))
 s['status']='draft';s['dynamic']['preparation']={'review':None,'probe':None}
 p=tmp_path/'fixture_draft.json';p.write_text(json.dumps(s))
 assert v.validate_scenario(p)['status']=='VALID'
 assert v.validate_scenario(p,True)['status']=='NOT_READY'
@pytest.mark.parametrize('case',['version','entity','frame','trigger','empty_actor','empty_motion','duplicate','nonfinite','unknown_field'])
def test_bad_contract(spec,tmp_path,case):
 if case=='version':spec['schema_version']=3
 if case=='entity':spec['dynamic']['entity_name']='another'
 if case=='frame':spec['dynamic']['frame']='map'
 if case=='trigger':spec['dynamic']['trigger']='wall_clock'
 if case=='empty_actor':spec['dynamic']['actor']=None
 if case=='empty_motion':spec['dynamic']['motion']=None
 if case=='duplicate':spec['dynamic_events']*=2
 if case=='nonfinite':spec['dynamic']['position_tolerance_m']=float('nan')
 if case=='unknown_field':spec['dynamic']['hacks']=True
 p=tmp_path/'spec.json';p.write_text(json.dumps(spec))
 assert v.validate_scenario(p)['status']=='INVALID'
def test_ready_requires_real_preparation(spec,tmp_path):
 spec['status']='ready_for_probe';spec['dynamic']['purpose']='probe';spec['dynamic']['trigger']='probe_armed'
 p=tmp_path/'spec.json';p.write_text(json.dumps(spec));assert v.validate_scenario(p)['status']=='INVALID'
def test_version_one_dynamic_still_draft_only(spec,tmp_path):
 spec.pop('dynamic');spec['schema_version']=1;spec['status']='ready_for_manual_smoke'
 p=tmp_path/'spec.json';p.write_text(json.dumps(spec));assert v.validate_scenario(p)['status']=='INVALID'
def test_archived_v2_draft_does_not_depend_on_host_paths(spec):
 for ref in spec['resources'].values():
  if ref:ref['path']='/absent/archive/source'
 v.validate_structure(spec,'scenario','archive');v.scenario_rules(spec,'archive',False)
def test_preparation_fingerprint_changes_with_motion_or_goal(spec):
 first=fingerprint(spec);spec['dynamic']['motion']['sha256']='a'*64;assert fingerprint(spec)!=first
 spec['task']['goal_pose']['x_m']+=.01;assert fingerprint(spec)!=first
@pytest.mark.parametrize('case',['order','speed','nan','wall_period'])
def test_motion_rejects(case):
 m=v.load_strict(ROOT/'experiments/motions/crossing.yaml')
 if case=='order':m['knots'][1]['t_ns']=0
 if case=='speed':m['knots'][2]['t_ns']=2000000001
 if case=='nan':m['knots'][0]['xyz'][0]=float('nan')
 if case=='wall_period':m['period_ns']=100
 with pytest.raises(ValueError):motion_rules(m)
def test_hash_change_rejected(tmp_path):
 p=tmp_path/'data';p.write_text('old');ref={'path':str(p),'sha256':v.digest(p)};p.write_text('new')
 with pytest.raises(ValueError):resource(ref)

def test_prepare_requires_human_ack_without_writes(tmp_path):
 from prepare_dynamic_probe import prepare
 with pytest.raises(ValueError,match='acknowledgement'):
  prepare(ROOT/'experiments/scenarios/p0_06_probe.yaml',ROOT/'experiments/profiles/p0_06_probe.yaml',tmp_path/'no_output',False)
 assert not (tmp_path/'no_output').exists()
def test_static_preparation_and_profile_gate_offline_only(tmp_path,monkeypatch,capsys,current_probe_inputs):
 from prepare_dynamic_probe import prepare
 monkeypatch.setattr(run_trial.subprocess,'Popen',lambda *a,**k:pytest.fail('preparation cannot launch'))
 output=tmp_path/'fixture_preparation'
 paths=prepare(*current_probe_inputs,output,True)
 spec=v.load_strict(paths['scenario']);profile=v.load_strict(paths['profile'])
 assert v.validate_scenario(paths['scenario'])['status']=='VALID'
 assert v.validate_scenario(paths['scenario'],True)['status']=='NOT_READY'
 p,_=run_trial.load_inputs(paths['scenario'],paths['profile'],'mppi','p0-06-fixture-check')
 assert p['mode']=='probe'
 monkeypatch.setattr(sys,'argv',['run_trial','--scenario',paths['scenario'],'--profile',paths['profile'],'--method','mppi','--run-id','p0-06-fixture-check','--check'])
 assert run_trial.main()==0
 plan=json.loads(capsys.readouterr().out)
 assert plan['sends_goal'] is False and {'scene-driver','scene-capture'}<=set(plan['processes'])
 assert not (ROOT/'evidence/runs/p0-06-fixture-check').exists()
 spec['status']='ready_for_recording';spec['dynamic']['purpose']='navigation_recording';spec['dynamic']['trigger']='goal_accepted'
 with pytest.raises(ValueError):v.scenario_rules(spec,'fixture')
def test_v2_snapshot_archive_is_self_contained(tmp_path):
 from prepare_dynamic_probe import prepare
 from dynamic_contract import freeze_inputs,archive_rules
 paths=prepare(ROOT/'experiments/scenarios/p0_06_probe.yaml',ROOT/'experiments/profiles/p0_06_probe.yaml',tmp_path/'fixture_preparation',True)
 original=v.load_strict(paths['scenario'])
 run=tmp_path/'fixture_archive';(run/'artifacts').mkdir(parents=True)
 frozen=freeze_inputs(run,original)
 assert fingerprint(original)==fingerprint(frozen)
 v.validate_structure(frozen,'scenario','fixture')
 v.scenario_rules(frozen,'fixture',False)
 refs={str(i):({'sha256':v.digest(p)},p) for i,p in enumerate((run/'artifacts').iterdir())}
 archive_rules(frozen,refs,run,'fixture')
 key=next(k for k,vv in refs.items() if vv[0]['sha256']==frozen['dynamic']['motion']['sha256']);refs.pop(key)
 with pytest.raises(ValueError,match='archived'):archive_rules(frozen,refs,run,'fixture')
def test_v2_cli_keeps_invalid_exit_two(tmp_path):
 import subprocess
 s=v.load_strict(ROOT/'experiments/scenarios/crossing.yaml');s['dynamic']['entity_name']='wrong'
 path=tmp_path/'fixture.json';path.write_text(json.dumps(s))
 r=subprocess.run([sys.executable,'-B',str(ROOT/'tools/validate_run.py'),'scenario',str(path)],capture_output=True,text=True,timeout=5)
 assert r.returncode==2 and json.loads(r.stdout)['status']=='INVALID'

def test_probe_preparation_binding_excludes_only_execution_purpose(spec):
 first=fingerprint(spec);spec['dynamic']['purpose']='probe';spec['dynamic']['trigger']='probe_armed'
 assert fingerprint(spec)==first
 spec['dynamic']['delay_ns']+=1
 assert fingerprint(spec)!=first
def test_occlusion_can_prepare_its_own_probe_without_editing_formal_draft(tmp_path):
 from prepare_dynamic_probe import prepare
 before=(ROOT/'experiments/scenarios/observed_occlusion.yaml').read_bytes()
 draft=v.load_strict(ROOT/'experiments/scenarios/observed_occlusion.yaml')
 draft['status']='draft';draft['dynamic']['preparation']={'review':None,'probe':None}
 source=tmp_path/'fixture_s3_draft.json';source.write_text(json.dumps(draft))
 paths=prepare(source,ROOT/'experiments/profiles/p0_06_observed_occlusion.yaml',tmp_path/'fixture_s3_probe',True,as_probe=True)
 s=v.load_strict(paths['scenario'])
 assert s['kind']=='observed_occlusion' and s['dynamic']['purpose']=='probe' and s['dynamic']['trigger']=='probe_armed'
 assert (ROOT/'experiments/scenarios/observed_occlusion.yaml').read_bytes()==before

@pytest.mark.parametrize('field',['trigger','motion_ref'])
def test_legacy_event_conflicts_rejected(spec,field):
 spec['dynamic_events'][0][field]={'time_domain':'ros_sim','at_s':0} if field=='trigger' else '/other/motion'
 with pytest.raises(ValueError):v.scenario_rules(spec,'fixture',False)
