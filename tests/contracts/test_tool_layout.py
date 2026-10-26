"""Directory-v2 contracts and detached package checks; no ROS initialization."""
import ast
import hashlib
import importlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from unittest.mock import patch
import pytest
from tools import PROJECT_ROOT as ROOT
from tools.runner.archive import snapshot_code

CLI = ['run_trial','run_trial_observe','run_smoke_matrix','validate_run',
       'summarize_runs','prepare_geometry_spec','evaluate_geometry','plot_geometry']
STAGES = ['prepare_dynamic_probe.py','make_baselines.py','p0_03_navigation.py',
          'p0_06_host_trial.py','p0_03_env.bash']

def test_root_and_stage_inventory():
    assert {p.stem for p in (ROOT/'tools').glob('*.py')} == set(CLI)|{'__init__'}
    assert {p.name for p in (ROOT/'tools/stages').iterdir()
            if p.is_file() and p.name!='__init__.py'} == set(STAGES)
    assert not any(p.is_symlink() for p in (ROOT/'tools').rglob('*'))
    for name in CLI:
        tree=ast.parse((ROOT/'tools'/(name+'.py')).read_text())
        assert not any(isinstance(n,(ast.FunctionDef,ast.ClassDef)) for n in tree.body)

def test_internal_dependencies_do_not_import_root_cli():
    forbidden={'tools.'+n for n in CLI}
    for path in (ROOT/'tools').rglob('*.py'):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node,ast.ImportFrom):
                assert node.module not in forbidden,(path,node.module)
            if isinstance(node,ast.Import):
                assert all(a.name not in forbidden for a in node.names),path
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute):
                assert ast.unparse(node.func) not in ['sys.path.insert','sys.path.append'],path

def test_gui_uses_one_runner_implementation(monkeypatch):
    from tools.runner import trial,observe
    assert observe.runner is trial
    original=(trial.ProcessSupervisor,trial.WorkerLink)
    def main():
        assert (trial.ProcessSupervisor,trial.WorkerLink)==original
        return 17
    monkeypatch.setattr(trial,'main',main)
    assert observe.main(['--no-gazebo-gui'])==17

def test_baseline_generator_import_is_read_only():
    paths=list((ROOT/'src/cmr_bringup/config').glob('*.yaml'))
    before={str(p):p.read_bytes() for p in paths}
    import tools.navigation.baselines as implementation
    with patch.object(Path,'write_text',side_effect=AssertionError('import wrote a file')):
        importlib.reload(implementation)
        importlib.reload(importlib.import_module('tools.stages.make_baselines'))
    assert before=={str(p):p.read_bytes() for p in paths}
    assert callable(implementation.main)

@pytest.fixture
def detached(tmp_path):
    source=tmp_path/'run';(source/'artifacts').mkdir(parents=True)
    layout=snapshot_code(source)
    destination=tmp_path/'detached'
    shutil.move(str(source/'artifacts'),destination)
    for relative,digest in layout['files'].items():
        assert hashlib.sha256((destination/relative).read_bytes()).hexdigest()==digest
    return destination

@pytest.mark.parametrize('name',CLI)
def test_detached_root_cli_help(detached,name):
    env=os.environ.copy();env['PYTHONPATH']='';env['PYTHONDONTWRITEBYTECODE']='1'
    p=subprocess.run([sys.executable,'-B',str(detached/'tools'/(name+'.py')),'--help'],
                     cwd=detached.parent,capture_output=True,text=True,env=env,timeout=15)
    assert p.returncode==(2 if name=='validate_run' else 0),(p.stdout,p.stderr)

@pytest.mark.parametrize('relative',[
    'navigation/nav_goal.py','runner/inspect_bag.py',
    'dynamic/inspect_dynamic_recording.py','stages/prepare_dynamic_probe.py',
    'stages/p0_03_navigation.py','stages/p0_06_host_trial.py'])
def test_nested_entry_from_outside(detached,relative):
    # manual navigation imports Jazzy at module load, as before.
    env=os.environ.copy();env['PYTHONDONTWRITEBYTECODE']='1'
    p=subprocess.run([sys.executable,'-B',str(detached/'tools'/relative),'--help'],
                     cwd=detached.parent,capture_output=True,text=True,env=env,timeout=15)
    assert p.returncode==0,p.stderr

def test_detached_validator_schema_closure(detached):
    env=os.environ.copy();env['PYTHONPATH']='';env['PYTHONDONTWRITEBYTECODE']='1'
    run=ROOT/'tests/fixtures/runs/succeeded_unknown'
    p=subprocess.run([sys.executable,'-B',str(detached/'tools/validate_run.py'),
                      'run',str(run),'--allow-fixtures'],cwd=detached.parent,
                      env=env,capture_output=True,text=True,timeout=15)
    assert p.returncode==0,p.stderr
    assert json.loads(p.stdout)['status']=='VALID'

def test_archive_module_roots_are_detached(detached):
    code=(
        "from runpy import run_path;from pathlib import Path;"
        "run_path("+repr(str(detached/'tools/__init__.py'))+")['bootstrap']();"
        "from tools import PROJECT_ROOT;from tools.runner import trial,observe;"
        "from tools.geometry import geometry_inputs;"
        "assert PROJECT_ROOT==Path("+repr(str(detached))+");"
        "assert geometry_inputs.ROOT==PROJECT_ROOT;"
        "assert (PROJECT_ROOT/'src/cmr_bringup/config/fastdds_loopback_udp.xml').is_file();"
        "assert observe.runner is trial"
    )
    env=os.environ.copy();env['PYTHONPATH']=''
    p=subprocess.run([sys.executable,'-B','-c',code],cwd=detached.parent,
                     env=env,capture_output=True,text=True,timeout=15)
    assert p.returncode==0,p.stderr

def test_source_environment_entry_outside_cwd(tmp_path):
    p=subprocess.run(['bash','--noprofile','--norc','-c',
                      'source "$1"; python3 -c \'import os;print(os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"]);print(os.environ["ROS_DOMAIN_ID"])\'',
                      'test',str(ROOT/'tools/stages/p0_03_env.bash')],
                      cwd=tmp_path,capture_output=True,text=True,timeout=10)
    assert p.returncode==0,p.stderr
    assert p.stdout.splitlines()==[str(ROOT/'src/cmr_bringup/config/fastdds_loopback_udp.xml'),'71']

def test_new_dynamic_inputs_are_explicit_drafts():
    from tools.contracts.validation import validate_scenario
    folder=ROOT/'experiments/tool_layout_v2'
    for path in (folder/'scenarios').glob('*.yaml'):
        data=json.loads(path.read_text())
        assert data['status']=='draft'
        assert data['dynamic']['preparation']=={'review':None,'probe':None}
        assert validate_scenario(path)['status']=='VALID'
        lock=json.loads(Path(data['resources']['dependency_lock']['path']).read_text())
        for ref in lock['resources']:
            assert hashlib.sha256(Path(ref['path']).read_bytes()).hexdigest()==ref['sha256']

def test_check_remains_read_only(monkeypatch,capsys):
    from tools.runner import trial
    before=set((ROOT/'evidence/runs').iterdir())
    monkeypatch.setattr(trial.subprocess,'Popen',lambda *a,**k:pytest.fail('check spawned process'))
    monkeypatch.setattr(sys,'argv',['run_trial.py','--scenario',str(ROOT/'experiments/scenarios/p0_long_route_static_v1.yaml'),
                                   '--profile',str(ROOT/'experiments/profiles/p0_long_route_static_v1.yaml'),
                                   '--method','mppi','--run-id','tools-v2-check-only','--check'])
    assert trial.main()==0
    assert json.loads(capsys.readouterr().out)['check_has_no_side_effects']
    assert before==set((ROOT/'evidence/runs').iterdir())

def test_detached_geometry_precheck(detached):
    """Archived code validates an explicit old Run without importing workspace code."""
    run=ROOT/'evidence/runs/p0-long-crossing-low-20261029-a1'
    spec=ROOT/'experiments/evaluations/p0-long-crossing-low-20261029-a1.json'
    env=os.environ.copy();env['PYTHONPATH']='';env['PYTHONDONTWRITEBYTECODE']='1'
    process=subprocess.run([sys.executable,'-B',str(detached/'tools/evaluate_geometry.py'),
                            '--run-dir',str(run),'--spec',str(spec),'--check'],
                            cwd=detached.parent,env=env,capture_output=True,text=True,timeout=30)
    assert process.returncode==0,process.stderr
    assert json.loads(process.stdout)['coverage']=='full'
