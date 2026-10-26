import sys,copy,queue,io,time
from pathlib import Path
import pytest
from tools.runner.runner_faults import *

def profile(f):return {'faults':[f] if f else [],'p0_08':{'version':1,'fault':f,'policy':POLICY.copy(),'raw_gazebo':True,'arrival_policy':'first_fresh_post_terminal_0.5s'}}
@pytest.mark.parametrize('f',[None,*FAULTS])
def test_policy_exact(f):
 p=profile(f);validate_policy(p,f);p['p0_08']['policy']['pause_wall_s']=0
 with pytest.raises(ValueError):validate_policy(p,f)
def test_legacy_and_unknown_whitelist():
 validate_policy({},None);validate_policy({},'withhold-initial-pose')
 with pytest.raises(ValueError):validate_policy({},'pause-gazebo')
 with pytest.raises(ValueError):validate_policy(profile('pause-gazebo'),'short-deadline')
def test_resume_not_dependent_on_worker_rpc(tmp_path):
 g=GazeboEvidence.__new__(GazeboEvidence);g.queue=queue.Queue();g.file=io.StringIO();g.resume_due=time.monotonic()-1;g.resume_error=None;g.resume_wall=None;calls=[];g.control=lambda paused:calls.append(paused);g.log=lambda *x:None
 g.tick();g.tick();assert calls==[False] and g.resume_wall is not None

def test_resume_failure_preserved():
 g=GazeboEvidence.__new__(GazeboEvidence);g.queue=queue.Queue();g.file=io.StringIO();g.resume_due=time.monotonic()-1;g.resume_error=None;g.log=lambda *x:None
 g.control=lambda x:(_ for _ in ()).throw(RuntimeError('service lost'));g.tick();assert g.resume_error=='service lost' and g.resume_due is None
