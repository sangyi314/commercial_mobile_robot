import os,signal,subprocess,sys,time
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from runner_core import DeadlineBudget
from runner_processes import ProcessSupervisor,ProjectLock,identity

def supervise(tmp_path,code):
 (tmp_path/'logs').mkdir(exist_ok=True);s=ProcessSupervisor(tmp_path);p=s.spawn('child',[sys.executable,'-u','-c',code],os.environ.copy(),tmp_path);return s,p

def ready(path):
 end=time.monotonic()+2
 while not path.exists() and time.monotonic()<end:time.sleep(.01)
 assert path.exists()
def test_normal_idempotent_and_sentinel(tmp_path):
 sentinel=subprocess.Popen([sys.executable,'-c','import time;time.sleep(20)'],start_new_session=True)
 try:
  s,p=supervise(tmp_path,"import signal,time,pathlib;signal.signal(signal.SIGINT,lambda *a:exit(0));pathlib.Path('ready').touch();time.sleep(20)");ready(tmp_path/'ready');r=s.close(p,DeadlineBudget.after(3),grace=1);assert r['ok'] and s.close(p,DeadlineBudget.after(1))==r;assert sentinel.poll() is None
 finally:sentinel.terminate();sentinel.wait(timeout=2)
@pytest.mark.parametrize('ignore_term',[False,True])
def test_escalation_keeps_failure(tmp_path,ignore_term):
 code="import signal,time,pathlib;signal.signal(signal.SIGINT,signal.SIG_IGN);"+("signal.signal(signal.SIGTERM,signal.SIG_IGN);" if ignore_term else '')+"pathlib.Path('ready').touch();time.sleep(20)"
 s,p=supervise(tmp_path,code);ready(tmp_path/'ready');r=s.close(p,DeadlineBudget.after(5),grace=.05);assert r['forced'] and not r['ok'] and not r['remaining']
def test_leader_exit_orphan_checked(tmp_path):
 code="import subprocess,sys;subprocess.Popen([sys.executable,'-c','import time;time.sleep(20)']);sys.exit(0)"
 s,p=supervise(tmp_path,code);p.process.wait(timeout=2);assert s.members(p);r=s.close(p,DeadlineBudget.after(4),grace=.05);assert r['forced'] and not r['remaining']
def test_large_log_no_pipe_deadlock(tmp_path):
 s,p=supervise(tmp_path,"import sys;sys.stdout.write('x'*2000000)");p.process.wait(timeout=2);r=s.close(p,DeadlineBudget.after(2),initial_signal=False,grace=.05);assert r['ok'] and (tmp_path/'logs/child.log').stat().st_size==2000000
def test_crash_is_not_success(tmp_path):
 s,p=supervise(tmp_path,'import os;os._exit(7)');p.process.wait(timeout=2);assert not s.close(p,DeadlineBudget.after(1),initial_signal=False,grace=.05)['ok']
def test_pid_reuse_refused(tmp_path,monkeypatch):
 import runner_processes
 s,p=supervise(tmp_path,"import time,pathlib;pathlib.Path('ready').touch();time.sleep(20)");ready(tmp_path/'ready');real=runner_processes.identity
 try:
  monkeypatch.setattr(runner_processes,'identity',lambda pid:{**real(pid),'start_ticks':real(pid)['start_ticks']+1})
  with pytest.raises(RuntimeError):s.signal_identity(p.birth,signal.SIGINT)
  assert p.process.poll() is None
 finally:monkeypatch.setattr(runner_processes,'identity',real);s.close(p,DeadlineBudget.after(3),grace=.2)
def test_argv_no_shell_and_lock(tmp_path):
 literal='中文 path ; $(touch INJECTION)'
 (tmp_path/'logs').mkdir();s=ProcessSupervisor(tmp_path);p=s.spawn('argv',[sys.executable,'-c','import sys;print(sys.argv[1])',literal],os.environ.copy(),tmp_path,False);p.process.wait(timeout=2);s.close(p,DeadlineBudget.after(1),False,.05);assert literal in (tmp_path/'logs/argv.log').read_text() and not (tmp_path/'INJECTION').exists()
 with ProjectLock(tmp_path/'lock'):
  with pytest.raises(RuntimeError):
   with ProjectLock(tmp_path/'lock'):pass


def test_descendant_changes_session_still_owned(tmp_path):
 code="import subprocess,sys;subprocess.Popen([sys.executable,'-c','import os,time;os.setsid();time.sleep(20)']);sys.exit(0)"
 s,p=supervise(tmp_path,code);p.process.wait(timeout=2);end=time.monotonic()+1
 while not s.members(p) and time.monotonic()<end:time.sleep(.01)
 assert s.members(p);r=s.close(p,DeadlineBudget.after(4),grace=.05);assert r['forced'] and not r['remaining']


def test_recorder_early_exit_detected_even_when_exit_zero(tmp_path):
 s,p=supervise(tmp_path,'pass');p.role='recorder';p.process.wait(timeout=2)
 with pytest.raises(RuntimeError,match='critical process'):s.assert_critical()
 s.close(p,DeadlineBudget.after(1),initial_signal=False,grace=.05)

def test_worker_oversized_output_fails_bounded(tmp_path):
 from run_trial import WorkerLink
 (tmp_path/'logs').mkdir();s=ProcessSupervisor(tmp_path);p=s.spawn('worker',[sys.executable,'-c',"import os;os.write(1,b'x'*70000)"],os.environ.copy(),tmp_path,ipc=True);link=WorkerLink(p,'fixture',lambda *x:None,s)
 try:
  end=DeadlineBudget.after(2)
  with pytest.raises((ValueError,RuntimeError)):
   while True:end.require();link.pump()
 finally:s.close(p,DeadlineBudget.after(2),grace=.1)

def test_worker_no_heartbeat_has_wall_deadline(tmp_path):
 from run_trial import WorkerLink
 (tmp_path/'logs').mkdir();s=ProcessSupervisor(tmp_path);p=s.spawn('worker',[sys.executable,'-c','import time;time.sleep(20)'],os.environ.copy(),tmp_path,ipc=True);link=WorkerLink(p,'fixture',lambda *x:None,s);link.last_heartbeat=time.monotonic()-4
 try:
  with pytest.raises(TimeoutError,match='heartbeat'):link.pump()
 finally:s.close(p,DeadlineBudget.after(2),grace=.1)
