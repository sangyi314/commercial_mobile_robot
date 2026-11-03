# SPDX-License-Identifier: MIT
import os,sys,signal
import pytest
from tools.runner.runner_processes import ProcessSupervisor
from tools.runner.runner_core import DeadlineBudget

def test_only_predeclared_owned_mission_kill_is_expected(tmp_path):
 (tmp_path/'logs').mkdir();events=[];s=ProcessSupervisor(tmp_path,lambda k,v:events.append((k,v)));p=s.spawn('mission',[sys.executable,'-c','import time;time.sleep(10)'],os.environ.copy(),tmp_path)
 s.expect_mission_fault_exit(p,'G06: qualified fixture owner fault');p.critical=False;s.signal_identity(p.birth,signal.SIGKILL);p.process.wait(timeout=2);r=s.close(p,DeadlineBudget.after(2),initial_signal=False,grace=.1)
 assert r['ok'] and r['exit']==-9 and not r['forced'] and r['expected_exit_reason'].startswith('G06:')
 assert events[-2][0]=='EXPECTED_MISSION_FAULT_EXIT'
 with pytest.raises(ValueError):s.expect_mission_fault_exit(p,'G06: too late')

def test_gate_fault_cannot_be_relabelled_expected(tmp_path):
 (tmp_path/'logs').mkdir();s=ProcessSupervisor(tmp_path);p=s.spawn('execution-gate',[sys.executable,'-c','import time;time.sleep(10)'],os.environ.copy(),tmp_path)
 try:
  with pytest.raises(ValueError):s.expect_mission_fault_exit(p,'G06: forbidden Gate failure')
 finally:s.close(p,DeadlineBudget.after(2),grace=.2)
