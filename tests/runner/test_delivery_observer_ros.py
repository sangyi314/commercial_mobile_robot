# SPDX-License-Identifier: MIT
"""Actual isolated ROS worker ownership proof; never a physical motion proof."""
import json,os,sys,uuid
from pathlib import Path
import pytest
from tests.runner.test_runner_ros_contract import ros,Fake,until
from tools.runner.runner_core import DeadlineBudget,write_json
from tools.runner.runner_processes import ProcessSupervisor
from tools.runner.trial import WorkerLink
pytestmark=pytest.mark.ros_contract
def test_actual_observer_worker_has_no_goal_client_and_rejects_commands(ros,tmp_path):
 rclpy,ex=ros;ns='delivery_guard_'+uuid.uuid4().hex[:8];fake=Fake(rclpy,ex,ns);run=tmp_path/('fixture_'+ns)
 for d in ['logs','runtime','artifacts']:(run/d).mkdir(parents=True)
 root=Path(__file__).resolve().parents[2];profile=json.loads((root/'experiments/runner_profile.yaml').read_text());profile.update(selected_method='mppi',synthetic_test=True,test_namespace=ns,delivery_v1={'version':1,'synthetic_observations':True})
 write_json(run/'artifacts/profile.json',profile)
 supervisor=ProcessSupervisor(run);worker=supervisor.spawn('worker',[sys.executable,'-B',str(root/'tools/navigation/nav_goal.py'),'--run-dir',str(run),'--test-namespace',ns,'--mission-observer'],os.environ.copy(),root,ipc=True);link=WorkerLink(worker,run.name,lambda *a:None,supervisor)
 try:
  link.rpc('sensors',{},DeadlineBudget.after(4))
  for op in ['send','cancel','fault_ready','deactivate_controller','fault_deactivate','failure_stop']:
   with pytest.raises(RuntimeError,match='MISSION_OBSERVER_FORBIDS_GOAL_CONTROL'):link.rpc(op,{'uuid':uuid.uuid4().hex},DeadlineBudget.after(2))
  assert fake.count==fake.cancels==0
  clients=fake.n.get_client_names_and_types_by_node('cmr_worker','/'+ns)
  assert not any(name.endswith('/_action/send_goal') or name.endswith('/_action/cancel_goal') for name,types in clients)
  assert json.loads((run/'runtime/worker-mode.json').read_text())['action_client_created'] is False
 finally:
  worker.critical=False
  try:link.rpc('exit',{},DeadlineBudget.after(2))
  finally:
   result=supervisor.close(worker,DeadlineBudget.after(3),initial_signal=False,grace=1);fake.close()
  assert result['ok'] and not result['remaining']
