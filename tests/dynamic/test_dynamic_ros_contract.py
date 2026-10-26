"""Isolated domain72 worker regressions; no Gazebo or navigation trial."""
import json,os,sys
from pathlib import Path
import pytest
pytestmark = pytest.mark.ros_contract
from ..runner.test_runner_ros_contract import ros,Fake
from tools.runner.runner_processes import ProcessSupervisor
from tools.runner.runner_core import DeadlineBudget, write_json
from tools.runner.trial import WorkerLink
@pytest.fixture
def probe_worker(ros,tmp_path):
 import uuid
 rclpy,ex=ros;ns='probe_'+uuid.uuid4().hex[:8];fake=Fake(rclpy,ex,ns)
 root=Path(__file__).resolve().parents[2];run=tmp_path/('fixture_'+ns)
 for d in ['logs','runtime','artifacts']:(run/d).mkdir(parents=True)
 profile=json.loads((root/'experiments/runner_profile.yaml').read_text());profile.update({'mode':'probe','selected_method':'mppi','synthetic_test':True,'test_namespace':ns})
 write_json(run/'artifacts/profile.json',profile);(run/'artifacts/goal.json').write_bytes(Path(profile['files']['goal']['path']).read_bytes())
 supervisor=ProcessSupervisor(run);item=supervisor.spawn('worker',[sys.executable,'-B',str(root/'tools/navigation/nav_goal.py'),'--run-dir',str(run),'--test-namespace',ns],os.environ.copy(),root,ipc=True);link=WorkerLink(item,run.name,lambda *x:None,supervisor)
 try:
  link.rpc('sensors',{},DeadlineBudget.after(3));yield fake,run,link
 finally:
  item.critical=False
  try:link.rpc('exit',{},DeadlineBudget.after(2))
  except Exception:pass
  supervisor.close(item,DeadlineBudget.after(3),initial_signal=False,grace=.5);fake.close()
def test_probe_worker_rejects_send_before_action_client(probe_worker):
 fake,run,link=probe_worker
 with pytest.raises(RuntimeError,match='probe forbids'):link.rpc('send',{'uuid':'b'*32},DeadlineBudget.after(1))
 assert fake.count==0 and not (run/'runtime/goal-dispatch.json').exists()
 assert not (run/'logs/action-client.jsonl').read_text()
def test_probe_observation_needs_real_samples(probe_worker):
 fake,run,link=probe_worker
 value=link.rpc('probe_observe',{'duration_sim_s':.25},DeadlineBudget.after(2))
 assert value['robot_stationary'] and value['samples']>=5 and fake.count==0
def test_probe_pause_is_bounded_and_does_not_dispatch(probe_worker):
 fake,run,link=probe_worker
 assert link.rpc('probe_pause',{'enter':True},DeadlineBudget.after(1))['pause_check']
 assert not link.rpc('probe_pause',{'enter':False},DeadlineBudget.after(1))['pause_check']
 assert fake.count==0

def test_recording_marker_has_bounded_reliable_drain(probe_worker):
 fake,run,link=probe_worker
 from std_msgs.msg import String
 seen=[]
 sub=fake.n.create_subscription(String,'cmr_experiment/events',lambda msg:seen.append(json.loads(msg.data)),100)
 value=link.rpc('mark',{'name':'RECORDING_END'},DeadlineBudget.after(2))
 assert value['marked'] and value['reliable_acknowledged'] and value['source_clock_advance_s']>=.1
 assert value['disk_content_verified'] is False and fake.count==0
 assert any(x['type']=='RECORDING_END' for x in seen)
 fake.n.destroy_subscription(sub)
