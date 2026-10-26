"""GUI observation counterexamples; no navigation or Gazebo is launched here."""
import io,json,os,pty,sys,time
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import run_trial as runner
import run_trial_observe as observe
from runner_gui import GUIOptions,GUIClient,GUIFailure,gui_argv,log_readiness
from runner_core import DeadlineBudget
R=Path(__file__).resolve().parents[1]

@pytest.mark.parametrize('kwargs',[{'gazebo_gui':1},{'wait_for_operator':True},{'operator_timeout_s':0},{'operator_timeout_s':float('nan')},{'operator_timeout_s':float('inf')},{'operator_timeout_s':121},{'gui_startup_timeout_s':61},{'gui_startup_timeout_s':-1},{'operator_timeout_s':True}])
def test_invalid_options(kwargs):
 with pytest.raises(ValueError):GUIOptions(**kwargs).validate()
def test_defaults_and_gui_only_argv():
 o=GUIOptions().validate();assert not o.gazebo_gui and not o.wait_for_operator and o.plan()['process_roles']==[]
 cmd=gui_argv();assert cmd==['stdbuf','-oL','-eL','gz','sim','-g','-v','4','--render-engine-gui','ogre'];assert '-s' not in cmd and '-r' not in cmd and not any(x.endswith('.sdf') for x in cmd)
@pytest.mark.parametrize('kind',['legacy','long','dynamic'])
@pytest.mark.parametrize('flags',[[],['--gazebo-gui'],['--gazebo-gui','--wait-for-operator']])
def test_check_zero_side_effects_and_old_plan(kind,flags,monkeypatch,capsys):
 pair={'legacy':('experiments/scenarios/p0_05_official_static.yaml','experiments/runner_profile.yaml'),'long':('experiments/scenarios/p0_long_route_static_v1.yaml','experiments/profiles/p0_long_route_static_v1.yaml'),'dynamic':('experiments/scenarios/crossing.yaml','experiments/profiles/p0_06_crossing.yaml')}[kind]
 before=set((R/'evidence/runs').iterdir());monkeypatch.setattr(runner.subprocess,'Popen',lambda *a,**k:pytest.fail('--check cannot spawn'))
 args=['--scenario',str(R/pair[0]),'--profile',str(R/pair[1]),'--method','mppi','--run-id','p0-gui-fixture-check','--check']
 assert observe.main(args+flags)==0;plan=json.loads(capsys.readouterr().out);assert plan['budgets']['goal']==120 and plan['budgets']['work']==360
 assert plan['observation']['gazebo_gui']==('--gazebo-gui' in flags)
 assert ('gazebo-gui' in plan['processes'])==('--gazebo-gui' in flags)
 assert before==set((R/'evidence/runs').iterdir())
@pytest.mark.parametrize('flags',[['--wait-for-operator'],['--operator-timeout','nan'],['--operator-timeout','inf'],['--gui-startup-timeout','0']])
def test_invalid_cli_rejected_before_original(flags,monkeypatch):
 monkeypatch.setattr(runner,'main',lambda:pytest.fail('invalid settings cannot enter runner'))
 assert observe.main(flags)==2

def test_disabled_entry_delegates_without_adapters(monkeypatch):
 classes=(runner.ProcessSupervisor,runner.WorkerLink);argv=sys.argv
 def old():
  assert (runner.ProcessSupervisor,runner.WorkerLink)==classes;assert sys.argv[1:]==['--profile','unchanged'];return 17
 monkeypatch.setattr(runner,'main',old);assert observe.main(['--profile','unchanged'])==17;assert sys.argv is argv

def test_adapter_scope_restores_even_on_interrupt():
 classes=(runner.ProcessSupervisor,runner.WorkerLink)
 with pytest.raises(KeyboardInterrupt):
  with observe.gui_adapters(GUIOptions(True)):
   assert runner.ProcessSupervisor is not classes[0];raise KeyboardInterrupt()
 assert (runner.ProcessSupervisor,runner.WorkerLink)==classes

def test_gui_handshake_requires_all_positive_markers():
 text=json.loads((R/'tests/fixtures/gazebo_gui_logs.json').read_text())['ready']
 proof=log_readiness(text);assert proof['complete']
 import re
 plain=re.sub(r'\x1b\[[0-9;]*m','',text)
 for marker in proof['markers']:assert not log_readiness(plain.replace(marker,''))['complete']
 assert not log_readiness('GUI process exists; main window created')['complete']

class Proc:
 def __init__(self,code=None):self.code=code
 def poll(self):return self.code
class Item:
 def __init__(self,code=None):self.process=Proc(code);self.role='gazebo-gui';self.closed=False;self.result=None
class FakeSupervisor:
 def __init__(self,run):self.run=run;self.calls=[];self.emit=lambda *a:None
 def spawn(self,*a,**kw):self.calls.append((a,kw));return Item()
@pytest.fixture
def client(tmp_path):
 for d in ['artifacts','runtime','logs']:(tmp_path/d).mkdir()
 s=FakeSupervisor(tmp_path);c=GUIClient(s,GUIOptions(True,True,.5,20),lambda *a:None);return c

def test_server_environment_partition_and_rendering_inherited(client):
 env={'GZ_PARTITION':'cmr_unique','DISPLAY':':42','XAUTHORITY':'/actual/authority','LIBGL_ALWAYS_SOFTWARE':'1','LD_PRELOAD':'/verified/preload'};client.start(env,R)
 sent=client.supervisor.calls[0][0][2];assert all(sent[k]==v for k,v in env.items());assert sent['QT_QPA_PLATFORM']=='xcb';assert env.get('QT_QPA_PLATFORM') is None
 assert client.supervisor.calls[0][1]['critical'] is False

def test_spawn_failure_is_explicit(client):
 def bad(*a,**k):raise OSError('cannot execute')
 client.supervisor.spawn=bad
 with pytest.raises(GUIFailure,match='GUI_START_FAILED'):client.start({'GZ_PARTITION':'x','DISPLAY':':1','LIBGL_ALWAYS_SOFTWARE':'1'},R)
 report=json.loads((client.run/'runtime/gui-status.json').read_text());assert not report['ready'] and report['failure']['code']=='GUI_START_FAILED'
@pytest.mark.parametrize('ready',[False,True])
def test_early_exit_never_masquerades_as_ready(client,ready):
 client.item=Item(0);client.ready=ready
 with pytest.raises(GUIFailure,match='GUI_EARLY_EXIT'):client.tick()
 assert not client.monitor and client.failure['code']=='GUI_EARLY_EXIT'
 client.tick()  # failure emitted once; original cleanup can continue pumping

def test_gui_readiness_deadline_is_not_process_existence(client):
 client.item=Item();client.start_deadline=time.monotonic()-1
 with pytest.raises(GUIFailure,match='GUI_READY_TIMEOUT'):client.tick()
 assert not client.ready

def test_operator_requires_ready(client):
 with pytest.raises(GUIFailure,match='GUI_NOT_READY'):client.await_operator(None)

def test_non_tty_input_is_rejected(client):
 client.ready=True;rd,wr=os.pipe()
 try:
  with pytest.raises(GUIFailure,match='OPERATOR_TTY_REQUIRED'):client.await_operator(None,fd=rd)
 finally:os.close(rd);os.close(wr)

@pytest.mark.parametrize('mode',['enter','timeout','stale','heartbeat','exit','nonempty','exit_in_rpc','heartbeat_in_rpc'])
def test_operator_wait_pumps_checks_and_has_finite_deadline(client,mode):
 client.ready=True;master,slave=pty.openpty();now=[10.];commands=[];pumps=[]
 class Link:
  def pump(self):
   now[0]+=.1;pumps.append(1)
   if mode=='heartbeat':raise TimeoutError('worker heartbeat timeout')
   if mode=='exit':client.item=Item(0);client.tick()
   if len(pumps)==1 and mode=='nonempty':os.write(master,b'go\n')
   if (mode=='enter' and len(pumps)==2) or (mode=='nonempty' and len(pumps)==3):os.write(master,b'\n')
  def rpc(self,op,args,budget):
   commands.append(op)
   if mode=='stale':raise RuntimeError('sensors: clock/scan/odom/TF readiness')
   if mode=='exit_in_rpc':client.item=Item(0);client.tick()
   if mode=='heartbeat_in_rpc':raise TimeoutError('worker heartbeat timeout')
   return {}
 try:
  if mode in ('enter','nonempty'):
   client.await_operator(Link(),fd=slave,clock=lambda:now[0]);assert client.operator=='confirmed' and 'sensors' in commands
  else:
   expected={'timeout':'OPERATOR_WAIT_TIMEOUT','stale':'OPERATOR_INPUT_STALE','heartbeat':'heartbeat','exit':'GUI_EARLY_EXIT','exit_in_rpc':'GUI_EARLY_EXIT','heartbeat_in_rpc':'heartbeat'}[mode]
   with pytest.raises((GUIFailure,TimeoutError),match=expected):client.await_operator(Link(),fd=slave,clock=lambda:now[0])
  assert pumps and all(op=='sensors' for op in commands)  # no pause / send / timeout changes
  assert os.get_blocking(slave)
 finally:os.close(master);os.close(slave)

def test_wait_timeout_has_no_dispatch_intent(client):
 assert not (client.run/'runtime/dispatch-intent.json').exists()
 # Integration ordering is enforced by the adapter's nav_ready interception.
 import inspect
 src=inspect.getsource(observe.gui_adapters)
 assert "op=='nav_ready'" in src and 'await_operator' in src and "op=='send'" not in src

@pytest.mark.parametrize('case',['hidden','wrong_owner','unmapped','mapped'])
def test_owned_mapped_window_is_required_beyond_renderer_logs(client,case,monkeypatch):
 from types import SimpleNamespace
 import runner_gui
 markers=log_readiness('')['markers'];(client.run/'logs/gazebo-gui.log').write_text('\n'.join(markers))
 client.item=Item();client.env={'GZ_PARTITION':'fixture'};client.start_deadline=time.monotonic()+10;client.window={'id':'0x42','width':800,'height':600}
 content={'hidden':'0x42 "Gazebo GUI": () 1x1+0+0','wrong_owner':'_NET_WM_PID(CARDINAL) = 99','unmapped':'Map State: IsUnMapped','mapped':'Map State: IsViewable'}[case]
 step={'hidden':'tree','wrong_owner':'pid','unmapped':'mapped','mapped':'mapped'}[case]
 file=client.run/'logs/window.log';file.write_text(content);probe=Item(0);probe.logfile=SimpleNamespace(name=str(file));client.probe=probe;client.probe_step=step
 client.supervisor.members=lambda item:[{'pid':42}]
 monkeypatch.setattr(runner_gui.BaseSupervisor,'close',lambda *a,**k:{'ok':True})
 if case=='wrong_owner':
  with pytest.raises(GUIFailure,match='OWNER_MISMATCH'):client.tick()
 else:
  client.tick();assert client.window_ready==(case=='mapped');assert not client.ready


def test_real_interleaved_gui_log_keeps_endpoint_requirement():
 text=json.loads((R/'tests/fixtures/gazebo_gui_logs.json').read_text())['interleaved']
 assert log_readiness(text)['complete']
 assert not log_readiness(text.replace('/world/default/gui/info','/world/another/gui/info'))['complete']
 assert not log_readiness(text.replace('Requesting GUI from','irrelevant text'))['complete']

@pytest.mark.parametrize('case',['valid','wrong_partition','negative','malformed','crashed'])
def test_scene_gate_never_infers_visual_success(client,case,monkeypatch):
 from types import SimpleNamespace
 import runner_gui
 client.window_ready=True;client.env={'GZ_PARTITION':'fixture'}
 proof={'valid':case!='negative','partition':'wrong' if case=='wrong_partition' else 'fixture'}
 log=client.run/'logs/scene.log';log.write_text('not JSON' if case=='malformed' else json.dumps(proof))
 item=Item(2 if case=='crashed' else 0);item.logfile=SimpleNamespace(name=str(log))
 client.supervisor.spawn=lambda *a,**k:item
 monkeypatch.setattr(runner_gui.BaseSupervisor,'close',lambda *a,**k:{'ok':case!='crashed'})
 if case=='valid':
  client.await_ready(None);client.finish()
  state=json.loads((client.run/'runtime/gui-status.json').read_text())
  assert state['ready'] and state['server_scene_verified']
  assert state['gui_received_entities'] is None and state['visual_confirmed'] is None
 else:
  with pytest.raises(GUIFailure):client.await_ready(None)
  assert not client.ready and client.scene_proof is None

def test_scene_probe_timeout_is_bounded_and_keeps_process_for_cleanup(client,monkeypatch):
 from types import SimpleNamespace
 import runner_gui
 client.window_ready=True;client.env={'GZ_PARTITION':'fixture'}
 probe=Item();client.supervisor.spawn=lambda *a,**k:probe
 monkeypatch.setattr(runner_gui.DeadlineBudget,'after',lambda *a:SimpleNamespace(remaining=lambda:0))
 with pytest.raises(GUIFailure,match='GUI_SCENE_TIMEOUT'):client.await_ready(None)
 assert client.probe is probe and not client.ready
