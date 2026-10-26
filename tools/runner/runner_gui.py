"""Optional observation only. All processes use the original runner supervisor."""
from dataclasses import dataclass,asdict
from pathlib import Path
import json,math,os,re,select,sys,termios,time
from tools.runner.runner_core import DeadlineBudget, write_json
from tools.runner.runner_processes import ProcessSupervisor as BaseSupervisor
import tools.runner.runner_processes as runner_processes

class GUIFailure(RuntimeError):
 def __init__(self,code,detail):self.code=code;super().__init__(code+': '+str(detail))

@dataclass(frozen=True)
class GUIOptions:
 gazebo_gui:bool=False
 wait_for_operator:bool=False
 operator_timeout_s:float=60.
 gui_startup_timeout_s:float=20.
 def validate(self):
  if type(self.gazebo_gui) is not bool or type(self.wait_for_operator) is not bool:raise ValueError('GUI options must be booleans')
  if self.wait_for_operator and not self.gazebo_gui:raise ValueError('wait_for_operator requires gazebo_gui')
  for name,limit in [('operator_timeout_s',120),('gui_startup_timeout_s',60)]:
   v=getattr(self,name)
   if type(v) not in (int,float) or not math.isfinite(v) or not 0<v<=limit:raise ValueError(name+' must be finite and within (0, '+str(limit)+']')
  return self
 def plan(self):
  return {**asdict(self),'gui_argv':gui_argv() if self.gazebo_gui else None,'process_roles':['gazebo-gui','gui-window-probes','gui-scene-probe'] if self.gazebo_gui else [],'server_count':1,'partition':'inherit this run GZ_PARTITION','renderer':'ogre / LIBGL_ALWAYS_SOFTWARE=1','wait_point':'after nav_ready, before dispatch intent and goal deadline','wait_clock':'independent wall-clock deadline; original total work budget still applies','pauses_gazebo':False}

def gui_argv():return ['stdbuf','-oL','-eL','gz','sim','-g','-v','4','--render-engine-gui','ogre']

def log_readiness(text):
 # Initialization markers only: requests are NOT evidence of scene receipt.
 text=re.sub(r'\x1b\[[0-9;]*m','',text)
 markers=['Requesting GUI from [/world/default/gui/info]','Received resource paths.','Requesting initial state from [/world/default/state]','Loading plugin [gz-rendering-ogre]','Camera pose topic advertised on [/gui/camera/pose]']
 # Gazebo writes message fragments from multiple threads; another log line may
 # appear between a request prefix and its endpoint. Keep both required and
 # bound the association, rather than requiring an uninterrupted whole line.
 found={}
 for marker in markers:
  prefix,sep,suffix=marker.partition('[')
  found[marker]=bool(re.search(re.escape(prefix+sep)+r'[\s\S]{0,512}?'+re.escape(suffix),text)) if sep else marker in text
 return {'world':'default','markers':found,'complete':all(found.values())}

class GUIClient:
 def __init__(self,supervisor,options,emit):
  self.supervisor=supervisor;self.options=options;self.emit=emit;self.run=supervisor.run;self.item=None;self.probe=None;self.ready=False;self.window_ready=False;self.scene_proof=None;self.monitor=True;self.finished=False;self.failure=None;self.events=[];self.window={};self.counter=0;self.next_probe=0.;self.operator='not_requested';self.env=None
  write_json(self.run/'artifacts/gui-options.json',asdict(options))
 def event(self,state,**data):
  value={'state':state,'wall_monotonic_s':time.monotonic(),**data};self.events.append(value);self.emit('GUI_'+state,value)
  with (self.run/'runtime/gui-events.jsonl').open('a') as f:
   import json
   f.write(json.dumps(value,allow_nan=False)+'\n')
 def fail(self,code,detail):
  self.failure={'code':code,'detail':str(detail)};self.monitor=False;self.event(code,detail=str(detail))
  raise GUIFailure(code,detail)
 def start(self,env,cwd):
  self.env=env.copy();self.env['QT_QPA_PLATFORM']='xcb'
  if not self.env.get('GZ_PARTITION') or not self.env.get('DISPLAY') or self.env.get('LIBGL_ALWAYS_SOFTWARE')!='1':
   try:self.fail('GUI_ENVIRONMENT_INVALID','requires current partition, actual DISPLAY and software rendering')
   finally:self.finish()
  self.started=time.monotonic();self.start_deadline=self.started+self.options.gui_startup_timeout_s
  self.event('STARTING',argv=gui_argv(),environment={k:self.env[k] for k in ['GZ_PARTITION','DISPLAY','XAUTHORITY','LIBGL_ALWAYS_SOFTWARE','LD_PRELOAD','QT_QPA_PLATFORM'] if k in self.env})
  try:self.item=self.supervisor.spawn('gazebo-gui',gui_argv(),self.env,cwd,critical=False)
  except Exception as exc:
   try:self.fail('GUI_START_FAILED',exc)
   finally:self.finish()
 def probe_start(self,step,argv):
  role='gui-window-'+str(self.counter)+'-'+step;self.counter+=1;self.probe_step=step;self.probe=self.supervisor.spawn(role,argv,self.env,self.run,critical=False);self.probe_deadline=min(time.monotonic()+2,self.start_deadline)
 def tick(self):
  if not self.monitor or self.item is None:return
  code=self.item.process.poll()
  if code is not None:self.fail('GUI_EARLY_EXIT',{'exit':code,'was_ready':self.ready})
  if self.window_ready:return
  if time.monotonic()>=self.start_deadline:self.fail('GUI_READY_TIMEOUT',{'readiness':getattr(self,'readiness',None),'window':self.window,'reason':'GUI initialization markers or owned visible window unavailable'})
  text=(self.run/'logs/gazebo-gui.log').read_text(errors='replace');self.readiness=log_readiness(text)
  if not self.readiness['complete']:return
  if self.probe is None:
   if time.monotonic()>=self.next_probe:self.probe_start('tree',['xwininfo','-root','-tree'])
   return
  if self.probe.process.poll() is None:
   if time.monotonic()>=self.probe_deadline:self.fail('GUI_WINDOW_CHECK_TIMEOUT',self.probe.role)
   return
  result=BaseSupervisor.close(self.supervisor,self.probe,DeadlineBudget.after(.3),initial_signal=False,grace=.1)
  output=Path(self.probe.logfile.name).read_text(errors='replace');self.probe=None
  if not result['ok']:self.fail('GUI_WINDOW_CHECK_FAILED',result)
  if self.probe_step=='tree':
   found=re.search(r'(0x[0-9a-f]+) "Gazebo Sim":.*?\s(\d+)x(\d+)\+',output)
   if not found or min(int(found[2]),int(found[3]))<64:self.next_probe=time.monotonic()+.2;return
   self.window={'id':found[1],'width':int(found[2]),'height':int(found[3])};self.probe_start('pid',['xprop','-id',self.window['id'],'_NET_WM_PID'])
  elif self.probe_step=='pid':
   found=re.search(r'_NET_WM_PID\(CARDINAL\) = (\d+)',output);pids={v['pid'] for v in self.supervisor.members(self.item)}
   if not found or int(found[1]) not in pids:self.fail('GUI_WINDOW_OWNER_MISMATCH',output)
   self.window['pid']=int(found[1]);self.probe_start('mapped',['xwininfo','-id',self.window['id'],'-stats'])
  else:
   if 'Map State: IsViewable' not in output:self.next_probe=time.monotonic()+.2;return
   self.window['map_state']='IsViewable';self.window_ready=True;self.event('WINDOW_READY',window=self.window,readiness=self.readiness,partition=self.env['GZ_PARTITION'])
 def await_ready(self,link):
  while not self.window_ready:link.pump()
  budget=DeadlineBudget.after(8)
  self.probe=self.supervisor.spawn('gui-scene-probe',[sys.executable,'-B',str(self.run/'artifacts/tools/runner/gui_scene_probe.py'),str(self.run)],self.env,self.run,critical=False)
  while self.probe.process.poll() is None:
   if budget.remaining()<=0:self.fail('GUI_SCENE_TIMEOUT','No complete scene evidence before deadline')
   link.pump()
  result=BaseSupervisor.close(self.supervisor,self.probe,budget,initial_signal=False,grace=.2)
  text=Path(self.probe.logfile.name).read_text(errors='replace');self.probe=None
  try:proof=json.loads(text)
  except ValueError:self.fail('GUI_SCENE_INVALID',text[-1000:])
  if not result['ok'] or not proof.get('valid') or proof.get('partition')!=self.env['GZ_PARTITION']:self.fail('GUI_SCENE_UNAVAILABLE',proof)
  self.scene_proof=proof;write_json(self.run/'runtime/gui-scene-proof.json',proof)
  self.ready=True;self.event('OBSERVATION_AVAILABLE',server_scene_verified=True,gui_received_entities=None,visual_confirmed=None)

 def fresh(self,link,budget):
  try:return link.rpc('sensors',{},budget.child(.8))
  except GUIFailure:raise  # A GUI exit during RPC must keep its original cause.
  except RuntimeError as exc:
   if str(exc).startswith('sensors:'):self.fail('OPERATOR_INPUT_STALE',exc)
   raise  # Worker IPC/heartbeat errors are not sensor freshness evidence.
 def await_operator(self,link,budget=None,fd=None,clock=time.monotonic):
  if not self.options.wait_for_operator:return
  if not self.ready:self.fail('GUI_NOT_READY','operator wait requires positive GUI readiness')
  fd=sys.stdin.fileno() if fd is None else fd
  if not os.isatty(fd):self.fail('OPERATOR_TTY_REQUIRED','use a local terminal or ssh -t')
  termios.tcflush(fd,termios.TCIFLUSH)
  wait=DeadlineBudget.after(self.options.operator_timeout_s,clock,parent=budget);self.operator='waiting';self.event('WAITING_OPERATOR',deadline_wall_s=wait.end)
  print('Server scene verified; GUI window open; navigation ready. Visually check robot, ground and obstacles, then press Enter to send ONE goal (timeout '+str(self.options.operator_timeout_s)+' wall seconds).',flush=True)
  pending=b'';next_fresh=clock();old_blocking=os.get_blocking(fd);os.set_blocking(fd,False)
  try:
   while True:
    if wait.remaining()<=0:self.operator='timed_out';self.fail('OPERATOR_WAIT_TIMEOUT','no Enter before independent wall-clock deadline')
    link.pump()
    if clock()>=next_fresh:
     self.fresh(link,wait)
     next_fresh=clock()+1
    if not select.select([fd],[],[],0)[0]:continue
    try:data=os.read(fd,1024)
    except BlockingIOError:continue
    if not data:self.operator='input_closed';self.fail('OPERATOR_INPUT_CLOSED','stdin EOF before Enter')
    pending+=data
    if len(pending)>4096:self.fail('OPERATOR_INPUT_INVALID','input line exceeds 4096 bytes')
    if b'\n' not in pending:continue
    line,_,pending=pending.partition(b'\n')
    if line.rstrip(b'\r'):
     self.event('OPERATOR_INPUT_IGNORED',reason='press Enter on an empty line');continue
    if wait.remaining()<=0:self.operator='timed_out';self.fail('OPERATOR_WAIT_TIMEOUT','Enter received at or after deadline')
    self.fresh(link,wait);self.operator='confirmed';self.event('OPERATOR_CONFIRMED');return
  finally:os.set_blocking(fd,old_blocking)
 def close(self,budget):
  if self.finished:return
  self.monitor=False
  if self.probe and not self.probe.closed:BaseSupervisor.close(self.supervisor,self.probe,budget,grace=.2)
  if self.item and not self.item.closed:BaseSupervisor.close(self.supervisor,self.item,budget,grace=3)
  self.event('CLOSED',process_result=self.item.result if self.item else None);self.finish()
 def finish(self):
  if self.finished:return
  self.finished=True;write_json(self.run/'runtime/gui-status.json',{'schema_version':1,'options':asdict(self.options),'ready':self.ready,'window_ready':self.window_ready,'server_scene_verified':bool(self.scene_proof),'gui_received_entities':None,'visual_confirmed':None,'window':self.window,'operator':self.operator,'failure':self.failure,'events':self.events,'gui_process':self.item.result if self.item else None,'scope':'ready means observation available: independent server scene and owned window verified; GUI entity consumption and visual correctness not automatically certified'})
