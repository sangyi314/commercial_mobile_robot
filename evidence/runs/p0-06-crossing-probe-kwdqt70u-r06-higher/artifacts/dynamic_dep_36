"""One bounded real simulation trial. Parent owns every spawned process."""
import argparse,copy,json,os,select,signal,subprocess,sys,tarfile,time,uuid
from pathlib import Path
from datetime import datetime,timezone
import yaml
from runner_core import *
from runner_processes import ProcessSupervisor,ProjectLock
import validate_run
ROOT=Path(__file__).resolve().parents[1]
def safe_print(*args,**kwargs):
 try:print(*args,**kwargs)
 except BrokenPipeError:sys.stdout=open(os.devnull,'w')

class WorkerLink:
 def __init__(self,item,identity,log,supervisor,on_event=lambda *a:None):
  self.item=item;self.identity=identity;self.log=log;self.supervisor=supervisor;self.on_event=on_event;self.seq=0;self.lastseq=0;self.input=bytearray();self.output=bytearray();self.results={};self.last_heartbeat=time.monotonic();self.events=[];self.sidecars=[]
  os.set_blocking(item.process.stdout.fileno(),False);os.set_blocking(item.process.stdin.fileno(),False)
 def send(self,op,args,deadline,rid=None):
  self.seq+=1;rid=rid or str(self.seq);value={'protocol_version':1,'run_id':self.identity,'seq':self.seq,'request_id':rid,'type':'COMMAND','data':{'op':op,'args':args,'deadline':deadline}}
  self.log('WORKER_COMMAND',value);self.output.extend(encode(value));return rid
 def pump(self,monitor=True):
  p=self.item.process
  if self.output:
   try:n=os.write(p.stdin.fileno(),self.output);del self.output[:n]
   except BlockingIOError:pass
  ready,_,_=select.select([p.stdout],[],[],.02)
  if ready:
   data=os.read(p.stdout.fileno(),65536)
   if not data:raise RuntimeError('worker IPC EOF')
   self.input.extend(data)
   if len(self.input)>65536:raise ValueError('oversized worker protocol')
   while b'\n' in self.input:
    line,_,rest=self.input.partition(b'\n');self.input=bytearray(rest);e=strict_json(line.decode());self.lastseq=validate_envelope(e,self.identity,self.lastseq)
    e['parent_receive_monotonic_s']=time.monotonic();self.log('WORKER_EVENT',e);self.events.append(e)
    if e['type'] in ['HEARTBEAT','HELLO']:self.last_heartbeat=time.monotonic()
    if e['type']=='RESULT':self.results[e['request_id']]=e['data']
    self.on_event(e,time.monotonic())
  if time.monotonic()-self.last_heartbeat>3:raise TimeoutError('worker heartbeat timeout')
  for sidecar in self.sidecars:sidecar.pump(monitor=False)
  if monitor:self.supervisor.assert_critical()
 def rpc(self,op,args,budget):
  budget.require();rid=self.send(op,args,budget.end)
  while rid not in self.results:
   budget.require();self.pump()
  response=self.results.pop(rid)
  if not response['ok']:raise RuntimeError(op+': '+response['error'])
  return response.get('value')

def load_inputs(scenario,profile,method,identity):
 run_id(identity);p=yaml.safe_load(Path(profile).read_text());
 if 'synthetic_test' in p or 'test_namespace' in p:raise ValueError('synthetic profile cannot execute production runner')
 s=validate_run.load_strict(Path(scenario));validate_run.validate_structure(s,'scenario',scenario);validate_run.scenario_rules(s,Path(scenario))
 if p.get('profile_version')==2:
  from dynamic_contract import profile_inputs
  return profile_inputs(s,p,method,load_inputs,identity)
 if p.get('scope')!='simulation_only' or p.get('profile_version')!=1 or s['kind']!='static_smoke' or s['dynamic_events']:raise ValueError('only reviewed static simulation profile supported')
 if p.get('launch')!={'package':'nav2_bringup','localization':'localization_launch.py','navigation':'navigation_launch.py','autostart':False,'use_composition':False,'use_respawn':False,'rviz':False}:raise ValueError('unreviewed launch profile')
 m=p['methods'][method]
 if m['plugin']!=PLUGINS[method] or digest(m['config'])!=m['sha256']:raise ValueError('config identity')
 for v in p['files'].values():
  if not Path(v['path']).is_absolute() or digest(v['path'])!=v['sha256']:raise ValueError('resource identity '+v['path'])
 pair_spec=strict_json((Path(p['pair'])/'pair.json').read_text())
 if p['files']['robot']['sha256']!=pair_spec['inputs_sha256']['robot.sdf']:raise ValueError('expanded robot identity differs from P0-04')
 import xml.etree.ElementTree as ET
 robot=ET.parse(p['files']['robot']['path']).getroot()
 if robot.tag!='sdf' or any('$(' in (x.text or '') for x in robot.iter('topic')):raise ValueError('robot SDF has unresolved topic substitutions')
 if digest(p['gazebo_environment']['LD_PRELOAD'])!=p['preload_sha256']:raise ValueError('preload changed')
 if p['environment']['ROS_DOMAIN_ID']!='71' or p['environment']['ROS_AUTOMATIC_DISCOVERY_RANGE']!='SYSTEM_DEFAULT':raise ValueError('DDS identity')
 if not all(type(x) in (float,int) and 0<x<=360 for x in p['budgets'].values()):raise ValueError('invalid budgets')
 if sum(p['budgets'][k] for k in ['work','cleanup','finalize'])>450:raise ValueError('total waiting budget')
 if any(s['task'][k] is None for k in ['initial_pose','goal_pose','common_constraints_ref']):raise ValueError('missing task')
 if s['task']['goal_pose']!=validate_run.load_strict(Path(p['pair'])/'inputs/scenario-template.yaml')['task']['goal_pose']:raise ValueError('unreviewed task')
 for v in json.loads(Path(p['files']['dependencies']['path']).read_text())['files'].items():
  if digest(v[0])!=v[1]:raise ValueError('model dependency changed')
 for name,v in p['methods'].items():
  reference=Path(v['reference_run']);d=validate_run.validate_run_dir(reference)
  if d['status']!='VALID' or d['record_origin']!='real' or digest(reference/'manifest.json')!=v['manifest_sha256']:raise ValueError('baseline reference')
  result=json.loads((reference/'result.json').read_text())
  if result['status']['action']!='SUCCEEDED' or result['status']['cleanup']!='SUCCEEDED':raise ValueError('baseline not accepted')
 if not Path(p['recording']['qos']).is_file():raise ValueError('QoS missing')
 return p,s

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--scenario',required=True);ap.add_argument('--method',choices=['mppi','dwb'],required=True);ap.add_argument('--profile',required=True);ap.add_argument('--run-id',required=True);ap.add_argument('--check',action='store_true');ap.add_argument('--fault',choices=['withhold-initial-pose']);ap.add_argument('--ack-test-fault',action='store_true');args=ap.parse_args();entry=time.monotonic()
 try:
  if ROOT!=Path('/home/robot/commercial_mobile_robot'):raise ValueError('wrong project root')
  if bool(args.fault)!=args.ack_test_fault:raise ValueError('fault requires explicit acknowledgement and vice versa')
  p,scenario=load_inputs(args.scenario,args.profile,args.method,args.run_id)
  run=ROOT/'evidence/runs'/args.run_id
  if run.exists():raise FileExistsError('run directory already exists')
  plan={'run_id':args.run_id,'method':args.method,'scope':'simulation_only','run_directory':str(run),'processes':['worker','gazebo','robot_spawn','robot_state_publisher','bridge','recorder','localization','navigation'],'budgets':p['budgets'],'topics':p['recording']['topics'],'fault':args.fault,'plugin':p['methods'][args.method]['plugin'],'check_has_no_side_effects':True,'mode':p.get('mode','navigation_recording'),'sends_goal':p.get('mode')!='probe','raw_capture':'Gazebo sidecars; no truth ROS publication'}
  if p.get('profile_version')==2:
   plan['processes']+=['scene-driver','scene-capture','dynamic-inspection']
   plan['executables']=p['dynamic_profile']['executables']
   plan['resources']=p['files']
   plan['motion_timing']={k:p['dynamic_spec']['dynamic'][k] for k in ['trigger','delay_ns','period_ns','max_gap_ns','truth_gap_ns']}
   plan['snapshots']={'inputs':str(run/'artifacts'),'raw_gazebo':str(run/'runtime/gazebo-raw.jsonl'),'motion_commands':str(run/'runtime/motion-commands.jsonl')}
   plan['cleanup_order']=['quiesce/drain scene driver','close raw capture','navigation shutdown','recorder flush','localization/bridge/robot','gazebo stop','worker exit']
  if args.check:print(json.dumps(plan,ensure_ascii=False));return 0
 except Exception as exc:print(json.dumps({'preflight_rejected':str(exc)},ensure_ascii=False),file=sys.stderr);return 2
 try:lock=ProjectLock(ROOT/'evidence/env/p0-05-project.lock');lock.__enter__()
 except Exception as exc:print(str(exc),file=sys.stderr);return 2
 state=TrialState();manifest=None;link=None;supervisor=None;behavior=None;bagcheck=None;interrupted=False;manager_started=[];objects={};goal_deadline=None;eventfile=None;cleanup_evidence=[];dynamic=None;dynamic_report=None
 def log(kind,data):
  if eventfile and not eventfile.closed:eventfile.write(encode({'type':kind,'parent_monotonic_s':time.monotonic(),'run_id':args.run_id,'data':data}).decode());eventfile.flush()
  if kind=='PHASE':safe_print(kind,data,flush=True)
 def update(event):
  nonlocal state
  state,_=reduce_event(state,event,time.monotonic());log('STATE',{'event':event,'state':{'phase':state.phase,'startup':state.startup,'action':state.action,'harness':state.harness}})
 def phase(name):update({'type':'PHASE','phase':name});log('PHASE',name)
 def incoming(e,received):
  nonlocal state
  if e['type']=='ACTION_TERMINAL' and state.uuid:
   data=e['data']['data'];state,_=reduce_event(state,{'type':'TERMINAL','action':data['action'],'uuid':data['uuid'],'deadline':goal_deadline},received)
  if e['type']=='CLOCK_JUMP':raise RuntimeError('clock jumped backwards')
  if e['type']=='INPUT_STALE':raise RuntimeError('navigation inputs stale '+str(e['data']))
 def wait_process(item,budget):
  while item.process.poll() is None:
   budget.require()
   if link:link.pump()
   else:time.sleep(min(.02,budget.remaining()))
  if item.process.returncode!=0:raise RuntimeError(item.role+' exit '+str(item.process.returncode))
  supervisor.close(item,budget,initial_signal=False,grace=.1)
 def rpc(op,data,budget):return link.rpc(op,data,budget)
 try:
  run.mkdir()
  for d in ['artifacts','runtime','logs']:(run/d).mkdir()
  eventfile=(run/'logs/events.jsonl').open('x');supervisor=ProcessSupervisor(run,log);phase('SNAPSHOT')
  env=os.environ.copy();env.update(p['environment']);env.update({'GZ_PARTITION':'cmr_'+args.run_id,'PYTHONDONTWRITEBYTECODE':'1','PYTHONUNBUFFERED':'1','RCUTILS_LOGGING_USE_STDOUT':'0','RCUTILS_COLORIZED_OUTPUT':'0'});env.pop('ROS_LOCALHOST_ONLY',None);env.pop('LD_PRELOAD',None)
  # Read the logged-in desktop's actual display variables, never guess them.
  for proc in Path('/proc').iterdir():
   if not proc.name.isdigit():continue
   try:
    if proc.stat().st_uid==os.getuid() and (proc/'comm').read_text().strip()=='xfce4-session':
     desktop=dict(x.split('=',1) for x in (proc/'environ').read_bytes().decode().split('\0') if '=' in x)
     for k in ['DISPLAY','XAUTHORITY']:env[k]=desktop[k]
   except (PermissionError,FileNotFoundError,ProcessLookupError):pass
  for key,name in [('world','world.sdf'),('robot','robot.sdf'),('goal','goal.json'),('initialpose','initialpose.json'),('constraints','constraints.json')]:atomic_write_new(run/'artifacts'/name,Path(p['files'][key]['path']).read_bytes())
  atomic_write_new(run/'artifacts/config.yaml',Path(p['methods'][args.method]['config']).read_bytes());atomic_write_new(run/'artifacts/recording_qos.yaml',Path(p['recording']['qos']).read_bytes())
  for name in ['runner_core.py','runner_processes.py','runner_analysis.py','nav_goal.py','inspect_bag.py','run_trial.py']:atomic_write_new(run/'artifacts'/name,(ROOT/'tools'/name).read_bytes())
  if p.get('profile_version')==2:
   for key in ['actor','motion','geometry']:
    atomic_write_new(run/'artifacts'/('dynamic_'+key+('.sdf' if key=='actor' else '.json')),Path(p['files']['dynamic_'+key]['path']).read_bytes())
   for name in ['runner_dynamic.py','dynamic_contract.py','dynamic_geometry.py','inspect_dynamic_recording.py']:
    atomic_write_new(run/'artifacts'/name,(ROOT/'tools'/name).read_bytes())
   for role,ref in p['dynamic_profile']['executables'].items():
    atomic_write_new(run/'artifacts'/('cmr_'+role+'.binary'),Path(ref['path']).read_bytes())
  frozen=copy.deepcopy(p);frozen.update({'selected_method':args.method,'fault':args.fault,'ack_test_fault':args.ack_test_fault});write_json(run/'artifacts/profile.json',frozen)
  scene=copy.deepcopy(scenario);scene['scenario_revision']=args.run_id;scene['resources']['world']['path']=str(run/'artifacts/world.sdf');scene['task']['common_constraints_ref']['path']=str(run/'artifacts/constraints.json')
  if p.get('profile_version')==2:
   from dynamic_contract import freeze_inputs
   scene=freeze_inputs(run,scenario);scene['scenario_revision']=args.run_id
  write_json(run/'artifacts/scenario.yaml',scene)
  head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True,timeout=5).strip();status=subprocess.check_output(['git','status','--porcelain=v1'],cwd=ROOT,text=True,timeout=5)
  if status:
   names=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z'],cwd=ROOT,timeout=5).decode().split('\0')
   with tarfile.open(run/'artifacts/source.tgz','w:gz') as archive:
    for name in sorted(set(names)):
     if name and Path(name).parts[0] not in ['evidence','build','install','log','.git'] and (ROOT/name).is_file():archive.add(ROOT/name,arcname=name,recursive=False)
  artifacts=[]
  for i,f in enumerate(sorted((run/'artifacts').iterdir())):
   kind={'scenario.yaml':'scenario','config.yaml':'config','constraints.json':'common_constraints','source.tgz':'source_snapshot'}.get(f.name,'other');artifacts.append({'id':'source_snapshot' if f.name=='source.tgz' else 'input_'+str(i),'path':str(f.relative_to(run)),'sha256':digest(f),'kind':kind})
  manifest={'schema_version':1,'run_id':args.run_id,'record_origin':'real','scenario':{'scenario_id':scene['scenario_id'],'scenario_revision':scene['scenario_revision'],'spec_sha256':digest(run/'artifacts/scenario.yaml')},'source':{'git_commit':head,'dirty':bool(status),'snapshot_ref':'source_snapshot' if status else None},'baseline':{'method':args.method,'config_sha256':digest(run/'artifacts/config.yaml'),'expected_plugin':PLUGINS[args.method],'common_constraints_sha256':digest(run/'artifacts/constraints.json')},'environment':{'ros_distro':'jazzy','platform':os.uname().sysname+' '+os.uname().machine,'package_versions':p['packages'],'external_seed':None,'seed_scope':[],'controller_seed_controlled':False},'clock':{'clock_epoch':args.run_id+':sim0','started_at_utc':datetime.now(timezone.utc).isoformat().replace('+00:00','Z'),'sim_start_s':None},'artifacts':artifacts};write_json(run/'manifest.json',manifest)
  check=validate_run.validate_run_dir(run);write_json(run/'runtime/pre-start-validation.json',check)
  if check['status']!='INCOMPLETE':raise RuntimeError('manifest rejected '+str(check))
  write_json(run/'runtime/environment.json',{k:env[k] for k in [*p['environment'],'GZ_PARTITION','DISPLAY','XAUTHORITY'] if k in env});write_json(run/'runtime/ownership.json',{'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'parent_pid':os.getpid(),'token':supervisor.token})
  work=DeadlineBudget(min(entry+p['budgets']['work'],time.monotonic()+p['budgets']['work']));startup=DeadlineBudget(min(entry+p['budgets']['startup'],work.end))
  display=supervisor.spawn('display-check',['xdpyinfo'],env,ROOT,critical=False);wait_process(display,startup.child(5))
  worker=supervisor.spawn('worker',[sys.executable,'-B',str(run/'artifacts/nav_goal.py'),'--run-dir',str(run)],env,ROOT,ipc=True);objects['worker']=worker;link=WorkerLink(worker,args.run_id,log,supervisor,incoming)
  rpc('graph',{},startup.child(5));phase('START_SIM')
  genv=env.copy();genv.update(p['gazebo_environment']);objects['gazebo']=supervisor.spawn('gazebo',['gz','sim','-s','-r','-v','3','--render-engine-server','ogre',str(run/'artifacts/world.sdf')],genv,ROOT)
  sp=p['spawn'];spawn=supervisor.spawn('robot-spawn',['ros2','run','ros_gz_sim','create','-world','default','-file',str(run/'artifacts/robot.sdf'),'-name','turtlebot3_waffle','-x',str(sp['x']),'-y',str(sp['y']),'-z',str(sp['z'])],env,ROOT,critical=False);wait_process(spawn,startup.child(30))
  sim=Path('/opt/ros/jazzy/share/nav2_minimal_tb3_sim')
  objects['robot']=supervisor.spawn('robot',['/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher','--ros-args','-p','use_sim_time:=true','-p','robot_description:='+(sim/'urdf/turtlebot3_waffle.urdf').read_text()],env,ROOT)
  objects['bridge']=supervisor.spawn('bridge',['/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge','--ros-args','-p','use_sim_time:=true','-p','expand_gz_topic_names:=true','-p','config_file:='+str(sim/'configs/turtlebot3_waffle_bridge.yaml')],env,ROOT)
  if p.get('profile_version')==2:
   from runner_dynamic import DynamicSession
   dynamic=DynamicSession(supervisor,run,p,env,log,link,WorkerLink)
  phase('SENSOR_READY');rpc('sensors',{},startup.child(35));phase('START_RECORDER')
  recorder_name='cmr_recorder_'+args.run_id.replace('-','_')
  objects['recorder']=supervisor.spawn('recorder',['ros2','bag','record','--storage','mcap','--output',str(run/'bag'),'--use-sim-time','--node-name',recorder_name,'--qos-profile-overrides-path',str(run/'artifacts/recording_qos.yaml'),'--include-hidden-topics','--topics',*p['recording']['topics']],env,ROOT)
  rpc('recorder',{'name':recorder_name,'topics':['/clock','/tf','/tf_static','/scan','/odom','/initialpose','/cmr_experiment/events']},startup.child(15));rpc('mark',{'name':'RECORDING_START'},startup.child(3))
  common=['params_file:='+str(run/'artifacts/config.yaml'),'use_sim_time:=true','autostart:=false','use_composition:=False','use_respawn:=False']
  phase('LOCALIZE');objects['localization']=supervisor.spawn('localization',['ros2','launch','--noninteractive','nav2_bringup','localization_launch.py',*common,'map:='+p['files']['map_yaml']['path']],env,ROOT);manager_started.append('localization')
  rpc('manager',{'phase':'localization','command':'STARTUP'},startup.child(55))
  if args.fault:rpc('withhold',{},startup.child(p['budgets']['fault_initialpose']))
  rpc('localize',{},startup.child(30))
  objects['navigation']=supervisor.spawn('navigation',['ros2','launch','--noninteractive','nav2_bringup','navigation_launch.py',*common],env,ROOT);manager_started.append('navigation')
  rpc('manager',{'phase':'navigation','command':'STARTUP'},startup.child(60));phase('NAV_READY');rpc('nav_ready',{},startup.child(30));rpc('recorder',{'name':recorder_name,'topics':p['recording']['topics']},startup.child(15));update({'type':'READY'})
  if dynamic:dynamic.prepare(startup.child(15))
  if p.get('mode')=='probe':
   dynamic.probe(work)
  else:
   phase('SEND_ONCE');uid=uuid.uuid4().hex;write_json(run/'runtime/dispatch-intent.json',{'uuid':uid,'monotonic_s':time.monotonic()});update({'type':'DISPATCH','uuid':uid});goalbudget=work.child(p['budgets']['goal']);goal_deadline=goalbudget.end

   try:accepted=rpc('send',{'uuid':uid},goalbudget.child(p['budgets']['accept']))
   except TimeoutError:
    update({'type':'FAIL','code':'ACCEPTANCE_TIMEOUT','detail':'No acceptance before parent deadline','harness':'NAV_TIMEOUT'});update({'type':'TIMEOUT'});accepted={'acceptance':'UNKNOWN'}
   if dynamic and accepted['acceptance']=='ACCEPTED':
    response=[e['data'] for e in link.events if e['type']=='ACTION_RESPONSE'][-1]
    dynamic.arm(work,response)
   phase('WAIT_RESULT')
   if accepted['acceptance']=='UNKNOWN':
    update({'type':'FAIL','code':'ACCEPTANCE_TIMEOUT','detail':'response not received before acceptance deadline','harness':'NAV_TIMEOUT'});update({'type':'TIMEOUT'})
   while state.action=='UNKNOWN' and not state.timeout_decided and goalbudget.remaining()>0:link.pump()
   if state.action=='UNKNOWN':
    if not state.timeout_decided:update({'type':'TIMEOUT'})
    cancel=work.child(p['budgets']['cancel']);rpc('cancel',{},cancel)
    while state.action=='UNKNOWN' and cancel.remaining()>0:link.pump()
   if dynamic and state.action=='SUCCEEDED':dynamic.wait_complete(work)
   phase('OBSERVE_STOP')
   if state.action=='SUCCEEDED':
    behavior=rpc('stop',{},work.child(p['budgets']['stop']))
    if not all(behavior['checks'].values()):raise RuntimeError('behavioral gate failed '+str(behavior['checks']))
   elif state.ledger.first is None:update({'type':'FAIL','code':'ACTION_'+state.action,'detail':'Navigation did not succeed','harness':'COMPLETED'})
 except KeyboardInterrupt:
  interrupted=True;update({'type':'INTERRUPT'})
 except BaseException as exc:
  code='CONTROLLED_INITIALPOSE_TIMEOUT' if 'CONTROLLED_INITIALPOSE_TIMEOUT' in str(exc) else getattr(exc,'code',type(exc).__name__)
  update({'type':'FAIL','code':code,'detail':str(exc)})
 finally:
  if supervisor:
   phase('SHUTDOWN');cleanup=DeadlineBudget.after(p['budgets']['cleanup']);cleanup_ok=True
   def attempt(label,fn):
    nonlocal cleanup_ok
    try:
     value=fn();cleanup_evidence.append({'operation':label,'ok':True,'value':value});return value
    except BaseException as exc:
     cleanup_ok=False;state.ledger.add('CLEANUP_'+label,state.phase,str(exc),time.monotonic());cleanup_evidence.append({'operation':label,'ok':False,'error':str(exc)});return None
   if state.sends and (state.action=='UNKNOWN' or behavior is None) and link:
    attempt('cancel',lambda:rpc('cancel',{},cleanup.child(5)))
    emergency=objects.get('gazebo')
    if emergency:attempt('emergency_physics',lambda:supervisor.close(emergency,cleanup,grace=3));cleanup_ok=False
   if dynamic:attempt('dynamic_close',lambda:dynamic.close(cleanup))
   if 'navigation' in manager_started and link:attempt('navigation_shutdown',lambda:rpc('manager',{'phase':'navigation','command':'SHUTDOWN'},cleanup.child(12)))
   if 'navigation' in objects:attempt('navigation_exit',lambda:supervisor.close(objects['navigation'],cleanup,grace=5))
   if 'recorder' in objects:
    if link:attempt('recording_end_marker',lambda:rpc('mark',{'name':'RECORDING_END'},cleanup.child(2)))
    attempt('recorder_exit',lambda:supervisor.close(objects['recorder'],cleanup,grace=8))
   if 'localization' in manager_started and link:attempt('localization_shutdown',lambda:rpc('manager',{'phase':'localization','command':'SHUTDOWN'},cleanup.child(10)))
   for role in ['localization','bridge','robot']:
    if role in objects:attempt(role+'_exit',lambda role=role:supervisor.close(objects[role],cleanup,grace=3))
   if 'gazebo' in objects and not objects['gazebo'].closed:
    objects['gazebo'].critical=False
    stop=supervisor.spawn('gazebo-stop',['gz','service','-s','/server_control','--reqtype','gz.msgs.ServerControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','stop: true'],env,ROOT,critical=False)
    attempt('gazebo_stop_service',lambda:wait_process(stop,cleanup.child(5)))
    attempt('gazebo_exit',lambda:supervisor.close(objects['gazebo'],cleanup,initial_signal=False,grace=8))
   if link and 'worker' in objects:
    objects['worker'].critical=False;attempt('worker_exit_request',lambda:rpc('exit',{},cleanup.child(3)))
    attempt('worker_exit',lambda:supervisor.close(objects['worker'],cleanup,initial_signal=False,grace=3))
   for item in reversed(supervisor.owned):
    if not item.closed:attempt(item.role+'_fallback',lambda item=item:supervisor.close(item,cleanup,grace=.2))
   cleanup_ok=cleanup_ok and all(x.result and x.result['ok'] for x in supervisor.owned)
   update({'type':'CLEANUP','ok':cleanup_ok,'detail':'See runtime/cleanup.json'})
   write_json(run/'runtime/cleanup.json',{'ok':cleanup_ok,'operations':cleanup_evidence,'processes':[x.result for x in supervisor.owned],'remaining':[m for x in supervisor.owned for m in supervisor.members(x)]})
   final=DeadlineBudget.after(p['budgets']['finalize']);phase('INSPECT_BAG')
   if (run/'bag/metadata.yaml').exists():
    inspect_item=supervisor.spawn('bag-inspection',[sys.executable,'-B',str(run/'artifacts/inspect_bag.py'),'--run-dir',str(run),'--output',str(run/'runtime/bag-check.json')],env,ROOT,critical=False)
    # Worker has exited; inspector is a bounded, expected one-shot process.
    try:
     inspect_item.process.wait(timeout=final.require());supervisor.close(inspect_item,final,initial_signal=False,grace=.1);bagcheck=json.loads((run/'runtime/bag-check.json').read_text())
     if not bagcheck['valid']:raise RuntimeError(str(bagcheck['errors']))
    except BaseException as exc:update({'type':'FAIL','code':'BAG_INVALID','detail':str(exc),'harness':'RECORDER_FAILED'});supervisor.close(inspect_item,final,grace=.1)
   elif 'recorder' in objects:update({'type':'FAIL','code':'BAG_MISSING','detail':'no closed metadata','harness':'RECORDER_FAILED'})
   if dynamic:
    inspector=supervisor.spawn('dynamic-inspection',[sys.executable,'-B',str(ROOT/'tools/inspect_dynamic_recording.py'),'--run-dir',str(run),'--output',str(run/'runtime/dynamic-report.json')],env,ROOT,critical=False)
    try:
     inspector.process.wait(timeout=final.require());supervisor.close(inspector,final,initial_signal=False,grace=.1);dynamic_report=json.loads((run/'runtime/dynamic-report.json').read_text())
     if not (dynamic_report['dynamic_recording_valid'] or dynamic_report['probe_pass']):update({'type':'FAIL','code':'DYNAMIC_RECORDING_INVALID','detail':str(dynamic_report['errors'])})
    except BaseException as exc:
     update({'type':'FAIL','code':'DYNAMIC_INSPECTION_FAILED','detail':str(exc)});supervisor.close(inspector,final,grace=.1)
   phase('FINALIZE');write_json(run/'runtime/failures.json',{'first':state.ledger.first,'later':state.ledger.later});write_json(run/'runtime/timing.json',{'entry_monotonic_s':entry,'end_monotonic_s':time.monotonic(),'elapsed_monotonic_s':time.monotonic()-entry,'scope':'Whole supervisor session; not controller performance','clock_epoch':args.run_id+':sim0'})
   log('FINAL_STATE',{'startup':state.startup,'action':state.action,'harness':state.harness,'cleanup':state.cleanup});eventfile.close()
   if manifest:
    artifacts=[]
    for f in sorted(run.rglob('*')):
     if not f.is_file() or not f.stat().st_size or f.name in ['manifest.json','result.json']:continue
     kind='bag_metadata' if f==run/'bag/metadata.yaml' else ('runtime_parameters' if f==run/'runtime/controller_server.yaml' else 'other')
     artifacts.append({'id':'runtime_parameters' if kind=='runtime_parameters' else 'evidence_'+str(len(artifacts)),'path':str(f.relative_to(run)),'sha256':digest(f),'kind':kind})
    plugin=json.loads((run/'runtime/nav-readiness.json').read_text())['plugin'] if (run/'runtime/nav-readiness.json').exists() else None
    metrics={'position_error':behavior['arrival']['position_m'],'yaw_error':behavior['arrival']['yaw_rad'],'stop':behavior['checks']['stop']} if behavior else {}
    try:
     candidate=finalize_record(manifest,digest(run/'manifest.json'),state,time.monotonic()-entry,artifacts,plugin,metrics);validate_run.validate_structure(candidate,'result',run/'result.json');write_json(run/'result.json',candidate);diagnostic=validate_run.validate_run_dir(run);write_json(run/'runtime/validation.json',diagnostic)
    except BaseException as exc:diagnostic={'status':'INVALID','error':str(exc)};write_json(run/'runtime/finalization-error.json',diagnostic)
   else:diagnostic={'status':'INCOMPLETE','error':'manifest snapshot failed before publication'}
   passed=state.startup=='READY' and state.action=='SUCCEEDED' and state.harness=='COMPLETED' and state.cleanup=='SUCCEEDED' and behavior and all(behavior['checks'].values()) and bagcheck and bagcheck['valid'] and diagnostic['status']=='VALID'
   if dynamic:passed=passed and dynamic_report and dynamic_report.get('dynamic_recording_valid')
   probe_pass=bool(p.get('mode')=='probe' and state.startup=='READY' and state.action=='NOT_SENT' and state.harness=='COMPLETED' and state.cleanup=='SUCCEEDED' and bagcheck and bagcheck['valid'] and dynamic_report and dynamic_report.get('probe_pass') and diagnostic['status']=='VALID')
   exitcode=130 if interrupted else (0 if passed or probe_pass else 3)
   write_json(run/'runtime/runner-exit.json',{'exit':exitcode,'normal_pass':bool(passed),'purpose':p.get('mode','navigation_recording'),'probe_pass':probe_pass,'dynamic_recording_valid':dynamic_report.get('dynamic_recording_valid') if dynamic_report else None,'record_contract':diagnostic['status'],'bag_valid':bool(bagcheck and bagcheck['valid'])});safe_print(json.dumps({'run':str(run),'exit':exitcode,'status':{'startup':state.startup,'action':state.action,'harness':state.harness,'cleanup':state.cleanup}},ensure_ascii=False),flush=True)
  else:exitcode=3
  lock.__exit__(None,None,None)
 return exitcode
if __name__=='__main__':sys.exit(main())
