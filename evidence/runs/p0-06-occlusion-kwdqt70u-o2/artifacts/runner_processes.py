"""Linux process ownership and finite, idempotent cleanup. No ROS imports."""
from dataclasses import dataclass
from pathlib import Path
import fcntl,json,os,signal,subprocess,time,uuid
from runner_core import DeadlineBudget

def identity(pid):
 try:
  p=Path('/proc')/str(pid);text=(p/'stat').read_text();fields=text[text.rfind(')')+2:].split()
  return {'pid':int(pid),'state':fields[0],'ppid':int(fields[1]),'pgid':int(fields[2]),'sid':int(fields[3]),'start_ticks':int(fields[19])}
 except (FileNotFoundError,ProcessLookupError):return None
class ProjectLock:
 def __init__(self,path):self.path=Path(path);self.file=None
 def __enter__(self):
  self.file=self.path.open('a+')
  try:fcntl.flock(self.file,fcntl.LOCK_EX|fcntl.LOCK_NB)
  except BaseException:self.file.close();raise RuntimeError('another project runner holds lock')
  return self
 def __exit__(self,*args):self.file.close()
@dataclass
class OwnedProcess:
 role:str
 process:object
 birth:dict
 logfile:object
 critical:bool=True
 closed:bool=False
 result:dict|None=None
class ProcessSupervisor:
 def __init__(self,run,emit=lambda *x:None):
  self.run=Path(run);self.emit=emit;self.token=uuid.uuid4().hex;self.owned=[]
 def spawn(self,role,argv,env,cwd,critical=True,ipc=False):
  if not isinstance(argv,list) or not all(isinstance(a,str) and '\0' not in a for a in argv):raise ValueError('argv must be strings')
  out=(self.run/'logs'/f'{role}.log').open('xb');childenv=env.copy();childenv['CMR_OWNERSHIP_TOKEN']=self.token+':'+role
  proc=subprocess.Popen(argv,shell=False,cwd=cwd,env=childenv,start_new_session=True,stdin=subprocess.PIPE if ipc else subprocess.DEVNULL,stdout=subprocess.PIPE if ipc else out,stderr=out)
  birth=identity(proc.pid);birth['ownership_token']=childenv['CMR_OWNERSHIP_TOKEN'];item=OwnedProcess(role,proc,birth,out,critical);self.owned.append(item)
  self.emit('PROCESS_STARTED',{'role':role,'argv':argv,'cwd':str(cwd),'identity':birth,'monotonic_s':time.monotonic(),'log':str(out.name),'environment':{k:childenv[k] for k in ['ROS_DOMAIN_ID','GZ_PARTITION','RMW_IMPLEMENTATION','ROS_AUTOMATIC_DISCOVERY_RANGE','LD_PRELOAD','CMR_OWNERSHIP_TOKEN'] if k in childenv}})
  return item
 def members(self,item):
  result=[]
  for p in Path('/proc').iterdir():
   if not p.name.isdigit():continue
   try:
    if p.stat().st_uid!=os.getuid():continue
    info=identity(p.name)
    if not info or info['state']=='Z':continue
    env=(p/'environ').read_bytes().split(b'\0')
    if ('CMR_OWNERSHIP_TOKEN='+item.birth['ownership_token']).encode() in env:info['ownership_token']=item.birth['ownership_token'];result.append(info)
   except (PermissionError,FileNotFoundError,ProcessLookupError):continue
  return result
 def signal_identity(self,info,sig):
  current=identity(info['pid'])
  if not current:return False
  if any(current[k]!=info[k] for k in ['start_ticks','sid','pgid']):raise RuntimeError('PID identity changed; refusing signal')
  marker=('CMR_OWNERSHIP_TOKEN='+info['ownership_token']).encode()
  if marker not in (Path('/proc')/str(info['pid'])/'environ').read_bytes().split(b'\0'):raise RuntimeError('ownership marker missing')
  os.kill(info['pid'],sig);return True
 def close(self,item,budget,initial_signal=True,grace=5):
  if item.closed:return item.result
  record={'role':item.role,'identity':item.birth,'forced':False,'signals':[],'errors':[]}
  try:
   if item.process.poll() is None and initial_signal:
    if self.signal_identity(item.birth,signal.SIGINT):record['signals'].append({'pid':item.process.pid,'signal':'SIGINT_LEADER'})
   deadline=budget.child(min(grace,budget.remaining()))
   while deadline.remaining()>0:
    item.process.poll()
    if item.process.returncode is not None and not self.members(item):break
    time.sleep(min(.02,deadline.remaining()))
   for sig,seconds in [(signal.SIGTERM,1.5),(signal.SIGKILL,1.5)]:
    members=self.members(item)
    if not members:break
    record['forced']=True
    for info in members:
     if budget.remaining()<=0:break
     self.signal_identity(info,sig);record['signals'].append({'pid':info['pid'],'signal':sig.name})
    end=budget.child(min(seconds,budget.remaining()))
    while end.remaining()>0 and self.members(item):time.sleep(min(.02,end.remaining()))
   if item.process.poll() is None and budget.remaining()>0:
    try:item.process.wait(timeout=budget.remaining())
    except subprocess.TimeoutExpired:record['errors'].append('leader wait timeout')
  except (OSError,RuntimeError) as exc:record['errors'].append(str(exc))
  record['remaining']=self.members(item);record['exit']=item.process.poll()
  record['ok']=not record['forced'] and not record['remaining'] and not record['errors'] and record['exit'] in (0,-signal.SIGINT)
  item.logfile.flush();item.logfile.close();item.closed=True;item.result=record;self.emit('PROCESS_CLOSED',record);return record
 def assert_critical(self,except_roles=()):
  for item in self.owned:
   if item.critical and not item.closed and item.role not in except_roles and item.process.poll() is not None:raise RuntimeError('critical process exited: '+item.role+' '+str(item.process.returncode))
