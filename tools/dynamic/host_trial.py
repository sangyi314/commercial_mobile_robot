"""Bounded host-only load observation around ONE existing Ubuntu runner call."""
import argparse,ctypes,json,os,pathlib,platform,shlex,subprocess,sys,time,threading

def cpu_ticks():
 lib=ctypes.CDLL('/usr/lib/libSystem.B.dylib');lib.mach_host_self.restype=ctypes.c_uint
 values=(ctypes.c_uint*4)();count=ctypes.c_uint(4)
 rc=lib.host_statistics(lib.mach_host_self(),3,ctypes.cast(values,ctypes.POINTER(ctypes.c_int)),ctypes.byref(count))
 if rc:raise RuntimeError('host_statistics '+str(rc))
 return list(values)

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--scenario',required=True);ap.add_argument('--profile',required=True);ap.add_argument('--run-id',required=True);ap.add_argument('--load',choices=['low','higher'],required=True);ap.add_argument('--output',required=True);a=ap.parse_args()
 out=pathlib.Path(a.output);out.mkdir(parents=True,exist_ok=False);children=[];stop=threading.Event();rows=[];fail=[]
 def record(row):
  row['host_monotonic_s']=time.monotonic();row['host_wall_ns']=time.time_ns();rows.append(row)
  with (out/'host-load.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
 def ssh(code,timeout=5):return subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=3','cmr-ubuntu',code],capture_output=True,text=True,timeout=timeout)
 meta={'platform':platform.platform(),'load_request':a.load,'run_id':a.run_id,'logical_cpus':int(subprocess.check_output(['sysctl','-n','hw.logicalcpu'],text=True)),'load_workers':2 if a.load=='higher' else 0,'worker_nice':19,'worker_max_wall_s':120,'scope':'actual Mac host, not VM-only pressure; no VM allocation changes','personal_confirmation':None}
 (out/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
 def sample():
  previous=cpu_ticks();count=0
  while not stop.wait(1):
   try:
    current=cpu_ticks();delta=[(b-a)&0xffffffff for a,b in zip(previous,current)];previous=current;total=sum(delta)
    record({'kind':'mac_cpu','ticks':current,'delta_ticks':delta,'busy_fraction':1-delta[2]/total if total else None,'owned_load_pids':[p.pid for p,_ in children if p.poll() is None]})
    if count%2==0:
     sent=time.monotonic();r=ssh("python3 -B -c 'import json,time,os;print(json.dumps({\"monotonic_s\":time.monotonic(),\"cpu_stat\":open(\"/proc/stat\").readline(),\"loadavg\":open(\"/proc/loadavg\").read().strip(),\"cpus\":os.cpu_count()}))'")
     record({'kind':'ubuntu_cpu','host_send_monotonic_s':sent,'host_receive_monotonic_s':time.monotonic(),'exit':r.returncode,'raw':r.stdout,'stderr':r.stderr})
    count+=1
   except Exception as e:fail.append(str(e));record({'kind':'sample_error','error':str(e)})
 thread=threading.Thread(target=sample,daemon=True);thread.start()
 worker="import os,time,signal,json;os.nice(19);start=time.monotonic();cpu=time.process_time();stop=False\ndef end(*a):\n global stop\n stop=True\nsignal.signal(signal.SIGTERM,end)\nx=1;n=0\nwhile not stop and time.monotonic()-start<120:\n for i in range(20000):x=(x*1664525+1013904223)&0xffffffff\n n+=1\nprint(json.dumps({'pid':os.getpid(),'start_monotonic_s':start,'end_monotonic_s':time.monotonic(),'cpu_s':time.process_time()-cpu,'iterations':n,'nice':os.nice(0)}))"
 try:
  time.sleep(2)
  if a.load=='higher':
   for i in range(2):
    f=(out/('load-worker-'+str(i)+'.log')).open('x');p=subprocess.Popen([sys.executable,'-B','-c',worker],stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append((p,f));record({'kind':'owned_load_start','pid':p.pid,'worker':i})
   time.sleep(2)
  argv=['python3','-B','tools/run_trial.py','--scenario',a.scenario,'--method','mppi','--profile',a.profile,'--run-id',a.run_id]
  remote='source /opt/ros/jazzy/setup.bash && source /home/robot/commercial_mobile_robot/install/p0-06-a3179_6v/setup.bash && cd /home/robot/commercial_mobile_robot && '+shlex.join(argv)
  record({'kind':'runner_start','argv':argv});start=time.monotonic()
  with (out/'runner.log').open('x') as f:
   p=subprocess.Popen(['ssh','-o','BatchMode=yes','cmr-ubuntu',remote],stdout=f,stderr=subprocess.STDOUT)
   rc=p.wait(timeout=480)
  record({'kind':'runner_exit','exit':rc,'elapsed_host_s':time.monotonic()-start})
 finally:
  for p,f in children:
   if p.poll() is None:p.terminate()
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:p.kill();p.wait();fail.append('owned load required kill')
   record({'kind':'owned_load_exit','pid':p.pid,'exit':p.returncode});f.close()
  stop.set();thread.join(timeout=10)
  (out/'sampling-errors.json').write_text(json.dumps(fail,indent=2)+'\n')
 print(json.dumps({'output':str(out),'runner_exit':rc,'sampling_errors':fail}));return rc
