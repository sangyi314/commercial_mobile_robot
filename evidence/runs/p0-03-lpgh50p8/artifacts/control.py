"""Single-run process/capture helper. All commands inherit the same session.bash."""
from pathlib import Path
import os,sys,json,time,subprocess
run=Path(os.environ["CMR_RUN"])
assert run.is_dir() and os.environ["ROS_DOMAIN_ID"]=="71" and os.environ["GZ_PARTITION"]=="cmr_"+run.name
mode,label,*cmd=sys.argv[1:]
stamp={"monotonic_s":time.monotonic(),"mode":mode,"label":label,"argv":cmd,"session_sha256":__import__("hashlib").sha256((run/"session.bash").read_bytes()).hexdigest(),"environment":{k:os.environ.get(k) for k in ("CMR_RUN","ROS_DOMAIN_ID","GZ_PARTITION","DISPLAY","XAUTHORITY")}}
with (run/"logs/control-events.jsonl").open("a") as f:f.write(json.dumps(stamp)+"\n")
if mode=="start":
 assert not (run/"runtime"/(label+"-process.json")).exists()
 logfile={"gazebo":"gazebo.log","robot":"robot-launch.log","nav2":"nav2-launch.log","observer":"observer.log"}[label]
 out=(run/"logs"/logfile).open("x")
 p=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
 d={"pid":p.pid,"start_ticks":Path(f"/proc/{p.pid}/stat").read_text().split()[21],"argv":cmd,"started_monotonic_s":time.monotonic(),"run":str(run)}
 (run/"runtime"/(label+"-process.json")).write_text(json.dumps(d,indent=2)+"\n")
 print(json.dumps(d))
elif mode=="capture":
 limit=float(cmd.pop(0));t=time.monotonic()
 with (run/"logs"/(label+".out")).open("x") as out,(run/"logs"/(label+".err")).open("x") as err:
  try:r=subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=out,stderr=err,timeout=limit);code=r.returncode
  except subprocess.TimeoutExpired:code=124
 (run/"logs"/(label+".exit")).write_text(str(code)+"\n")
 print(json.dumps({"label":label,"exit":code,"elapsed_s":time.monotonic()-t}))
 if code:print((run/"logs"/(label+".err")).read_text()[-1200:])
 sys.exit(code)
else:raise RuntimeError(mode)
