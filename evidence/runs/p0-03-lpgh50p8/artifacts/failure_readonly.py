from pathlib import Path
import os,json,time
import rclpy
from lifecycle_msgs.srv import GetState
run=Path(os.environ["CMR_RUN"]);rclpy.init();n=rclpy.create_node("p03_failure_readonly")
start=time.monotonic()
while time.monotonic()-start<3:rclpy.spin_once(n,timeout_sec=.1)
report={"observed_monotonic_s":time.monotonic(),"nodes":n.get_node_names_and_namespaces(),"services":n.get_service_names_and_types(),"lifecycle":{}}
for name in ("map_server","amcl"):
 c=n.create_client(GetState,"/"+name+"/get_state")
 ready=c.wait_for_service(timeout_sec=3)
 if ready:
  f=c.call_async(GetState.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=3)
  report["lifecycle"][name]={"service_ready":True,"state":{"id":f.result().current_state.id,"label":f.result().current_state.label} if f.done() and f.result() else None}
 else:report["lifecycle"][name]={"service_ready":False,"state":None}
 n.destroy_client(c)
n.destroy_node();rclpy.shutdown()
processes=[]
for p in Path("/proc").glob("[0-9]*"):
 try:
  env=dict(x.split("=",1) for x in (p/"environ").read_text().split("\0") if "=" in x)
  if env.get("CMR_RUN")!=str(run) or int(p.name)==os.getpid():continue
  tasks=[]
  for t in (p/"task").iterdir():
   item={"tid":int(t.name),"comm":(t/"comm").read_text().strip(),"wchan":(t/"wchan").read_text().strip()}
   tasks.append(item)
  processes.append({"pid":int(p.name),"start_ticks":(p/"stat").read_text().split()[21],"cmdline":(p/"cmdline").read_bytes().replace(b"\0",b" ").decode(errors="replace"),"tasks":tasks})
 except (OSError,UnicodeError):pass
report["owned_processes"]=processes
(run/"runtime/failure-readonly.json").write_text(json.dumps(report,indent=2)+"\n")
print(json.dumps({"lifecycle":report["lifecycle"],"nodes":report["nodes"],"processes":[{"pid":p["pid"],"cmdline":p["cmdline"][:160],"main_wchan":p["tasks"][0]["wchan"]} for p in processes]},indent=2))
