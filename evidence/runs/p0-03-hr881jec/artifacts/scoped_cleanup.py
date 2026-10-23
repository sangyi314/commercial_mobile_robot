"""One-run scoped normal shutdown. Signal each child once, keep clock until RSP exits."""
from pathlib import Path
import os,json,subprocess,time,signal,argparse,datetime
p=argparse.ArgumentParser();p.add_argument('--navigation-ready',action='store_true');args=p.parse_args()
run=Path(os.environ['CMR_RUN']);events=[]
def emit(k,d):
 events.append({'kind':k,'monotonic_s':time.monotonic(),'data':d})
 (run/'logs/cleanup.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in events))
def alive(pid):
 p=Path(f'/proc/{pid}/stat');return p.exists() and p.read_text().split()[2]!='Z'
def owned(pid):
 p=Path(f'/proc/{pid}');env=dict(x.split('=',1) for x in (p/'environ').read_text().split('\0') if '=' in x)
 assert env.get('CMR_RUN')==str(run),'Not this run'
 return {'pid':pid,'start_ticks':(p/'stat').read_text().split()[21],'argv':(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')}
def interrupt(d,limit=5):
 pid=d['pid']
 if not alive(pid):return
 assert owned(pid)['start_ticks']==d['start_ticks'],'PID reused'
 os.kill(pid,signal.SIGINT);emit('single_SIGINT',d)
 end=time.monotonic()+limit
 while alive(pid) and time.monotonic()<end:time.sleep(.1)
 emit('after_SIGINT',{'pid':pid,'exited':not alive(pid)})
managers=['navigation','localization'] if args.navigation_ready else ['localization']
for manager in managers:
 r=subprocess.run(['ros2','service','call',f'/lifecycle_manager_{manager}/manage_nodes','nav2_msgs/srv/ManageLifecycleNodes','{command: 4}'],capture_output=True,text=True,timeout=25)
 for suffix,v in [('out',r.stdout),('err',r.stderr),('exit',str(r.returncode)+'\n')]: (run/'logs'/f'shutdown-{manager}.{suffix}').write_text(v)
 emit('lifecycle_shutdown',{'manager':manager,'exit':r.returncode,'success':'success=True' in r.stdout})
roots=[]
for name in ['nav2-process.json','robot-process.json']:
 f=run/'runtime'/name
 if not f.exists():continue
 d=json.loads(f.read_text());roots.append(d)
 if not alive(d['pid']):continue
 children=[]
 for pp in Path('/proc').iterdir():
  if not pp.name.isdigit():continue
  try:
   if pp.stat().st_uid==os.getuid() and (pp/'stat').read_text().split()[3]==str(d['pid']):children.append(owned(int(pp.name)))
  except (FileNotFoundError,PermissionError):pass
 # Each bridge remains until RSP and the other nodes have completed; no whole-group signal.
 children.sort(key=lambda c:0 if '/robot_state_publisher/' in c['argv'] else 2 if '/parameter_bridge ' in c['argv'] else 1)
 emit('verified_children',children)
 for child in children:interrupt(child,limit=5)
 interrupt(d,limit=14)
gz=json.loads((run/'runtime/gazebo-process.json').read_text());interrupt(gz,limit=8);roots.append(gz)
nodes=subprocess.run(['ros2','node','list','--no-daemon','--spin-time','3'],capture_output=True,text=True,timeout=12)
for suffix,v in [('out',nodes.stdout),('err',nodes.stderr),('exit',str(nodes.returncode)+'\n')]: (run/'logs'/f'nodes-after.{suffix}').write_text(v)
abnormal=[];clean=[];remaining=[]
for name in ['nav2-launch.log','robot-launch.log']:
 f=run/'logs'/name
 if not f.exists():continue
 for i,line in enumerate(f.read_text(errors='replace').splitlines(),1):
  if any(t in line for t in ['process has died','Caught exception','failed to terminate','escalating to']):abnormal.append({'file':name,'line':i,'text':line})
  if 'process has finished cleanly' in line:clean.append({'file':name,'line':i,'text':line})
for root in roots:
 if alive(root['pid']):remaining.append(root)
for pp in Path('/proc').iterdir():
 if not pp.name.isdigit():continue
 try:
  if pp.stat().st_uid!=os.getuid() or not alive(int(pp.name)):continue
  env=dict(x.split('=',1) for x in (pp/'environ').read_text().split('\0') if '=' in x)
  exe=(pp/'cmdline').read_bytes().split(b'\0')[0].decode(errors='replace')
  if env.get('CMR_RUN')==str(run) and any(t in exe for t in ['/robot_state_publisher','/controller_server','/parameter_bridge','/route_server','/collision_monitor','/bt_navigator']):remaining.append(owned(int(pp.name)))
 except (FileNotFoundError,PermissionError):pass
success=not remaining and nodes.returncode==0 and not nodes.stdout.strip() and not abnormal and all(x['data']['success'] for x in events if x['kind']=='lifecycle_shutdown')
assessment={'cleanup_status':'SUCCEEDED' if success else 'FAILED','resources_released':not remaining,'remaining_robot_processes':remaining,'node_list_exit':nodes.returncode,'node_list':nodes.stdout.splitlines(),'abnormal_exit_lines':abnormal,'clean_exit_lines':clean,'lifecycle_shutdowns':[x for x in events if x['kind']=='lifecycle_shutdown']}
(run/'runtime/cleanup-evidence.json').write_text(json.dumps(assessment,indent=2)+'\n')
start=json.loads((run/'repo-at-start.json').read_text());end={'ended_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat().replace('+00:00','Z'),'end_monotonic_s':time.monotonic(),'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
end['elapsed_monotonic_s']=end['end_monotonic_s']-start['start_monotonic_s']
(run/'runtime/session-ended.json').write_text(json.dumps(end,indent=2)+'\n')
print(json.dumps({'cleanup':assessment['cleanup_status'],'resources_released':not remaining,'abnormal_count':len(abnormal),'clean_exit_count':len(clean),'nodes':nodes.stdout.splitlines()},indent=2))
