#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Native Gazebo Transport source-graph negative; no server, bridge or actuator."""
if __package__ in (None,''):
 from pathlib import Path
 from runpy import run_path
 run_path(str(Path(__file__).resolve().parents[1]/'__init__.py'))['bootstrap']()
import argparse,json,os,sys,time,uuid
from pathlib import Path
from tools.runner.runner_processes import ProcessSupervisor
from tools.runner.runner_core import DeadlineBudget,write_json
from tools.execution.runner_gate import GateSession
def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output-dir',required=True);ap.add_argument('--check',action='store_true');a=ap.parse_args();out=Path(a.output_dir)
 if out.exists():raise ValueError('output exists')
 if a.check:print('CHECK_OK no process/Node/output');return 0
 out.mkdir(parents=True);(out/'logs').mkdir();(out/'runtime').mkdir();events=(out/'events.jsonl').open('x')
 def log(k,d):events.write(json.dumps({'kind':k,'data':d})+'\n');events.flush()
 env=os.environ.copy();env['GZ_PARTITION']='cmr_route_'+uuid.uuid4().hex;env['GZ_IP']='127.0.0.1';os.environ['GZ_IP']='127.0.0.1'
 from gz.transport13 import Node,NodeOptions
 from gz.msgs10.twist_pb2 import Twist
 opts=NodeOptions();opts.partition=env['GZ_PARTITION'];node=Node(opts);topic='/cmr_execution/route_probe_'+uuid.uuid4().hex;received=[]
 node.subscribe(Twist,topic,lambda m:received.append(time.monotonic()));sup=ProcessSupervisor(out,log);budget=DeadlineBudget.after(25);positive=None;negative=None;failure=None
 code="import os,sys,time\nfrom gz.transport13 import Node,NodeOptions\nfrom gz.msgs10.twist_pb2 import Twist\no=NodeOptions();o.partition=os.environ['GZ_PARTITION'];n=Node(o);p=n.advertise(sys.argv[1],Twist);print('READY',flush=True);end=time.monotonic()+9\nwhile time.monotonic()<end:p.publish(Twist());time.sleep(.03)\np=None;n=None"
 def publisher(role):
  p=sup.spawn(role,[sys.executable,'-u','-c',code,topic],env,Path.cwd())
  while 'READY' not in (out/'logs'/(role+'.log')).read_text():budget.require();sup.assert_critical();time.sleep(.02)
  return p
 counter=0
 def graph():
  nonlocal counter
  counter+=1;p=sup.spawn('graph-'+str(counter),['gz','topic','-i','-t',topic],env,Path.cwd(),critical=False);p.process.wait(timeout=budget.child(4).require());sup.close(p,budget,initial_signal=False,grace=.1)
  if p.process.returncode:raise RuntimeError('native topic query failed')
  return (out/'logs'/('graph-'+str(counter)+'.log')).read_text()
 try:
  one=publisher('reviewed-source')
  while len(received)<2:budget.require();time.sleep(.02)
  positive=graph()
  if not GateSession.route_qualified(positive):raise RuntimeError('positive one-source transport graph unqualified')
  two=publisher('unreviewed-extra-source');until=time.monotonic()+4
  while time.monotonic()<until:
   negative=graph()
   if negative.split('Subscribers')[0].count('gz.msgs.Twist')==2:break
   time.sleep(.05)
  if one.process.poll() is not None or two.process.poll() is not None:raise RuntimeError('NOT_EXERCISED producer ended before graph capture')
  if negative.split('Subscribers')[0].count('gz.msgs.Twist')!=2 or GateSession.route_qualified(negative):raise RuntimeError('extra source was not rejected')
 except Exception as ex:failure=repr(ex)
 finally:
  node.unsubscribe(topic);node=None
  for p in reversed(sup.owned):
   if not p.closed:sup.close(p,budget,initial_signal=False,grace=10)
  cleanup=all(p.result and p.result['ok'] for p in sup.owned)
  result={'case':'E08-native-GZ-source','layer':'Gazebo Transport only; no Gazebo server/Bridge/actuator','partition':env['GZ_PARTITION'],'topic':topic,'positive_graph':positive,'extra_source_graph':negative,'authority_ever_granted':False,'zero_messages_only':True,'received':len(received),'fault_activation':'CONFIRMED' if not failure else 'NOT_EXERCISED','cleanup':cleanup,'error':failure,'pass':failure is None and cleanup}
  write_json(out/'result.json',result);events.close();print(json.dumps(result));return 0 if result['pass'] else 3
if __name__=='__main__':raise SystemExit(main())
