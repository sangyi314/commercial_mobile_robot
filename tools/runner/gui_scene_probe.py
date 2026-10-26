"""Read-only Gazebo scene probe. Invoked and reaped by the original supervisor."""
if __package__ in (None, ''):
    from pathlib import Path
    from runpy import run_path
    run_path(str(Path(__file__).resolve().parents[1] / '__init__.py'))['bootstrap']()
import json, os, sys, threading, time
from pathlib import Path
from google.protobuf.json_format import MessageToDict
from gz.transport13 import Node
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.msgs10.pose_v_pb2 import Pose_V

def validate_scene(scene):
 names=[];by_name={}
 def visit(models):
  for m in models:
   names.append(m.name);by_name[m.name]=m
   visit(m.model)
 visit(scene.model)
 missing={'ground_plane','turtlebot3_waffle','turtlebot3_world'}-set(names)
 if missing: raise ValueError('SCENE_ENTITIES_MISSING:'+','.join(sorted(missing)))
 def visual_count(m):
  return sum(len(link.visual) for link in m.link)+sum(visual_count(child) for child in m.model)
 for name in ['ground_plane','turtlebot3_waffle','turtlebot3_world']:
  if not visual_count(by_name[name]):raise ValueError('SCENE_VISUALS_MISSING:'+name)
 return names

def main():
 run=Path(sys.argv[1]);expected='cmr_'+run.name
 if os.environ.get('GZ_PARTITION')!=expected: raise ValueError('SCENE_PARTITION_MISMATCH')
 node=Node();topics=[];lock=threading.Lock();samples={k:[] for k in ['clock','stats','pose']}
 def cb(k,m):
  with lock:
   if len(samples[k])<3:samples[k].append(MessageToDict(m,preserving_proto_field_name=True))
 try:
  # Complete synchronous request before Python callbacks start (Transport 13).
  ok,scene=node.request('/world/default/scene/info',Empty(),Empty,Scene,2000)
  if not ok:raise ValueError('SCENE_RESPONSE_TIMEOUT')
  for kind,typ in [('clock',Clock),('stats',WorldStatistics),('pose',Pose_V)]:
   topic='/world/default/'+('pose/info' if kind=='pose' else kind)
   topics.append(topic)
   if not node.subscribe(typ,topic,lambda m,k=kind:cb(k,m)):raise ValueError('SUBSCRIBE_FAILED:'+topic)
  names=validate_scene(scene);robot_id=next(m.id for m in scene.model if m.name=='turtlebot3_waffle')
  end=time.monotonic()+4
  while time.monotonic()<end:
   with lock:done=all(len(v)>=3 for v in samples.values())
   if done:break
   time.sleep(.02)
  with lock:data=dict(samples)
  def ns(d):return int(d.get('sec',0))*1000000000+int(d.get('nsec',0))
  for k,field in [('clock','sim'),('stats','sim_time')]:
   times=[ns(x.get(field,{})) for x in data[k]]
   if len(times)<2 or times[-1]<=times[0]:raise ValueError('SCENE_CLOCK_NOT_ADVANCING:'+k)
  poses=[p for x in data['pose'] for p in x.get('pose',[]) if int(p.get('id',-1))==robot_id]
  if not poses:raise ValueError('ROBOT_POSE_MISSING')
 finally:
  for topic in topics:node.unsubscribe(topic)
  del node
 print(json.dumps({'valid':True,'partition':expected,'models':names,'robot_id':robot_id,'scene':MessageToDict(scene,preserving_proto_field_name=True),'samples':data,'gui_received_entities':None,'visual_confirmed':None,'scope':'independent server scene response and live updates; GUI consumption and pixels require separate evidence'}))
if __name__=='__main__':
 try:main()
 except Exception as exc:
  print(json.dumps({'valid':False,'error':type(exc).__name__+': '+str(exc)}));sys.exit(2)
