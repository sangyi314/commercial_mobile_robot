"""One-shot NavigateToPose client for this run; timeout cancels only its own accepted goal."""
from pathlib import Path
import os,time,json,math,uuid
import rclpy
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from unique_identifier_msgs.msg import UUID
from rosidl_runtime_py.convert import message_to_ordereddict
run=Path(os.environ['CMR_RUN'])
assert not (run/'runtime/goal-dispatch.json').exists(),'Refusing a second dispatch'
goal=json.loads((run/'artifacts/goal.yaml').read_text())
f=(run/'logs/action-client.jsonl').open('x')
rclpy.init();n=rclpy.create_node('p03_single_goal_client',parameter_overrides=[Parameter('use_sim_time',value=True)])
client=ActionClient(n,NavigateToPose,'/navigate_to_pose')
def emit(kind,data):
 item={'kind':kind,'monotonic_s':time.monotonic(),'ros_s':n.get_clock().now().nanoseconds*1e-9,'data':data}
 f.write(json.dumps(item,allow_nan=False)+'\n');f.flush();return item
def wait(future,seconds):
 end=time.monotonic()+seconds
 while not future.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 return future.done()
try:
 assert client.wait_for_server(timeout_sec=10),'Action server missing'
 m=NavigateToPose.Goal();m.pose.header.frame_id=goal['pose']['header']['frame_id']
 m.pose.header.stamp.sec=goal['pose']['header']['stamp']['sec'];m.pose.header.stamp.nanosec=goal['pose']['header']['stamp']['nanosec']
 for key,value in goal['pose']['pose']['position'].items():setattr(m.pose.pose.position,key,value)
 for key,value in goal['pose']['pose']['orientation'].items():setattr(m.pose.pose.orientation,key,value)
 m.behavior_tree=goal['behavior_tree']
 uid=UUID(uuid=list(uuid.uuid4().bytes));identity=bytes(uid.uuid).hex()
 dispatch=emit('dispatch',{'uuid':identity,'request':message_to_ordereddict(m)})
 (run/'runtime/goal-dispatch.json').write_text(json.dumps(dispatch,indent=2)+'\n')
 sent=client.send_goal_async(m,feedback_callback=lambda msg:emit('feedback',message_to_ordereddict(msg)),goal_uuid=uid)
 assert wait(sent,15),'Acceptance timeout; goal may be unknown; no retry'
 handle=sent.result();emit('response',{'accepted':handle.accepted,'uuid':bytes(handle.goal_id.uuid).hex()})
 if not handle.accepted:
  final=emit('terminal',{'action':'REJECTED','uuid':identity})
 else:
  result=handle.get_result_async()
  if not wait(result,120):
   emit('timeout',{'wall_limit_s':120});cancel=handle.cancel_goal_async();wait(cancel,10)
   emit('cancel_response',message_to_ordereddict(cancel.result()) if cancel.done() else {'unconfirmed':True})
   wait(result,15)
  if result.done():
   response=result.result();status={4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}.get(response.status,'UNKNOWN')
   final=emit('terminal',{'action':status,'status':response.status,'uuid':identity,'result':message_to_ordereddict(response.result)})
  else:final=emit('terminal',{'action':'UNKNOWN','uuid':identity})
 (run/'runtime/goal-terminal.json').write_text(json.dumps(final,indent=2)+'\n')
 print(json.dumps(final,allow_nan=False))
finally:
 f.close();n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
