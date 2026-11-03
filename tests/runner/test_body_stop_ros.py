# SPDX-License-Identifier: MIT
"""Isolated sink-body ROS protocol proof; all feedback is explicitly synthetic."""
import json,os,signal,subprocess,time,uuid
from pathlib import Path
import pytest
pytestmark=pytest.mark.ros_contract

def test_body_feedback_required_and_stale_is_unknown(tmp_path):
 executable=os.environ.get('CMR_BODY_OBSERVER_BIN')
 if not executable:pytest.skip('explicit new observer binary required')
 import rclpy
 from rclpy.qos import QoSProfile,ReliabilityPolicy
 from cmr_interfaces.msg import GateState,StopReport
 from geometry_msgs.msg import Twist
 from nav_msgs.msg import Odometry
 from rosgraph_msgs.msg import Clock
 assert os.environ['ROS_DOMAIN_ID']=='72'
 rclpy.init();ns='/cmr_test/body_'+uuid.uuid4().hex[:8];sink=rclpy.create_node('execution_sink',namespace=ns);gate=rclpy.create_node('execution_gate',namespace=ns)
 qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
 clock=sink.create_publisher(Clock,ns+'/clock',qos);wheel=sink.create_publisher(Odometry,ns+'/odom',qos);body=sink.create_publisher(Odometry,ns+'/body_odom',qos);state=gate.create_publisher(GateState,ns+'/state',qos);final=gate.create_publisher(Twist,ns+'/final',qos)
 reports=[];sink.create_subscription(StopReport,ns+'/stop_report',lambda m:reports.append((time.monotonic(),m)),qos)
 log=(tmp_path/'observer.log').open('w');proc=subprocess.Popen([executable,'--ros-args','-r','__ns:='+ns,'-p','mode:=sink_body','-p','use_sim_time:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 start=time.monotonic();x=0.;raw=[]
 def pump(seconds,moving=False,body_on=True):
  nonlocal x
  until=time.monotonic()+seconds
  while time.monotonic()<until:
   assert proc.poll() is None,'observer unexpectedly exited'
   t=time.monotonic();stamp=10000000000+round((t-start)*1e9);c=Clock();c.clock.sec=stamp//1000000000;c.clock.nanosec=stamp%1000000000;clock.publish(c)
   g=GateState();g.identity.run_id='r';g.identity.gate_epoch='g';g.identity.owner_epoch='owner';g.identity.mission_id='m';g.identity.operation_id='o';g.identity.attempt_id='a';g.clock_epoch='clock';g.lease_id='lease';g.generation=1;g.synthetic=True;state.publish(g);final.publish(Twist())
   w=Odometry();w.header.stamp=c.clock;w.header.frame_id='odom';w.child_frame_id='base_link';w.pose.pose.orientation.w=1.;wheel.publish(w)
   if body_on:
    if moving:x+=.0004
    b=Odometry();b.header.stamp=c.clock;b.header.frame_id='body_world';b.child_frame_id='body_base';b.pose.pose.orientation.w=1.;b.pose.pose.position.x=x;body.publish(b)
   raw.append(dict(source_ns=stamp,body_x=x,moving=moving,body_on=body_on,synthetic=True))
   rclpy.spin_once(sink,timeout_sec=.01);time.sleep(.01)
 try:
  pump(4.2,moving=True);assert reports and not any(m.verdict=='STOPPED' for _,m in reports),'wheel zero cannot hide body motion'
  reports.clear();pump(4.3);assert any(m.verdict=='STOPPED' and m.synthetic and m.reason=='WINDOW_PROVEN_BODY_POSE' for _,m in reports)
  reports.clear();pump(.8,body_on=False);assert reports[-1][1].verdict=='STOP_UNKNOWN','missing body cannot reuse last good evidence'
 finally:
  if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=3);pytest.fail('observer failed bounded SIGINT exit')
  log.close();(tmp_path/'body-inputs.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in raw));(tmp_path/'body-output.json').write_text(json.dumps([dict(verdict=m.verdict,reason=m.reason,synthetic=m.synthetic) for _,m in reports]));sink.destroy_node();gate.destroy_node();rclpy.shutdown()
 assert proc.returncode==0
