#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Read-only closed MCAP validation for derived gate profiles; no playback."""
if __package__ in (None,''):
 from pathlib import Path
 from runpy import run_path
 run_path(str(Path(__file__).resolve().parents[1]/'__init__.py'))['bootstrap']()
import argparse,json,math
from pathlib import Path
from tools.runner.runner_core import write_json
from tools.execution.stop_analysis import evaluate
def inspect(run):
 import rosbag2_py
 from rclpy.serialization import deserialize_message
 from rosidl_runtime_py.utilities import get_message
 from rosidl_runtime_py.convert import message_to_ordereddict as dic
 p=json.loads((run/'artifacts/profile.json').read_text());e=p['execution_v1'];ns=e['namespace']
 r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(run/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));types={x.name:x.type for x in r.get_all_topics_and_types()};counts={};rows=[];events=[];clock=0.;seq=0;errors=[]
 stamp=lambda t:t.sec+t.nanosec*1e-9
 while r.has_next():
  t,data,receipt=r.read_next();seq+=1;counts[t]=counts.get(t,0)+1;m=deserialize_message(data,get_message(types[t]));d=None;k=None
  if t=='/clock':clock=stamp(m.clock)
  if t=='/odom':
   pos=m.pose.pose.position;q=m.pose.pose.orientation;v=m.twist.twist;k='odom';d={'stamp_s':stamp(m.header.stamp),'position':[pos.x,pos.y,pos.z],'quaternion_xyzw':[q.x,q.y,q.z,q.w],'linear':[v.linear.x,v.linear.y,v.linear.z],'angular':[v.angular.x,v.angular.y,v.angular.z]}
  elif t==ns+'/final':k='/cmd_vel';d={'linear':[m.linear.x,m.linear.y,m.linear.z],'angular':[m.angular.x,m.angular.y,m.angular.z]}
  elif t==ns+'/state':k='gate_state';d=dic(m)
  elif t==ns+'/stop_report':k='stop_report';d=dic(m)
  elif t=='/cmr_experiment/events':
   v=json.loads(m.data);events.append(v)
   if v['type'].startswith('EXECUTION_'):k='execution_marker';d={'name':v['type'],'data':v['data']}
  if k:rows.append({'kind':k,'data':d,'ros_now_s':clock,'receive_monotonic_s':receipt/1e9,'line':seq})
 for t in p['recording']['topics']:
  if not counts.get(t):errors.append('missing '+t)
 expected={'/cmd_vel_nav':'geometry_msgs/msg/TwistStamped','/cmd_vel_smoothed':'geometry_msgs/msg/TwistStamped',ns+'/candidate':'geometry_msgs/msg/TwistStamped',ns+'/final':'geometry_msgs/msg/Twist'}
 for t,typ in expected.items():
  if types.get(t)!=typ:errors.append('wrong type '+t)
 for marker in ['RECORDING_START','RECORDING_END']:
  if sum(x['type']==marker for x in events)!=1:errors.append('marker count '+marker)
 actions=[json.loads(x) for x in (run/'logs/action-client.jsonl').read_text().splitlines()]
 sends=[a for a in actions if a['kind']=='dispatch'];responses=[a for a in actions if a['kind']=='response'];terms=[a for a in actions if a['kind']=='terminal']
 if len(sends)!=1 or len(responses)!=1 or not responses[0]['data']['accepted'] or len(terms)!=1 or terms[0]['data']['uuid']!=sends[0]['data']['uuid']:errors.append('goal lifecycle/UUID')
 marker='EXECUTION_RESUMED' if e['fault']=='clock-pause' else ('EXECUTION_FAULT_BEFORE' if e['fault'] else 'EXECUTION_NORMAL_REVOKED')
 markers=[r for r in rows if r['kind']=='execution_marker' and r['data']['name']==marker];proof=None
 if markers:
  after=markers[-1]['receive_monotonic_s']
  # Evaluate at the recorded proof boundary, before teardown changes generation.
  proved=[r for r in rows if r['kind']=='execution_marker' and r['data']['name']=='EXECUTION_STOP_PROVED']
  boundary=proved[-1]['receive_monotonic_s'] if proved else float('inf')
  proof=evaluate([r for r in rows if r['receive_monotonic_s']<=boundary],after)
  if not all(proof['checks'].values()):errors.append('independent MCAP A/B/D stop')
 else:errors.append('stop marker absent')
 if e['fault'] and not any(v['type']=='EXECUTION_FAULT_AFTER' for v in events):errors.append('fault not exercised')
 return {'valid':not errors,'errors':errors,'counts':counts,'types':types,'stop':proof,'action':terms[-1]['data'] if terms else None,'C':'CONSUMER_ACK_UNKNOWN','synthetic':False}
if __name__=='__main__':
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--run-dir',required=True);ap.add_argument('--output',required=True);a=ap.parse_args()
 try:report=inspect(Path(a.run_dir))
 except Exception as ex:report={'valid':False,'errors':[type(ex).__name__+': '+str(ex)]}
 write_json(a.output,report);raise SystemExit(0 if report['valid'] else 3)
