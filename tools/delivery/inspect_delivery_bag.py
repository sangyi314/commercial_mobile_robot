#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Independent closed MCAP oracle. No Node, playback, service or Goal client."""
if __package__ in (None,''):
 from pathlib import Path
 from runpy import run_path
 run_path(str(Path(__file__).resolve().parents[1]/'__init__.py'))['bootstrap']()
import argparse,json,math,re
from pathlib import Path
from tools.delivery.records import audit_database
from tools.execution.stop_analysis import yaw
from tools.runner.runner_core import write_json
def stamp_ns(value):return round(value*1000000000)
def angular(a,b):return abs(math.atan2(math.sin(a-b),math.cos(a-b)))
def span(values,maxgap,minimum):
 return len(values)>=5 and values[-1]-values[0]>=minimum and all(0<b-a<=maxgap for a,b in zip(values,values[1:]))
def stop_window(rows,report):
 """Evaluate source-time measurements, never interpret bag ROS time as wall time."""
 d=report['data'];lo=d['from_ros_ns']*1e-9;hi=d['to_ros_ns']*1e-9
 result={'valid':False,'from_s':lo,'to_s':hi,'report_line':report['line'],'attempt':d['identity']['attempt_id'],'generation':d['generation'],'C':'CONSUMER_ACK_UNKNOWN'}
 if d['synthetic'] or d['verdict']!='STOPPED' or d['to_ros_ns']-d['from_ros_ns']<3000000000:return result
 od=[r for r in rows if r['kind']=='odom' and d['from_ros_ns']<=stamp_ns(r['data']['stamp_s'])<=d['to_ros_ns']]
 if not span([stamp_ns(r['data']['stamp_s']) for r in od],500000000,3000000000):return result
 try:
  p=od[0]['data']['position'];a=yaw(od[0]['data']['quaternion_xyzw'])
  values=[v for r in od for k in ['position','quaternion_xyzw','linear','angular'] for v in r['data'][k]]
  if not all(math.isfinite(x) for x in values):return result
  drift=max(math.dist(p[:2],r['data']['position'][:2]) for r in od);rotation=max(angular(a,yaw(r['data']['quaternion_xyzw'])) for r in od)
  v=max(math.sqrt(sum(x*x for x in r['data']['linear'])) for r in od);w=max(math.sqrt(sum(x*x for x in r['data']['angular'])) for r in od)
  def source_stamp(row):
   return row['data']['ros_ns'] if row['kind']=='gate_state' and 'ros_ns' in row['data'] else stamp_ns(row['source_s'])
  def covering(kind):
   # Separate DDS writers have no common arrival ordering. A later bag line
   # may carry an earlier source sample; the Report line is not a barrier.
   eligible=[r for r in rows if r['kind']==kind]
   before=[r for r in eligible if source_stamp(r)<=d['from_ros_ns']]
   after=[r for r in eligible if source_stamp(r)>=d['to_ros_ns']]
   if not before or not after:return []
   first=before[-1];last=after[0]
   return [r for r in eligible if first['line']<=r['line']<=last['line']]
  states=covering('gate_state');cmds=covering('final')
  # A transition AFTER to_ns must not retroactively invalidate an earlier
  # interval. Check its preceding samples, with bounded source-time age.
  within=[r for r in states if source_stamp(r)<=d['to_ros_ns']]
  A=bool(within) and all(0<=source_stamp(b)-source_stamp(a)<200000000 for a,b in zip(within,within[1:])) and 0<=d['to_ros_ns']-source_stamp(within[-1])<200000000 and all(not r['data']['allowed'] and not r['data']['synthetic'] and r['data']['identity']==d['identity'] and r['data']['clock_epoch']==d['clock_epoch'] and r['data']['generation']==d['generation'] for r in within)
  # Repeated /clock stamps are ordinary; strict advancement belongs to odometry.
  B=len(cmds)>=5 and stamp_ns(cmds[-1]['source_s'])-stamp_ns(cmds[0]['source_s'])>=3000000000 and all(math.isfinite(x) and abs(x)<=.001 for r in cmds for k in ['linear','angular'] for x in r['data'][k])
  D=v<=.01 and w<=.03 and drift<=.02 and rotation<=.03
  result.update(valid=A and B and D,A=A,B=B,D=D,samples=len(od),max_v=v,max_w=w,drift_m=drift,yaw_drift_rad=rotation,odom_lines=[od[0]['line'],od[-1]['line']])
 except (KeyError,ValueError,TypeError):pass
 return result
def body_window(rows,report):
 d=report['data'];lo=d['from_ros_ns'];hi=d['to_ros_ns'];samples=[r for r in rows if r['kind']=='body_odom']
 left=[i for i,r in enumerate(samples) if stamp_ns(r['data']['stamp_s'])<=lo];right=[i for i,r in enumerate(samples) if stamp_ns(r['data']['stamp_s'])>=hi]
 result={'valid':False,'source':'private worldPose odometry; pose differences and reported twist both checked'}
 if not left or not right:return result
 values=samples[max(0,left[-1]-1):right[0]+1]
 if len(values)<5 or hi-lo<3000000000:return result
 try:
  if not all(math.isfinite(v) for r in values for k in ['position','quaternion_xyzw','linear','angular'] for v in r['data'][k]):return result
  origin=values[0]['data'];gaps=[stamp_ns(b['data']['stamp_s'])-stamp_ns(a['data']['stamp_s']) for a,b in zip(values,values[1:])]
  if not all(0<x<=500000000 for x in gaps):return result
  v=max(math.dist(a['data']['position'][:2],b['data']['position'][:2])/(gap*1e-9) for a,b,gap in zip(values,values[1:],gaps));w=max(angular(yaw(a['data']['quaternion_xyzw']),yaw(b['data']['quaternion_xyzw']))/(gap*1e-9) for a,b,gap in zip(values,values[1:],gaps));drift=max(math.dist(origin['position'][:2],r['data']['position'][:2]) for r in values);rotation=max(angular(yaw(origin['quaternion_xyzw']),yaw(r['data']['quaternion_xyzw'])) for r in values)
  reported_v=max(math.sqrt(sum(x*x for x in r['data']['linear'])) for r in values);reported_w=max(math.sqrt(sum(x*x for x in r['data']['angular'])) for r in values)
  result.update(valid=d['reason']=='WINDOW_PROVEN_BODY_POSE' and v<=.01 and w<=.03 and reported_v<=.01 and reported_w<=.03 and drift<=.02 and rotation<=.03,max_pose_v=v,max_pose_w=w,max_reported_v=reported_v,max_reported_w=reported_w,drift_m=drift,yaw_drift_rad=rotation,samples=len(values),bag_lines=[values[0]['line'],values[-1]['line']])
 except (ValueError,KeyError,ZeroDivisionError,TypeError):pass
 return result

def body_receipt_crosscheck(observations,lo,hi):
 body=[r for r in observations if r['kind']=='body_odom'];left=[i for i,r in enumerate(body) if stamp_ns(r['data']['stamp_s'])<=lo];right=[i for i,r in enumerate(body) if stamp_ns(r['data']['stamp_s'])>=hi]
 result={'valid':False,'scope':'independent worker body-feedback steady receipt gaps; separate from source-time motion'}
 if left and right:
  selected=body[max(0,left[-1]-1):right[0]+1];gaps=[b['receive_monotonic_s']-a['receive_monotonic_s'] for a,b in zip(selected,selected[1:])]
  result.update(valid=len(selected)>=5 and bool(gaps) and all(0<x<.2 for x in gaps),max_receive_gap_s=max(gaps) if gaps else None,observer_lines=[selected[0]['line'],selected[-1]['line']])
 return result

def owner_loss_report_candidates(reports,proof,expiry_ns,boundary_ns):
 return [r for r in reports if r['data']['identity']==proof['identity'] and r['data']['clock_epoch']==proof['clock_epoch'] and r['data']['generation']==proof['generation'] and r['data']['from_ros_ns']>=expiry_ns and r['data']['to_ros_ns']<=boundary_ns]

def arrival_window(poses,detail,record,target):
 lo=detail['from_ns']*1e-9;hi=detail['to_ns']*1e-9
 selected=[];last=-1
 for row in poses:
  t=row['data']['stamp_s']
  if detail['from_ns']<=stamp_ns(t)<=detail['to_ns'] and t>last:selected.append(row);last=t
 result={'valid':False,'from_s':lo,'to_s':hi,'attempt':record['attempt']}
 if not span([stamp_ns(r['data']['stamp_s']) for r in selected],200000000,1000000000) or detail['from_ns']<record['terminal_ros_ns']:return result
 gp=target['position'];q=target['orientation'];gy=yaw([q[k] for k in ['x','y','z','w']])
 errors=[(math.dist(r['data']['position'][:2],[gp['x'],gp['y']]),angular(yaw(r['data']['quaternion_xyzw']),gy)) for r in selected]
 continuous=True
 for a,b in zip(selected,selected[1:]):
  dt=b['data']['stamp_s']-a['data']['stamp_s']
  if math.dist(a['data']['position'][:2],b['data']['position'][:2])>.1+.5*dt or angular(yaw(a['data']['quaternion_xyzw']),yaw(b['data']['quaternion_xyzw']))>.1+1.5*dt:continuous=False
 result.update(valid=continuous and max(x[0] for x in errors)<=.25 and max(x[1] for x in errors)<=.25,samples=len(selected),max_position_error_m=max(x[0] for x in errors),max_yaw_error_rad=max(x[1] for x in errors),bag_lines=[selected[0]['line'],selected[-1]['line']])
 return result
def observer_crosscheck(run,proof,observations):
 lo=stamp_ns(proof['from_s']);hi=stamp_ns(proof['to_s'])
 od=[r for r in observations if r['kind']=='odom' and lo<=stamp_ns(r['data']['stamp_s'])<=hi]
 if not span([stamp_ns(r['data']['stamp_s']) for r in od],500000000,3000000000):return {'valid':False,'reason':'observer odom window missing'}
 first,last=od[0]['receive_monotonic_s'],od[-1]['receive_monotonic_s']
 gaps=[b['receive_monotonic_s']-a['receive_monotonic_s'] for a,b in zip(od,od[1:])]
 cmds=[r for r in observations if r['kind']=='/cmd_vel' and first<=r['receive_monotonic_s']<=last]
 cgaps=[b['receive_monotonic_s']-a['receive_monotonic_s'] for a,b in zip(cmds,cmds[1:])]
 valid=bool(cmds and cgaps) and all(0<x<.2 for x in gaps+cgaps) and cmds[0]['receive_monotonic_s']-first<.2 and last-cmds[-1]['receive_monotonic_s']<.2 and all(abs(x)<=.001 for r in cmds for k in ['linear','angular'] for x in r['data'][k])
 return {'valid':valid,'scope':'independent worker receipt steady clock; MCAP time is never used for wall gaps','max_odom_receive_gap_s':max(gaps),'max_final_receive_gap_s':max(cgaps) if cgaps else None,'observer_lines':[od[0]['line'],od[-1]['line']]}
def consumed_evidence(events):
 """Bind facts to the exact applied Resolve, never to a later convenient report."""
 stops={};arrivals={};consumed_stops={};consumed_arrivals=[]
 for ev in events:
  if ev['record_type']!='mission_event' or ev['reason']!='APPLIED':continue
  r=ev['record'];key=(ev['mission_id'],r['attempt'],r['uuid'],r['gate_epoch'],r['clock_epoch'])
  if ev['kind']==8:stops[key]=ev
  if ev['kind']==9:arrivals[key]=ev
  if ev['kind']==15:
   if r['ever_sent']:consumed_stops[ev['persistent_seq']]=stops.get(key)
   if r['arrived'] and r['goal_succeeded'] and key in arrivals and arrivals[key] not in consumed_arrivals:consumed_arrivals.append(arrivals[key])
 return consumed_stops,consumed_arrivals

def exact_stop_report(reports,event,record):
 if event is None:return None
 match=re.search(r' generation=(\d+) from=(\d+) to=(\d+)(?:$| )',event['identity'])
 if not match:return None
 generation,lo,hi=map(int,match.groups())
 for report in reports:
  d=report['data'];who=d['identity']
  if (d['generation'],d['from_ros_ns'],d['to_ros_ns'])!=(generation,lo,hi):continue
  if d['clock_epoch']!=record['clock_epoch'] or hi<record['terminal_ros_ns']:continue
  if any(who.get(k)!=record[v] for k,v in [('mission_id','mission_id'),('operation_id','operation'),('attempt_id','attempt'),('owner_epoch','epoch'),('gate_epoch','gate_epoch')]):continue
  return report
 return None

def inspect(run):
 import rosbag2_py,tf2_py,yaml
 from rclpy.time import Time
 from rclpy.duration import Duration
 from rclpy.serialization import deserialize_message
 from rosidl_runtime_py.utilities import get_message
 from rosidl_runtime_py.convert import message_to_ordereddict as dic
 run=Path(run);p=json.loads((run/'artifacts/profile.json').read_text());ns=p['execution_v1']['namespace'];db=audit_database(run/'runtime/mission.db');errors=list(db['errors'])
 if p.get('profile_version') not in [4,5] or p['delivery_v1'].get('synthetic_observations') is not False:raise ValueError('wrong delivery evidence layer')
 md=yaml.safe_load((run/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
 if not md['relative_file_paths']:raise ValueError('empty bag')
 for f in md['relative_file_paths']:
  path=(run/'bag'/f).resolve()
  if not path.is_relative_to((run/'bag').resolve()) or not path.is_file() or path.stat().st_size==0:raise ValueError('invalid bag shard')
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(run/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
 types={x.name:x.type for x in reader.get_all_topics_and_types()};classes={k:get_message(v) for k,v in types.items()};counts={};rows=[];poses=[];events=[];statuses={};maps_good=0;source=0.;seq=0;buffer=tf2_py.BufferCore(Duration(seconds=1200))
 stamp=lambda t:t.sec+t.nanosec*1e-9
 while reader.has_next():
  topic,raw,bagtime=reader.read_next();seq+=1;counts[topic]=counts.get(topic,0)+1;m=deserialize_message(raw,classes[topic]);kind=None;d=None
  if topic=='/clock':
   now=stamp(m.clock)
   if now<source:errors.append('source clock rollback at '+str(seq))
   source=now
  elif topic in ['/tf','/tf_static']:
   for tr in m.transforms:
    if topic=='/tf_static':buffer.set_transform_static(tr,'recorded')
    else:buffer.set_transform(tr,'recorded')
  elif topic=='/map':
   import hashlib
   frozen=json.loads((run/'artifacts/stations.v1.json').read_text())['map']['metadata']
   origin=m.info.origin;expected_yaw=frozen['origin_yaw'];origin_good=abs(origin.position.x-frozen['origin_x'])<1e-7 and abs(origin.position.y-frozen['origin_y'])<1e-7 and abs(origin.position.z)<1e-7 and abs(origin.orientation.x)<1e-7 and abs(origin.orientation.y)<1e-7 and abs(origin.orientation.z-math.sin(expected_yaw/2))<1e-7 and abs(origin.orientation.w-math.cos(expected_yaw/2))<1e-7
   if origin_good and m.header.frame_id=='map' and m.info.width==frozen['width'] and m.info.height==frozen['height'] and abs(m.info.resolution-frozen['resolution'])<1e-7 and hashlib.sha256(bytes(x%256 for x in m.data)).hexdigest()==frozen['grid_sha256']:maps_good+=1
   else:errors.append('actual map grid mismatch')
  elif topic=='/odom' or topic==ns+'/body_odom':
   pos=m.pose.pose.position;q=m.pose.pose.orientation;v=m.twist.twist;kind='odom' if topic=='/odom' else 'body_odom';d={'stamp_s':stamp(m.header.stamp),'position':[pos.x,pos.y,pos.z],'quaternion_xyzw':[q.x,q.y,q.z,q.w],'linear':[v.linear.x,v.linear.y,v.linear.z],'angular':[v.angular.x,v.angular.y,v.angular.z]}
   if kind=='odom' and (m.header.frame_id!='odom' or m.child_frame_id not in ['base_link','base_footprint']):errors.append('odom frame')
   if kind=='body_odom' and (m.header.frame_id!='body_world' or m.child_frame_id!='body_base'):errors.append('body odom frame')
  elif topic==ns+'/final':kind='final';d={'linear':[m.linear.x,m.linear.y,m.linear.z],'angular':[m.angular.x,m.angular.y,m.angular.z]}
  elif topic==ns+'/state':kind='gate_state';d=dic(m)
  elif topic==ns+'/stop_report':kind='stop_report';d=dic(m)
  elif topic=='/navigate_to_pose/_action/status':
   for status in m.status_list:statuses.setdefault(bytes(status.goal_info.goal_id.uuid).hex(),set()).add(status.status)
  elif topic=='/cmr_experiment/events':
   e=json.loads(m.data);events.append(e)
   if e.get('run_id')!=run.name:errors.append('event session identity')
  if kind:rows.append({'kind':kind,'data':d,'source_s':source,'line':seq})
  if topic in ['/tf','/odom'] and source>0:
   try:
    tr=buffer.lookup_transform_core('map','base_footprint',Time());t=stamp(tr.header.stamp);pos=tr.transform.translation;q=tr.transform.rotation
    if 0<=source-t<.2:poses.append({'data':{'stamp_s':t,'position':[pos.x,pos.y,pos.z],'quaternion_xyzw':[q.x,q.y,q.z,q.w]},'line':seq})
   except tf2_py.TransformException:pass
 for t in p['recording']['topics']:
  if not counts.get(t):errors.append('required topic empty '+t)
 for t,typ in {ns+'/final':'geometry_msgs/msg/Twist',ns+'/candidate':'geometry_msgs/msg/TwistStamped','/odom':'nav_msgs/msg/Odometry'}.items():
  if types.get(t)!=typ:errors.append('wrong recorded type '+t)
 if p['delivery_v1'].get('stop_feedback',{}).get('required') and types.get(ns+'/body_odom')!='nav_msgs/msg/Odometry':errors.append('wrong body feedback type')
 for marker in ['RECORDING_START','RECORDING_END']:
  if sum(e['type']==marker for e in events)!=1:errors.append('recording marker '+marker)
 if maps_good==0:errors.append('actual map identity not recorded')
 accepted=db['accepted_uuids']
 for uid in accepted:
  if not statuses.get(uid):errors.append('persisted UUID absent from actual Action status '+uid)
 for uid in statuses:
  if uid not in accepted:errors.append('extra Action UUID not owned by persisted Mission '+uid)
 dispatch=[]
 for log in sorted((run/'logs').glob('mission*.log')):
  for line in log.read_text().splitlines():
   if line.startswith('NAV_DISPATCH '):dispatch.append(json.loads(line[len('NAV_DISPATCH '):]))
 if len({x['attempt'] for x in dispatch})!=len(dispatch) or len(dispatch)!=len(db['authorized_send_intents']):errors.append('native sends versus persisted intents')
 reports=[r for r in rows if r['kind']=='stop_report' and r['data']['verdict']=='STOPPED' and not r['data']['synthetic']]
 proofs=[];arrivals=[];snapshots={q['mission_id']:q['snapshot'] for q in db['queue']}
 consumed_stops,consumed_arrivals=consumed_evidence(db['events'])
 for event in db['events']:
  if event['record_type']!='mission_event' or event['reason']!='APPLIED':continue
  record=event['record'];kind=event['kind']
  if kind in [2,3,18] and record['uuid']:
   expected={2:4,3:6,18:5}[kind]
   if expected not in statuses.get(record['uuid'],set()):errors.append('terminal protocol/DB mismatch '+record['attempt'])
  if kind==9 and event in consumed_arrivals:
   detail=json.loads(event['identity'].split(' detail=',1)[1]);target=snapshots[record['mission_id']]['return' if record['returning'] else 'outbound']['pose'];a=arrival_window(poses,detail,record,target);a['event_seq']=event['persistent_seq'];arrivals.append(a)
   if not a['valid']:errors.append('independent arrival window '+record['attempt'])
  if kind==15 and record['ever_sent']:
   consumed=consumed_stops.get(event['persistent_seq']);rep=exact_stop_report(reports,consumed,record)
   proof=stop_window(rows,rep) if rep is not None else None
   if proof is None or not proof['valid']:errors.append('independent exact consumed A/B/D '+record['attempt'])
   if proof is not None:
    proof['event_seq']=event['persistent_seq'];proof['consumed_stop_event_seq']=consumed['persistent_seq'];proofs.append(proof)
 # Owner loss deliberately retains UNKNOWN Goal. Its stop must precede restart.
 if p['delivery_v1']['case']=='G06':
  markers=[e for e in events if e['type']=='DELIVERY_STOP_PROVED']
  faults=[e for e in events if e['type']=='DELIVERY_FAULT_BEFORE']
  if len(markers)!=1 or len(faults)!=1:errors.append('owner loss qualified stop/fault marker missing')
  else:
   bound=markers[0]['data']['proof'];boundary_ns=stamp_ns(markers[0]['ros_s']);fault_ns=stamp_ns(faults[0]['ros_s'])
   expired=[r for r in rows if r['kind']=='gate_state' and r['data']['reason']=='LEASE_EXPIRED' and r['data']['identity']==bound['identity'] and r['data']['clock_epoch']==bound['clock_epoch'] and r['data']['generation']==bound['generation'] and r['data']['ros_ns']>=fault_ns]
   matching=owner_loss_report_candidates(reports,bound,expired[0]['data']['ros_ns'],boundary_ns) if expired else []
   candidate=next((v for rep in matching if (v:=stop_window(rows,rep))['valid']),None)
   if candidate:proofs.append(candidate)
   else:errors.append('owner loss raw A/B/D must follow actual expiry and precede restart; initial/old stop forbidden')
 observations=[json.loads(line) for line in (run/'runtime/observations.jsonl').read_text().splitlines()]
 if p['delivery_v1'].get('stop_feedback',{}).get('required'):
  for proof in proofs:
   report=next(r for r in reports if r['line']==proof['report_line'])
   b=body_window(rows,report);proof['independent_body_feedback']=b
   if not b['valid']:errors.append('independent body feedback '+proof['attempt'])
   body_wall=body_receipt_crosscheck(observations,report['data']['from_ros_ns'],report['data']['to_ros_ns']);proof['independent_body_wall_observer']=body_wall
   if not body_wall['valid']:errors.append('independent body receive gap '+proof['attempt'])
 for proof in proofs:
  cross=observer_crosscheck(run,proof,observations);proof['independent_wall_observer']=cross
  if not cross['valid']:errors.append('independent wall observer gap/zero evidence '+proof['attempt'])
 fault_metrics={}
 if p['delivery_v1']['case'] in ['G03','G06']:
  marks=[r for r in observations if r['kind']=='execution_marker' and r['data']['name']=='DELIVERY_FAULT_BEFORE']
  if len(marks)!=1:errors.append('unique qualified fault marker missing')
  else:
   marker=marks[0];before=[r for r in observations if r['receive_monotonic_s']<=marker['receive_monotonic_s']];status_rows=[r for r in before if r['kind']=='action_status']
   try:
    from tools.delivery.worker_policy import motion_qualification
    uid=marker['data']['data']['goal_uuid'];qualification=motion_qualification(before,status_rows[-1]['data'],uid,marker['ros_now_s'],marker['receive_monotonic_s'])
    forbidden=[r for r in observations if r['kind']=='gate_state' and r['receive_monotonic_s']>=marker['receive_monotonic_s'] and not r['data']['allowed'] and r['data']['identity']['attempt_id']==db['accepted_uuids'][uid]]
    if not forbidden:raise ValueError('forbidden Gate state absent')
    first=forbidden[0];latency=first['receive_monotonic_s']-marker['receive_monotonic_s'];expected='REVOKED' if p['delivery_v1']['case']=='G03' else 'LEASE_EXPIRED'
    if first['data']['reason']!=expected or not 0<=latency<.6:raise ValueError('fault cause or frozen 600ms observation bound')
    body_qualification={}
    if p['delivery_v1'].get('stop_feedback',{}).get('required'):
     from tools.delivery.worker_policy import body_motion_qualification
     body_qualification=body_motion_qualification(before,status_rows[-1]['data'],uid,marker['ros_now_s'],marker['receive_monotonic_s'])
    fault_metrics={'body_qualification':body_qualification,'qualification':qualification,'first_forbidden_observer_line':first['line'],'cause':first['data']['reason'],'authorization_observed_latency_steady_s':latency,'scope':'independent worker steady observation, no hard real-time claim'}
   except (ValueError,KeyError,IndexError) as exc:errors.append('NOT_EXERCISED qualified fault: '+str(exc))
 from tools.mission.audit_gate_trials import world_window
 worlds=[]
 for proof in proofs:
  world=world_window(run,{'from_s':proof['from_s'],'to_s':proof['to_s']});world['attempt']=proof['attempt'];worlds.append(world)
  if world['status']=='CONTRADICTION':errors.append('Gazebo body-pose contradiction '+proof['attempt'])
 return {**({'fault_metrics':fault_metrics} if p['delivery_v1']['case'] in ['G03','G06'] else {}),'valid':not errors,'errors':errors,'world_pose_cross_checks':worlds,'oracle_stop_assisted':bool(p['delivery_v1'].get('stop_feedback',{}).get('oracle_stop_assisted')),'layer':'closed real Gazebo MCAP; source time, not wall latency; Oracle stop assistance explicitly recorded' if p['delivery_v1'].get('stop_feedback',{}).get('oracle_stop_assisted') else 'closed real Gazebo MCAP; source time, not wall latency','synthetic':False,'counts':counts,'types':types,'action_statuses':{k:sorted(v) for k,v in statuses.items()},'native_dispatches':dispatch,'stops':proofs,'arrivals':arrivals,'C':'CONSUMER_ACK_UNKNOWN','collision':None}
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-dir',required=True);p.add_argument('--output',required=True);a=p.parse_args()
 try:r=inspect(Path(a.run_dir))
 except Exception as ex:
  import traceback
  r={'valid':False,'errors':[type(ex).__name__+': '+str(ex)],'traceback':traceback.format_exc()}
 write_json(a.output,r);raise SystemExit(0 if r['valid'] else 3)
