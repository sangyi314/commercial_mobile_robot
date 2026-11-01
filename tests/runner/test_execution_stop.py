"""Independent hand-calculated stop evidence and derived-route rejection tests."""
import copy
from tools.execution.stop_analysis import evaluate
def fixture():
 who={'run_id':'r','gate_epoch':'g','owner_epoch':'o','mission_id':'m','operation_id':'op','attempt_id':'a'}
 rows=[]
 for i in range(34):
  t=10+i*.1
  def row(k,d):rows.append({'kind':k,'receive_monotonic_s':t,'ros_now_s':t,'data':d,'line':len(rows)+1})
  row('gate_state',{'identity':who,'generation':2,'clock_epoch':'c','allowed':False,'synthetic':False})
  row('/cmd_vel',{'linear':[0.,0.,0.],'angular':[0.,0.,0.]})
  row('odom',{'stamp_s':t,'position':[1.,2.,0.],'quaternion_xyzw':[0.,0.,0.,1.],'linear':[0.,0.,0.],'angular':[0.,0.,0.]})
  row('stop_report',{'identity':who,'generation':2,'clock_epoch':'c','verdict':'STOPPED','synthetic':False,'from_ros_ns':int(10e9)})
 return rows
def test_hand_calculated_stationary_window():
 r=evaluate(fixture(),9.,13.31)
 assert all(r['checks'].values())
 assert r['window']['drift_m']==0 and r['window']['yaw_drift_rad']==0
 assert r['C']=='CONSUMER_ACK_UNKNOWN'
def test_counterexamples():
 base=fixture()
 for mode in ['drift','speed','yaw','nonzero','duplicate','receive_gap','old_generation','old_clock','wrong_identity','synthetic','stale','paused']:
  rows=copy.deepcopy(base)
  for row in rows:
   if row['kind']=='odom':
    if mode=='drift':row['data']['position'][0]+=(row['receive_monotonic_s']-10)*.02
    if mode=='speed':row['data']['linear'][0]=.02
    if mode=='yaw':row['data']['quaternion_xyzw']=[0,0,.1,1]
    if mode in ['duplicate','paused']:row['data']['stamp_s']=10
    if mode=='receive_gap' and row['receive_monotonic_s']>11:row['receive_monotonic_s']+=.3
   if row['kind']=='/cmd_vel' and mode=='nonzero':row['data']['linear'][0]=.1
   if row['kind']=='stop_report':
    if mode=='old_generation':row['data']['generation']=1
    if mode=='old_clock':row['data']['clock_epoch']='old'
    if mode=='wrong_identity':row['data']['identity']={'mission_id':'other'}
    if mode=='synthetic':row['data']['synthetic']=True
  r=evaluate(rows,9.,14 if mode=='stale' else 13.31)
  assert not all(r['checks'].values()),mode

def test_world_pose_window_keeps_deceleration_and_rejects_continuous_motion(tmp_path):
 import json
 from tools.mission.audit_gate_trials import world_window
 for mode in ['stationary','deceleration','moving','drift']:
  run=tmp_path/mode;(run/'runtime').mkdir(parents=True);rows=[]
  for i in range(41):
   x=0 if mode=='stationary' else (.002 if i else 0) if mode=='deceleration' else i*(.002 if mode=='moving' else .0009)
   rows.append({'seq':i+1,'kind':'pose','run_id':run.name,'clock_epoch':run.name+':sim0','protobuf':{'header':{'stamp':{'sec':1+i//10,'nsec':i%10*100000000}},'pose':[{'name':'turtlebot3_waffle','position':{'x':x,'y':0,'z':0},'orientation':{'w':1}}]}})
  (run/'runtime/p0-08-gazebo.jsonl').write_text('\n'.join(json.dumps(x) for x in rows))
  result=world_window(run,{'from_s':1,'to_s':5})
  if mode in ['stationary','deceleration']:
   assert result['status']=='STOPPED' and result['span_s']>=3
   assert result['full_raw_samples']==41
   if mode=='deceleration':assert result['deceleration_prefix_violations']
  else:assert result['status']=='CONTRADICTION',result
