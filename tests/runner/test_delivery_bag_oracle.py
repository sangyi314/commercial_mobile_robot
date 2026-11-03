# SPDX-License-Identifier: MIT
import copy
from tools.delivery.inspect_delivery_bag import stop_window,arrival_window
def fixture():
 who={'attempt_id':'a'};rows=[];line=0
 for n in range(161):
  t=10+n*.02
  def add(kind,data):
   nonlocal line
   line+=1;rows.append({'line':line,'source_s':t,'kind':kind,'data':data})
  add('gate_state',{'allowed':False,'synthetic':False,'identity':who,'clock_epoch':'clock','generation':2})
  add('final',{'linear':[0.,0.,0.],'angular':[0.,0.,0.]})
  add('odom',{'stamp_s':t,'position':[0.,0.,0.],'quaternion_xyzw':[0.,0.,0.,1.],'linear':[0.,0.,0.],'angular':[0.,0.,0.]})
 report={'line':line+1,'data':{'identity':who,'clock_epoch':'clock','generation':2,'from_ros_ns':10000000000,'to_ros_ns':13200000000,'verdict':'STOPPED','synthetic':False}}
 return rows,report
def test_stop_oracle_uses_measurements_and_generation():
 rows,report=fixture();assert stop_window(rows,report)['valid']
 for mutate in ['moving','gap','wrong_generation','nonzero','nan','synthetic']:
  r=copy.deepcopy(rows);p=copy.deepcopy(report)
  if mutate=='moving':
   for x in r:
    if x['kind']=='odom':x['data']['linear'][0]=.02
  elif mutate=='gap':r=[x for x in r if x['kind']!='odom' or not 11<x['data']['stamp_s']<12]
  elif mutate=='wrong_generation':p['data']['generation']=3
  elif mutate=='nonzero':next(x for x in r if x['kind']=='final')['data']['linear'][0]=.1
  elif mutate=='nan':next(x for x in r if x['kind']=='odom')['data']['position'][0]=float('nan')
  else:p['data']['synthetic']=True
  assert not stop_window(r,p)['valid'],mutate
def test_arrival_oracle_rejects_wrong_target_gap_and_preterminal():
 poses=[{'line':i,'data':{'stamp_s':10+i*.1,'position':[0.,0.,0.],'quaternion_xyzw':[0.,0.,0.,1.]}} for i in range(13)]
 detail={'from_ns':10000000000,'to_ns':11200000000};record={'attempt':'a','terminal_ros_ns':9000000000};target={'position':{'x':0.,'y':0.},'orientation':dict(x=0.,y=0.,z=0.,w=1.)}
 assert arrival_window(poses,detail,record,target)['valid']
 assert not arrival_window(poses,detail,{**record,'terminal_ros_ns':10500000000},target)['valid']
 bad=copy.deepcopy(target);bad['position']['x']=1.;assert not arrival_window(poses,detail,record,bad)['valid']
 assert not arrival_window(poses[:3]+poses[8:],detail,record,target)['valid']


def test_consumed_stop_cannot_be_replaced_by_later_good_window():
 from tools.delivery.inspect_delivery_bag import consumed_evidence,exact_stop_report
 record=dict(mission_id='m',operation='o',attempt='a',uuid='u',epoch='owner',gate_epoch='gate',clock_epoch='clock',terminal_ros_ns=9,ever_sent=True,arrived=False,goal_succeeded=False)
 def event(seq,kind,detail='',r=None):
  return dict(record_type='mission_event',reason='APPLIED',persistent_seq=seq,kind=kind,identity=detail,mission_id='m',record=record if r is None else r)
 stop=event(1,8,'D_REPORT x generation=2 from=10 to=20')
 resolve=event(2,15);late=event(3,8,'D_REPORT y generation=2 from=30 to=40')
 bound,arrivals=consumed_evidence([stop,resolve,late]);assert bound[2] is stop and not arrivals
 who=dict(mission_id='m',operation_id='o',attempt_id='a',owner_epoch='owner',gate_epoch='gate')
 def report(lo,hi,**kw):return dict(data=dict(identity=who,clock_epoch='clock',generation=2,from_ros_ns=lo,to_ros_ns=hi,**kw))
 good=report(30,40);bad=report(10,20)
 assert exact_stop_report([good],bound[2],record) is None
 assert exact_stop_report([good,bad],bound[2],record) is bad
 wrong=copy.deepcopy(bad);wrong['data']['identity']['owner_epoch']='old'
 assert exact_stop_report([wrong],bound[2],record) is None
 returning={**record,'attempt':'return','uuid':'new','arrived':False}
 bound,arrivals=consumed_evidence([event(1,9),event(2,15,r=returning)])
 assert bound[2] is None and not arrivals

def test_wheel_stop_does_not_prove_body_stop(tmp_path):
 import json
 from tools.mission.audit_gate_trials import world_window
 rows,report=fixture();assert stop_window(rows,report)['valid']
 run=tmp_path/'case';(run/'runtime').mkdir(parents=True)
 samples=[]
 for i in range(161):
  t=10+i*.02;stamp=round(t*1000000000)
  samples.append(dict(kind='pose',run_id='case',clock_epoch='case:sim0',seq=i,protobuf=dict(header=dict(stamp=dict(sec=stamp//1000000000,nsec=stamp%1000000000)),pose=[dict(name='turtlebot3_waffle',position=dict(x=.02*(t-10),y=0,z=0),orientation=dict(x=0,y=0,z=0,w=1))])))
 (run/'runtime/p0-08-gazebo.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in samples))
 result=world_window(run,dict(from_s=10,to_s=13.2))
 assert result['status']=='CONTRADICTION' and result['prefix_violations']


def test_body_source_independently_rejects_zero_twist_with_motion():
 from tools.delivery.inspect_delivery_bag import body_window
 rows,report=fixture();report['data']['reason']='WINDOW_PROVEN_BODY_POSE'
 body=[]
 for row in rows:
  if row['kind']=='odom':
   b=copy.deepcopy(row);b['kind']='body_odom';body.append(b)
 assert body_window(rows+body,report)['valid']
 for row in body:row['data']['position'][0]=.011*(row['data']['stamp_s']-10)
 assert not body_window(rows+body,report)['valid']
 assert not body_window(rows,report)['valid']


def test_body_receipt_crosscheck_has_its_own_steady_clock():
 from tools.delivery.inspect_delivery_bag import body_receipt_crosscheck
 rows=[dict(kind='body_odom',line=i,receive_monotonic_s=100+i*.02,data=dict(stamp_s=10+i*.02)) for i in range(170)]
 assert body_receipt_crosscheck(rows,10100000000,13200000000)['valid']
 assert not body_receipt_crosscheck([],10100000000,13200000000)['valid']
 rows[80]['receive_monotonic_s']+=.4
 assert not body_receipt_crosscheck(rows,10100000000,13200000000)['valid']


def test_owner_loss_never_reuses_initial_stationary_report():
 from tools.delivery.inspect_delivery_bag import owner_loss_report_candidates
 proof=dict(identity={'owner_epoch':'owner','attempt_id':'a'},clock_epoch='clock',generation=3)
 def report(lo,hi,gen=3):return dict(data=dict(identity=proof['identity'],clock_epoch='clock',generation=gen,from_ros_ns=lo,to_ros_ns=hi))
 initial=report(1,4,0);old=report(1,4);after_restart=report(13,17);correct=report(10,14)
 assert owner_loss_report_candidates([initial,old,after_restart,correct],proof,10,15)==[correct]


def test_stop_oracle_uses_source_time_not_cross_writer_report_arrival():
 rows,report=fixture()
 # Report arrives before the final state/final-topic samples in the recorder.
 report['line']=rows[-10]['line']
 assert stop_window(rows,report)['valid']
 # State writer carries an authoritative source stamp. A reset after the
 # exact report endpoint does not change the earlier generation's evidence.
 for row in rows:
  if row['kind']=='gate_state':row['data']['ros_ns']=round(row['source_s']*1e9)
 tail=copy.deepcopy(rows[-3]);tail['line']=rows[-1]['line']+1;tail['source_s']=13.22;tail['data']['ros_ns']=13220000000;tail['data']['generation']=3;rows.append(tail)
 assert stop_window(rows,report)['valid']
 bad=copy.deepcopy(rows);next(r for r in bad if r['kind']=='gate_state' and r['source_s']>12)['data']['allowed']=True
 assert not stop_window(bad,report)['valid']
