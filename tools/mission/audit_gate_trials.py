#!/usr/bin/env python3
"""Read-only independent gate trial audit; no ROS nodes or bag playback."""
if __package__ in (None,''):
 from pathlib import Path
 from runpy import run_path
 run_path(str(Path(__file__).resolve().parents[1]/'__init__.py'))['bootstrap']()
import argparse,json,math,hashlib
from pathlib import Path
from tools.execution.stop_analysis import evaluate,yaw
from tools.runner.runner_core import write_json
def lines(p):return [json.loads(x) for x in p.read_text().splitlines() if x]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def iter_lines(path):
 with Path(path).open() as stream:
  for line in stream:
   if line.strip():yield json.loads(line)
def world_window(run,window):
 samples=[]
 for row in iter_lines(run/'runtime/p0-08-gazebo.jsonl'):
  if row['kind']!='pose':continue
  assert row['run_id']==run.name and row['clock_epoch']==run.name+':sim0'
  m=row['protobuf'];s=m.get('header',{}).get('stamp',{});t=int(s.get('sec',0))+int(s.get('nsec',0))*1e-9
  if not window['from_s']<=t<=window['to_s']:continue
  for p in m.get('pose',[]):
   if p.get('name')!='turtlebot3_waffle':continue
   v=p['position'];q=p['orientation'];samples.append((t,[float(v.get(k,0)) for k in ['x','y','z']],yaw([float(q.get(k,0)) for k in ['x','y','z','w']]),row['seq']))
 if len(samples)<5:return {'status':'UNKNOWN','samples':len(samples),'reason':'insufficient independent world pose samples'}
 # The wheel-derived and body-pose stationary windows need not start at the
 # same sample. Retain the complete prefix as deceleration evidence, then
 # require a continuous three-second suffix after the last violation.
 original=list(samples);start=0;violations=[]
 angle=lambda a,b:abs(math.atan2(math.sin(a-b),math.cos(a-b)))
 for i in range(1,len(samples)):
  dt=samples[i][0]-samples[i-1][0]
  speed=math.dist(samples[i][1][:2],samples[i-1][1][:2])/dt if dt>0 else float('inf')
  rotation=angle(samples[i][2],samples[i-1][2])/dt if dt>0 else float('inf')
  if dt<=0 or dt>.5 or speed>.01 or rotation>.03 or math.dist(samples[i][1][:2],samples[start][1][:2])>.02 or angle(samples[i][2],samples[start][2])>.03:
   violations.append({'raw_seq':samples[i][3],'sim_s':samples[i][0],'v_xy':speed,'wz':rotation,'gap_s':dt,'motion_or_drift':dt>0 and (speed>.01 or rotation>.03 or math.dist(samples[i][1][:2],samples[start][1][:2])>.02 or angle(samples[i][2],samples[start][2])>.03)});start=i
 samples=samples[start:]
 if len(samples)<5:return {'status':'CONTRADICTION','reason':'no qualified suffix after body-motion violation','prefix_violations':violations}
 gaps=[b[0]-a[0] for a,b in zip(samples,samples[1:])]
 if min(gaps)<=0 or max(gaps)>.5 or samples[-1][0]-samples[0][0]<3:return {'status':'CONTRADICTION' if any(v['motion_or_drift'] for v in violations) else 'UNKNOWN','samples':len(samples),'reason':'world pose window/gap qualification','prefix_violations':violations}
 drift=max(math.dist(x[1][:2],samples[0][1][:2]) for x in samples);angular=lambda a,b:abs(math.atan2(math.sin(a-b),math.cos(a-b)))
 ydrift=max(angular(x[2],samples[0][2]) for x in samples);v=max(math.dist(a[1][:2],b[1][:2])/dt for a,b,dt in zip(samples,samples[1:],gaps));w=max(angular(a[2],b[2])/dt for a,b,dt in zip(samples,samples[1:],gaps))
 ok=drift<=.02 and ydrift<=.03 and v<=.01 and w<=.03
 return {'status':'STOPPED' if ok else 'CONTRADICTION','samples':len(samples),'span_s':samples[-1][0]-samples[0][0],'max_gap_s':max(gaps),'drift_xy_m':drift,'yaw_drift_rad':ydrift,'max_estimated_v_xy':v,'max_estimated_wz':w,'raw_seq_range':[samples[0][3],samples[-1][3]],'source':'independent Gazebo model pose, no command integration','qualified_from_sim_s':samples[0][0],'qualified_to_sim_s':samples[-1][0],'deceleration_prefix_violations':violations,'full_raw_samples':len(original)}
def audit(run):
 p=json.loads((run/'artifacts/profile.json').read_text());e=p['execution_v1'];r=json.loads((run/'runtime/execution-check.json').read_text());cleanup=json.loads((run/'runtime/cleanup.json').read_text());bag=json.loads((run/'runtime/bag-check.json').read_text());validation=json.loads((run/'runtime/validation.json').read_text())
 assert r['case_pass'] and not r['synthetic'] and cleanup['ok'] and not cleanup['remaining'] and all(x['ok'] and not x['forced'] for x in cleanup['processes'])
 assert bag['valid'] and validation['status']=='VALID'
 actions=lines(run/'logs/action-client.jsonl');sent=[x for x in actions if x['kind']=='dispatch'];accept=[x for x in actions if x['kind']=='response'];term=[x for x in actions if x['kind']=='terminal']
 assert len(sent)==len(accept)==len(term)==1 and accept[0]['data']['accepted'] and sent[0]['data']['uuid']==accept[0]['data']['uuid']==term[0]['data']['uuid']
 rows=lines(run/'runtime/observations.jsonl');markers=lambda n:[x for x in rows if x['kind']=='execution_marker' and x['data']['name']==n]
 name='EXECUTION_NORMAL_REVOKED' if not e['fault'] else 'EXECUTION_RESUMED' if e['fault']=='clock-pause' else 'EXECUTION_FAULT_BEFORE'
 start=markers(name)[-1];end=markers('EXECUTION_STOP_PROVED')[-1];relevant=[x for x in rows if x['receive_monotonic_s']<=end['receive_monotonic_s']];proof=evaluate(relevant,start['receive_monotonic_s'])
 assert all(proof['checks'].values()) and proof['identity']['run_id']==run.name
 states=[x for x in relevant if x['kind']=='gate_state' and x['data']['generation']==proof['generation'] and x['receive_monotonic_s']>=start['receive_monotonic_s']]
 assert all(all(abs(x['data']['output'][a][b])<=.001 for a in ['linear','angular'] for b in ['x','y','z']) for x in states if not x['data']['allowed'])
 fault_start=markers('EXECUTION_FAULT_BEFORE')[-1] if e['fault'] else start
 all_forbidden=[x for x in relevant if x['kind']=='gate_state' and x['receive_monotonic_s']>=fault_start['receive_monotonic_s'] and not x['data']['allowed'] and x['data']['identity']==proof['identity']]
 output=[x for x in relevant if x['kind']=='/cmd_vel' and x['receive_monotonic_s']>=fault_start['receive_monotonic_s']]
 nz=[i for i,x in enumerate(output) if any(abs(v)>.001 for k in ['linear','angular'] for v in x['data'][k])]
 zero=output[(nz[-1]+1) if nz else 0]
 metrics={'A_receive_latency_s':all_forbidden[0]['receive_monotonic_s']-fault_start['receive_monotonic_s'],'sustained_zero_receive_latency_s':zero['receive_monotonic_s']-fault_start['receive_monotonic_s'],'A_B_origin':fault_start['data']['name'],'D_origin':start['data']['name'],'stationary_window_start_sim_latency_s':proof['window']['from_s']-start['ros_now_s'],'D_proof_source_sim_latency_s':proof['window']['to_s']-start['ros_now_s']}
 od=[x for x in relevant if x['kind']=='odom'];at=[x for x in od if x['data']['stamp_s']>=start['ros_now_s']];st=[x for x in at if x['data']['stamp_s']<=proof['window']['from_s']]
 metrics['observed_stop_distance_odom_xy_m']=max((math.dist(x['data']['position'][:2],at[0]['data']['position'][:2]) for x in st),default=None)
 ticks=[x['data']['gate_steady_ms'] for x in relevant if x['kind']=='gate_state' and x['data'].get('trigger')=='tick'];gaps=sorted(b-a for a,b in zip(ticks,ticks[1:]) if b>a)
 metrics['observed_tick_gap_ms']={'count':len(gaps),'max':max(gaps),'p50':gaps[len(gaps)//2],'p95':gaps[int(.95*(len(gaps)-1))],'p99':gaps[int(.99*(len(gaps)-1))],'scope':'Gate local steady stamps received by external observer; not hard real time'}
 all_states=[x['data'] for x in relevant if x['kind']=='gate_state']
 consecutive=[b['gate_steady_ms']-a['gate_steady_ms'] for a,b in zip(all_states,all_states[1:]) if b['output_seq']==a['output_seq']+1 and a.get('trigger')==b.get('trigger')=='tick']
 metrics['observation_integrity']={'missing_output_sequences':sum(max(0,b['output_seq']-a['output_seq']-1) for a,b in zip(all_states,all_states[1:])),'max_contiguous_tick_gap_ms':max(consecutive) if consecutive else None,'meaning':'A sequence gap is missing observation; do not call it measured scheduling delay. No hard real-time guarantee.'}
 binaries={}
 for role,ref in e['binaries'].items():
  f=run/'artifacts'/(role+'.binary');assert sha(f)==ref['sha256'];binaries[role]=ref['sha256']
 world=world_window(run,proof['window']);assert world['status']!='CONTRADICTION',world
 if e['fault']:
  assert r['fault_activation']=='CONFIRMED'
  q=json.loads((run/'runtime/execution-fault-qualification.json').read_text());assert q['independent_stop_before_cancel']
  cancels=[x for x in actions if x['kind']=='cancel_response'];assert cancels and cancels[0]['monotonic_s']>=end['receive_monotonic_s']
 else:assert term[0]['data']['action']=='SUCCEEDED'
 return {'case':e['case'],'run_id':run.name,'pass':True,'source_profile':e['control_profile_version'],'goal_uuid':sent[0]['data']['uuid'],'goal_terminal':term[0]['data']['action'],'gate_cause':proof['gate_reason'],'A_B_D':proof,'C':'CONSUMER_ACK_UNKNOWN; no delivery inferred from endpoint presence','world_pose_cross_check':world,'metrics':metrics,'native_binaries':binaries,'cleanup':'PASS','record_contract':'VALID','collision':None,'goal_owner':e['goal_owner'],'lease_owner':e['lease_owner'],'actual_mission_integration':False}
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--runs',nargs='+',required=True);p.add_argument('--output',required=True);a=p.parse_args();result=[audit(Path(x)) for x in a.runs];write_json(a.output,result);print('AUDIT_PASS',len(result),'Gazebo trials')
