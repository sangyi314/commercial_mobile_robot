# SPDX-License-Identifier: MIT
import copy,json,os,sys
from pathlib import Path
import pytest
from tools.reliability.plan import policy
from tools.reliability.audit import audit_data
from tools.reliability.model import QueueModel

def fixture(tmp_path):
 root=tmp_path/'session';(root/'artifacts').mkdir(parents=True);(root/'runtime').mkdir();(root/'logs').mkdir();p=policy(300)
 (root/'artifacts/profile.json').write_text(json.dumps({'reliability_v1':{'policy':p}}))
 ids=[root.name+'_m'+str(i) for i in [1,2,3]]
 w={'steady_t0_s':1000.,'steady_end_s':1300.,'continuous_s':300.,'offers':[{'scheduled_s':x} for x in p['opportunities_s']],'submissions':[{'scheduled_s':x,'observed_s':x+.1} for x in p['submit_at_s']],'handover_source':'synthetic'}
 (root/'runtime/reliability-window.json').write_text(json.dumps(w))
 db={'records':{id:{'state':'COMPLETED','handover_source':'synthetic'} for id in ids},'unique_admitted':3,'waiting':[],'authorized_send_intents':list(range(6))}
 samples=[{'steady_s':1000.+i,'rss_bytes':1024,'cpu_percent':20.,'free_bytes':100*1024**3,'records':db['records'],'bag_bytes':1000+i,'bag_last_progress_age_s':0.,'gate':{'state':{'ros_ns':i*1000000000}}} for i in range(301)]
 (root/'runtime/reliability-samples.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in samples));(root/'logs/events.jsonl').write_text('')
 world=[]
 for i in range(181):
  t=round((9.8+i*.02)*1e9);world.append({'kind':'pose','run_id':root.name,'clock_epoch':root.name+':sim0','seq':i,'protobuf':{'header':{'stamp':{'sec':t//1000000000,'nsec':t%1000000000}},'pose':[{'name':'turtlebot3_waffle','position':{'x':0,'y':0},'orientation':{'w':1}}]}})
 (root/'runtime/p0-08-gazebo.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in world));(root/'runtime/bag-check.json').write_text(json.dumps({'stops':[{'from_s':10.,'to_s':13.2,'attempt':'fixture_attempt'}]}))
 return root,p,w,db,samples

def test_independent_audit_accepts_complete_window(tmp_path):
 root,p,w,db,s=fixture(tmp_path);r=audit_data(root,db,{});assert r['valid'] and r['counts']['COMPLETED']==3 and r['rtf']==1

@pytest.mark.parametrize('fault',['short','gap','rollback','missing_order','waiting','fake_human','over_rss','frozen_clock','excess_goal','submission_drift','missing_sample_file'])
def test_independent_audit_rejects_bad_raw_evidence(tmp_path,fault):
 root,p,w,db,s=fixture(tmp_path)
 if fault=='short':w['steady_end_s']-=1;w['continuous_s']-=1
 elif fault=='gap':del s[80:85]
 elif fault=='rollback':s[90]['steady_s']=s[88]['steady_s']
 elif fault=='missing_order':db['records'].pop(next(iter(db['records'])))
 elif fault=='waiting':db['waiting']=['new']
 elif fault=='fake_human':w['handover_source']='human'
 elif fault=='over_rss':s[90]['rss_bytes']=p['rss_limit_bytes']+1
 elif fault=='frozen_clock':
  for row in s:row['gate']['state']['ros_ns']=0
 elif fault=='excess_goal':db['authorized_send_intents']=list(range(61))
 elif fault=='submission_drift':w['submissions'][0]['observed_s']=20
 elif fault=='missing_sample_file':
  (root/'runtime/reliability-samples.jsonl').unlink()
  assert not audit_data(root,db,{})['valid']
  return
 (root/'runtime/reliability-window.json').write_text(json.dumps(w));(root/'runtime/reliability-samples.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in s))
 assert not audit_data(root,db,{})['valid'],fault

def test_frozen_capacity_budget_and_distribution():
 p=policy(3600);assert len(p['submit_at_s'])==20 and 3*len(p['submit_at_s'])<=p['gate_begin_budget']<64
 assert all(any(i*1200<=x<(i+1)*1200 for x in p['submit_at_s']) for i in range(3))
 with pytest.raises(ValueError):policy(3599)

def test_reference_model_fifo_and_duplicate_priority():
 m=QueueModel();assert m.receive(0,'m')=='ACCEPTED'
 for n in range(4):assert m.receive(0,str(n))=='QUEUED'
 assert m.receive(0,'m')=='EXISTING' and m.receive(0,'extra')=='QUEUE_FULL'
 assert m.receive(2,'0')=='QUEUE_CANCELED';assert m.receive(2,'m')=='CANCEL_PENDING'
 assert m.receive(3,'')=='SCHEDULE_PAUSED' and m.active is None

def test_cli_check_does_not_create_run_or_spawn(tmp_path,monkeypatch,capsys):
 from tools.mission import run_reliability as cli
 from tools.runner import trial
 from tools.reliability.candidate import sha
 candidate=tmp_path/'candidate.json';candidate.write_text(json.dumps({'source_id':cli.source_id([]),'candidate_id':cli.candidate_identity(cli.source_id([]),[],[]),'files':[],'binaries':[]}))
 monkeypatch.setattr(cli,'runtime_manifest',lambda:[])
 good=tmp_path/'audit.json';good.write_text('{"valid":true}')
 profile={'reliability_v1':{'policy':policy(300),'candidate':{'path':str(candidate),'sha256':sha(candidate)},'prerequisites':{k:{'path':str(good),'sha256':sha(good)} for k in ['delegated_admission','core_regression']}}}
 monkeypatch.setattr(trial,'load_inputs',lambda *a:(profile,{}));monkeypatch.setattr(trial,'main',lambda:0)
 monkeypatch.setattr(sys,'argv',['run_reliability.py','run','--profile','unused','--scenario','unused','--run-id','check','--check'])
 before=sorted(tmp_path.iterdir());assert cli.main()==0;assert sorted(tmp_path.iterdir())==before
 profile['reliability_v1']['prerequisites'].pop('delegated_admission');assert cli.main()==2

def test_window_filter_has_no_pass_selection():
 from tools.delivery.inspect_delivery_bag import within_windows
 assert within_windows(10,[(9,11)]) and not within_windows(12,[(9,11)])


def test_resource_sampler_checks_pid_birth(tmp_path):
 from types import SimpleNamespace
 from tools.reliability.resources import Resources
 from tools.runner.runner_processes import identity
 info=identity(os.getpid());owner=SimpleNamespace(owned=[SimpleNamespace(closed=False)],members=lambda item:[info])
 r=Resources(owner,tmp_path).sample();assert r['processes'][0]['pid']==os.getpid() and r['rss_bytes']>0
 info['start_ticks']+=1
 with pytest.raises(RuntimeError,match='birth'):Resources(owner,tmp_path).sample()


def test_runtime_inventory_is_pure_filesystem(monkeypatch):
 import subprocess
 from tools.reliability.candidate import runtime_manifest
 monkeypatch.setattr(subprocess,'Popen',lambda *a,**k:pytest.fail('inventory spawned a process'))
 rows=runtime_manifest();assert any(r['path']=='tools/mission/run_reliability.py' for r in rows)


def test_long_recording_is_unbuffered_without_changing_legacy(tmp_path):
 from tools.reliability.plan import recording_args
 assert recording_args({},tmp_path)==[]
 args=recording_args({'reliability_v1':{}},tmp_path)
 # Empty dict is false, just as an absent long-session configuration.
 assert args==[]
 args=recording_args({'reliability_v1':{'version':1}},tmp_path)
 assert args[:2]==['--max-cache-size','0'] and args[-1]==str(tmp_path/'artifacts/reliability-mcap.yaml')


def test_recording_progress_uses_bytes_and_steady_clock(tmp_path):
 from tools.reliability.resources import RecordingProgress
 (tmp_path/'bag').mkdir();p=tmp_path/'bag/data.mcap';p.write_bytes(b'header')
 r=RecordingProgress(tmp_path,3,now=10)
 with pytest.raises(RuntimeError,match='progress'):r.sample(now=11,require_advance=True)
 p.write_bytes(b'header data');assert r.sample(now=12,require_advance=True)['bag_bytes']==11
 assert r.sample(now=15)['bag_last_progress_age_s']==3
 with pytest.raises(RuntimeError,match='progress'):r.sample(now=15.001)
 p.write_bytes(b'');
 with pytest.raises(RuntimeError,match='truncated'):r.sample(now=16)


def test_world_oracle_must_not_drop_a_short_moving_prefix(tmp_path):
 from tools.reliability.world import check_windows
 root,p,w,db,s=fixture(tmp_path);path=root/'runtime/p0-08-gazebo.jsonl';rows=[json.loads(x) for x in path.read_text().splitlines()]
 for row in rows:
  t=row['protobuf']['header']['stamp'];ns=t['sec']*1000000000+t['nsec']
  if ns>=10020000000:row['protobuf']['pose'][0]['position']['x']=.0004
 path.write_text(''.join(json.dumps(x)+'\n' for x in rows))
 r=check_windows(root,[{'from_s':10.,'to_s':13.2,'attempt':'a'}]);assert not r['valid'] and r['windows'][0]['max_v']>.01


def test_missing_recording_samples_is_invalid_not_a_pass(tmp_path):
 root,p,w,db,s=fixture(tmp_path);(root/'runtime/reliability-samples.jsonl').unlink()
 r=audit_data(root,db,{})
 assert not r['valid'] and 'sample log missing' in r['errors']
