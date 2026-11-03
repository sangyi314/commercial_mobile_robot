# SPDX-License-Identifier: MIT
"""Timed business driver delegated to the existing Runner; no process/Goal creation."""
import json,time
from pathlib import Path
from tools.delivery.session import DeliverySession
from tools.delivery.records import decode_record
from tools.reliability.resources import Resources,RecordingProgress
from tools.runner.runner_core import write_json
class ReliabilitySession(DeliverySession):
 def execute(self,work,gz):
  self.progress=None
  try:return self._execute(work,gz)
  except BaseException as exc:
   path=self.run/'runtime/reliability-window.json'
   if not path.exists():
    now=time.monotonic();p=self.progress;started=p is not None;t0=p['t0'] if started else None
    write_json(path,{'version':1,'started':started,'aborted':True,'error':str(exc),'steady_t0_s':t0,'steady_end_s':now if started else None,'continuous_s':now-t0 if started else None,'scheduled_duration_s':self.p['reliability_v1']['policy']['duration_s'],'offers':p['offers'] if started else [],'submissions':p['submissions'] if started else [],'completed':sorted(p['completed']) if started else [],'handover_source':'synthetic','oracle_stop_assisted':True})
   raise
 def _execute(self,work,gz):
  policy=self.p['reliability_v1']['policy'];self.ids=[];recording=RecordingProgress(self.run,policy['max_record_gap_s']);self.proof('RELIABILITY_INITIAL_IDLE',work);recording.sample(require_advance=True)
  resources=Resources(self.supervisor,self.run);t0=time.monotonic();end=t0+policy['duration_s'];drain=end+policy['drain_s'];next_sample=t0;submit_index=0;offer_index=0;completed=set();wait_since={};offers=[];submissions=[];samples=[];first=True
  self.progress={'t0':t0,'offers':offers,'submissions':submissions,'completed':completed}
  self.mark('RELIABILITY_READY',{'steady_s':t0,'duration_s':policy['duration_s'],'handover_source':'synthetic'},work)
  path=self.run/'runtime/reliability-samples.jsonl'
  with path.open('x') as stream:
   while True:
    now=time.monotonic();work.require();elapsed=now-t0
    while offer_index<len(policy['opportunities_s']) and elapsed>=policy['opportunities_s'][offer_index]:
     offers.append({'index':offer_index,'scheduled_s':policy['opportunities_s'][offer_index],'observed_s':elapsed,'decision':'SCHEDULED_ORDER' if policy['opportunities_s'][offer_index] in policy['submit_at_s'] else 'FROZEN_SPARSE_LOAD_NO_NEW_ORDER'})
     if self.ids and (offer_index+1)%policy['duplicate_every']==0:
      q=self.command('submit',self.ids[0],work,schema_version=1,delivery_station='delivery',return_station='home',map_version=self.catalog['map_version'],station_version=self.catalog['station_version'],created_at_unix_ms=1)
      if q['code']!='EXISTING':raise RuntimeError('duplicate order not EXISTING')
     offer_index+=1
    while submit_index<len(policy['submit_at_s']) and elapsed>=policy['submit_at_s'][submit_index] and now<end:
     id=self.run.name+'_m'+str(submit_index+1);q=self.submit(id,work);self.ids.append(id);submissions.append({'mission_id':id,'scheduled_s':policy['submit_at_s'][submit_index],'observed_s':time.monotonic()-t0,'response':q['code']});submit_index+=1
    if now>=next_sample:
     sample=resources.sample();sample['elapsed_s']=sample['steady_s']-t0;records={}
     for id in self.ids:
      q=self.statuses[id] if id in completed else self.query(id,work);records[id]=q
      if q['state'] in ['FAILED','CANCELED','PAUSED']:raise RuntimeError('normal reliability workload entered '+q['state'])
      if q['state']=='WAIT_HANDOVER' and not q['handed_over']:
       if time.monotonic()-wait_since.setdefault(id,time.monotonic())>=policy['handover_delay_s']:self.handover(id,q,work)
      if q['state']=='COMPLETED' and id not in completed:
       completed.add(id);self.mark('DELIVERY_COMPLETED',{'mission_id':id,'record':q,'elapsed_s':time.monotonic()-t0},work)
     active=[q for q in records.values() if q['state'] and q['state'] not in ['COMPLETED','FAILED','CANCELED']]
     # WAITING has an empty business state, not a second active Mission.
     unresolved=[decode_record(json.loads(q['detail'])['record']) for q in active if json.loads(q['detail']).get('record')]
     if len(active)>1 or sum(r['ever_sent'] and not r['goal_terminal'] for r in unresolved)>1:raise RuntimeError('I1/I2 violated')
     gate=self.link.rpc('execution_status',{},work.child(3));sample.update(records=records,gate=gate,completed=len(completed))
     if gate['state']['state']=='FAULT_LATCHED':raise RuntimeError('Gate fault latched')
     if sample['rss_bytes']>policy['rss_limit_bytes'] or sample['cpu_percent']>policy['cpu_limit_percent'] or sample['free_bytes']<policy['min_free_bytes']:raise RuntimeError('frozen resource limit exceeded')
     if samples and sample['steady_s']-samples[-1]['steady_s']>policy['max_sample_gap_s']:raise RuntimeError('reliability sampling gap')
     sample.update(recording.sample())
     first=False;stream.write(json.dumps(sample,allow_nan=False)+'\n');stream.flush();samples.append(sample);next_sample=sample['steady_s']+policy['sample_period_s']
    if now>=end:
     if len(completed)==len(self.ids) and submit_index==len(policy['submit_at_s']):break
     if now>=drain:raise TimeoutError('reliability drain budget')
    self.link.pump()
  measured=time.monotonic()-t0
  self.mark('RELIABILITY_WINDOW_END',{'steady_s':t0+measured,'continuous_s':measured},work)
  if len(completed)<policy['min_completed']:raise RuntimeError('insufficient completed deliveries')
  self.proof('RELIABILITY_FINAL_IDLE',work);self.case_done=True
  result={'version':1,'steady_t0_s':t0,'steady_end_s':t0+measured,'continuous_s':measured,'scheduled_duration_s':policy['duration_s'],'offers':offers,'submissions':submissions,'completed':sorted(completed),'sample_count':len(samples),'handover_source':'synthetic','oracle_stop_assisted':True,'sensor_only_E2':False,'collision':None}
  write_json(self.run/'runtime/reliability-window.json',result);write_json(self.run/'runtime/delivery-stop-proofs.json',self.proofs)
  write_json(self.run/'runtime/delivery-case.json',{'case':self.d['case'],'expected_business_result':True,'fault_activation':'NOT_APPLICABLE','synthetic_observations':False,'handover_policy':'synthetic','oracle_stop_assisted':True})
