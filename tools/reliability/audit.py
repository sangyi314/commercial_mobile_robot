# SPDX-License-Identifier: MIT
"""Independent timing/accounting checks; raw DB/MCAP audit remains mandatory."""
import json,math
from pathlib import Path

def audit_data(root,db,bag):
 root=Path(root);p=json.loads((root/'artifacts/profile.json').read_text())['reliability_v1']['policy'];w=json.loads((root/'runtime/reliability-window.json').read_text());samples=[json.loads(x) for x in (root/'runtime/reliability-samples.jsonl').read_text().splitlines()];errors=[]
 def need(b,s):
  if not b:errors.append(s)
 times=[r['steady_s'] for r in samples];need(len(times)>=p['duration_s']/p['max_sample_gap_s'],'sampling coverage count');need(all(math.isfinite(t) for t in times),'nonfinite sample time')
 need(bool(times) and 0<=times[0]-w['steady_t0_s']<=p['max_sample_gap_s'],'first sample coverage')
 need(bool(times) and times[-1]-w['steady_t0_s']>=p['duration_s']-p['max_sample_gap_s'],'last sample coverage')
 need(all(0<b-a<=p['max_sample_gap_s'] for a,b in zip(times,times[1:])),'sample gap or rollback')
 need(w['steady_end_s']-w['steady_t0_s']>=p['duration_s'] and w['continuous_s']==w['steady_end_s']-w['steady_t0_s'],'continuous wall duration')
 need(w['continuous_s']<=p['duration_s']+p['drain_s'],'drain budget')
 expected={root.name+'_m'+str(i+1) for i in range(len(p['submit_at_s']))};need(set(db['records'])==expected and db['unique_admitted']==len(expected),'admitted order plan/accounting')
 counts={s:sum(r['state']==s for r in db['records'].values()) for s in ['COMPLETED','FAILED','CANCELED']};active=sum(r['state'] not in counts for r in db['records'].values());waiting=len(db['waiting'])
 need(db['unique_admitted']==sum(counts.values())+active+waiting,'order conservation');need(counts['COMPLETED']>=p['min_completed'] and counts['FAILED']==counts['CANCELED']==active==waiting==0,'normal workload result')
 need(len(w['offers'])==len(p['opportunities_s']) and [x['scheduled_s'] for x in w['offers']]==p['opportunities_s'],'opportunity plan')
 need(len(w['submissions'])==len(p['submit_at_s']) and [x['scheduled_s'] for x in w['submissions']]==p['submit_at_s'],'submission plan')
 need(all(x['observed_s']>=x['scheduled_s'] and x['observed_s']-x['scheduled_s']<=p['max_sample_gap_s']+8 for x in w['submissions']),'submission schedule drift')
 need(all(r['handover_source']=='synthetic' for r in db['records'].values()) and w['handover_source']=='synthetic','handover provenance')
 for s in samples:
  need(s['rss_bytes']<=p['rss_limit_bytes'] and s['cpu_percent']<=p['cpu_limit_percent'] and s['free_bytes']>=p['min_free_bytes'],'resource budget')
  need(sum(bool(r['state']) and r['state'] not in ['COMPLETED','FAILED','CANCELED'] for r in s['records'].values())<=1,'sample active slot')
 # Completion distribution derives from actual logged owner query replies/markers,
 # rather than a Runner PASS flag. These markers carry committed record state.
 events=[json.loads(x) for x in (root/'logs/events.jsonl').read_text().splitlines()];finishes={}
 for e in events:
  if e['type']=='DELIVERY_API':
   q=e['data']['response']
   if q['state']=='COMPLETED':finishes.setdefault(q['mission_id'],e['parent_monotonic_s']-w['steady_t0_s'])
 if p['duration_s']==3600:
  need(all(any(i*1200<=t<(i+1)*1200 for t in finishes.values()) for i in range(3)),'completion in each third')
 # Source sim coverage is taken from independently recorded samples, never
 # substituted for steady duration. Unknown RTF remains null.
 source=[s['gate']['state']['ros_ns']*1e-9 for s in samples];sim_s=source[-1]-source[0] if source else None
 need(bool(source) and all(b>=a for a,b in zip(source,source[1:])) and sim_s>0,'advancing sim time')
 need(len(db['authorized_send_intents'])<=p['gate_begin_budget'],'Gate lifetime request budget')
 return {'valid':not errors,'errors':errors,'continuous_wall_s':w['continuous_s'],'sim_s':sim_s,'rtf':sim_s/(times[-1]-times[0]) if sim_s and len(times)>1 else None,'counts':counts,'active':active,'waiting':waiting,'unique_admitted':db['unique_admitted'],'max_rss_bytes':max((s['rss_bytes'] for s in samples),default=None),'max_cpu_percent':max((s['cpu_percent'] for s in samples),default=None),'collision':None,'oracle_stop_assisted':True,'sensor_only_E2':False}
