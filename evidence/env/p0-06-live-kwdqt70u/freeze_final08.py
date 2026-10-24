"""Freeze two inputs only after separately qualified closed real probes."""
from pathlib import Path
import json,sys,copy,tarfile,hashlib
ROOT=Path('/home/robot/commercial_mobile_robot');sys.path.insert(0,str(ROOT/'tools'))
import validate_run as v
from dynamic_contract import fingerprint
DEV=ROOT/'evidence/env/p0-06-live-kwdqt70u'
def h(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def ref(p):return {'path':str(p),'sha256':h(p)}
def write(p,j):Path(p).write_text(json.dumps(j,ensure_ascii=False,indent=2)+'\n')

contrast=json.loads((DEV/'probe-load-contrast-final08.json').read_text());assert all(p['differentiated'] for p in contrast['pairs'])
backup=DEV/'before-formal-freeze-final08';backup.mkdir()
targets=['experiments/scenarios/crossing.yaml','experiments/scenarios/observed_occlusion.yaml','experiments/profiles/p0_06_crossing.yaml','experiments/profiles/p0_06_observed_occlusion.yaml']
with tarfile.open(backup/'drafts.tgz','x:gz') as t:
 for p in targets:t.add(ROOT/p,arcname=p)
summaries=[]
for kind,prep,names in [('crossing','crossing-final08-preparation',['p0-06-crossing-probe-kwdqt70u-final08-low','p0-06-crossing-probe-kwdqt70u-final08-higher']),('observed_occlusion','observed_occlusion-final08-preparation',['p0-06-occlusion-probe-kwdqt70u-final08-low','p0-06-occlusion-probe-kwdqt70u-final08-higher'])]:
 spec=v.load_strict(DEV/prep/'probe.yaml');profile=v.load_strict(DEV/prep/'profile.yaml');fp=fingerprint(spec);reports=[];evidence=[];servers=[];epochs=[]
 for name in names:
  run=ROOT/'evidence/runs'/name
  assert v.validate_run_dir(run)['status']=='VALID'
  m=v.load_strict(run/'manifest.json');r=v.load_strict(run/'result.json');e=v.load_strict(run/'runtime/runner-exit.json');c=v.load_strict(run/'runtime/cleanup.json');d=v.load_strict(run/'runtime/dynamic-report.json');ready=v.load_strict(run/'runtime/dynamic-readiness.json');rp=v.load_strict(run/'artifacts/profile.json')
  assert m['record_origin']=='real' and r['status']['startup']=='READY' and r['status']['action']=='NOT_SENT' and r['status']['harness']=='COMPLETED' and r['status']['cleanup']=='SUCCEEDED'
  assert e['probe_pass'] and e['bag_valid'] and c['ok'] and not c['remaining'] and all(not p['forced'] for p in c['processes'])
  assert fingerprint(rp['dynamic_spec'])==fp and d['probe_pass'] and d['motion_valid'] and d['time_valid'] and d['observation_valid'] and not d['errors'] and d['collision'] is None
  assert all(x is True for x in d['probe_facilities']['checks'].values()) and ready['input_isolation']['truth_leak_detected'] is False
  assert not (run/'logs/action-client.jsonl').read_text()
  servers.append(next(p['identity']['pid'] for p in c['processes'] if p['role']=='gazebo'));epochs.append(m['clock']['clock_epoch']);reports.append(d)
  files=['manifest.json','result.json','runtime/runner-exit.json','runtime/dynamic-report.json','runtime/dynamic-readiness.json','runtime/cleanup.json','runtime/gazebo-raw.jsonl','runtime/motion-commands.jsonl','runtime/observations.jsonl','logs/events.jsonl','bag/metadata.yaml']
  evidence += [ref(run/p) for p in files]+[ref(p) for p in sorted((run/'bag').glob('*.mcap'))]
 assert len(set(servers))==2 and len(set(epochs))==2
 evidence += [ref(DEV/'independent-reviews'/(n+'.json')) for n in names]
 key='s1' if kind=='crossing' else 's3'
 evidence += [ref(DEV/('load-'+key+'-final08-'+condition+'.json')) for condition in ['low','higher']]
 evidence += [ref(DEV/'probe-load-contrast-final08.json'),ref(DEV/'world-map-landmark-review.json'),ref(DEV/'calibration-01.json'),ref(DEV/'load-contrast-criteria.json')]
 hostroot=DEV/'host-evidence-09'
 for sub in (['crossing-final08-probe-low','crossing-final08-probe-higher'] if kind=='crossing' else ['occlusion-final08-probe-low','occlusion-final08-probe-higher']):
  evidence += [ref(p) for p in sorted((hostroot/sub).iterdir()) if p.is_file()]
 proof={'schema_version':1,'record_origin':'real','scope':'closed_real_probe_and_load_review','input_fingerprint':fp,'checks':{k:True for k in ['visual','collision','scan','time_alignment','pause','cold_start','truth_isolation']},'manifest':ref(ROOT/'evidence/runs'/names[0]/'manifest.json'),'probe_runs':[str(ROOT/'evidence/runs'/n) for n in names],'gazebo_pids':servers,'clock_epochs':epochs,'evidence':evidence,'visual_scope':'actual Scene visual geometry and GPU lidar returns; personal visual observation pending','collision_scope':'two-location cylinder contact and actual ball response only; navigation collision=null','reviewer':'Codex engineering review under explicit user continuation authorization','personal_visual_confirmation':None,'personal_understanding_confirmation':None,'map_scope':'nominal world-map identity with measured raster landmark offsets; no AMCL-as-truth and no exact pixel equivalence'}
 proofpath=DEV/('ready-proof-'+kind+'-final08.json');assert not proofpath.exists();write(proofpath,proof)
 formal=copy.deepcopy(spec);formal['scenario_id']=kind;formal['scenario_revision']='p0-06-frozen-'+key+'-final08-kwdqt70u';formal['status']='ready_for_recording';formal['dynamic']['purpose']='navigation_recording';formal['dynamic']['trigger']='goal_accepted';formal['dynamic']['preparation']['probe']=ref(proofpath)
 for name in ['resource_dependencies','world_map_alignment','poses_free','interfaces_observed']:formal['verification'][name]='checked'
 formal['verification']['evidence_refs']=[str(proofpath),str(DEV/'world-map-landmark-review.json'),str(DEV/'calibration-01.json')]
 assert fingerprint(formal)==fp
 scenario=ROOT/'experiments/scenarios'/(kind+'.yaml');write(scenario,formal)
 profile['mode']='navigation_recording';profile['scenario_ref']=ref(scenario);profilepath=ROOT/'experiments/profiles'/('p0_06_'+kind+'.yaml');write(profilepath,profile)
 diagnostic=v.validate_scenario(scenario,True);assert diagnostic['status']=='VALID',diagnostic
 summaries.append({'kind':kind,'scenario':ref(scenario),'profile':ref(profilepath),'preparation_proof':ref(proofpath),'input_fingerprint':fp,'validator':diagnostic})
(DEV/'formal-input-freeze-final08.json').open('x').write(json.dumps({'scenarios':summaries,'personal_confirmation':None,'navigation_collision':None},indent=2)+'\n')
print(json.dumps(summaries,ensure_ascii=False))
