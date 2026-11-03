# SPDX-License-Identifier: MIT
"""Pure reliability input validation; check does not create paths or initialize ROS."""
import copy,json,math,yaml
from pathlib import Path
from tools.reliability.candidate import sha

def policy(duration):
 if duration not in [300,600,3600]:raise ValueError('reviewed duration must be 300,600,3600')
 # Gate retains 64 Begin decisions. 20 admitted orders reserve 60 attempts.
 slots=([0,1,180] if duration==300 else ([0,1,180,360] if duration==600 else [0,1]+list(range(360,3421,180))))
 return {'version':1,'duration_s':duration,'opportunities_s':list(range(0,duration,60)),'submit_at_s':slots,'duplicate_every':5,'handover_delay_s':.2,'handover_source':'synthetic','sample_period_s':1.,'max_sample_gap_s':3.,'max_record_gap_s':3.,'drain_s':180,'min_completed':3 if duration==300 else (4 if duration==600 else 10),'rss_limit_bytes':6*1024**3,'cpu_limit_percent':700.,'min_free_bytes':8*1024**3,'faults':[],'gate_begin_budget':60,'oracle_stop_assisted':True,'sensor_only_E2':False}
def validate(scene,p,method,legacy,identity):
 r=p['reliability_v1']
 if set(r)!={'version','base_profile','base_scenario','candidate','policy','prerequisites','storage'} or r['version']!=1:raise ValueError('reliability schema')
 for ref in [r['base_profile'],r['base_scenario'],r['candidate'],r['storage'],*r['prerequisites'].values()]:
  if sha(ref['path'])!=ref['sha256']:raise ValueError('reliability input hash '+ref['path'])
 if yaml.safe_load(Path(r['storage']['path']).read_text())!={'noChunking':True,'enableDataCRC':True}:raise ValueError('unreviewed MCAP storage options')
 duration=r['policy']['duration_s']
 if r['policy']!=policy(duration):raise ValueError('unreviewed reliability policy')
 base,oldscene=legacy(r['base_scenario']['path'],r['base_profile']['path'],method,identity)
 if base['delivery_v1']['case']!='G05' or method!='mppi':raise ValueError('long session requires real Mission, synthetic FIFO, Shim+MPPI')
 expected=copy.deepcopy(base);expected['profile_version']=5;expected['reliability_v1']=r;expected['budgets']['work']=duration+380
 expected['budgets']['finalize']=180
 if p!=expected or scene!=oldscene:raise ValueError('reliability delta beyond reviewed budgets')
 candidate=json.loads(Path(r['candidate']['path']).read_text())
 if len(candidate['candidate_id'])!=64 or not candidate.get('files'):raise ValueError('candidate identity absent')
 return p,scene

def prepare(base_profile,base_scenario,candidate,output,prerequisites,duration):
 root=Path(output).resolve();root.mkdir(parents=True,exist_ok=False)
 ref=lambda p:{'path':str(Path(p).resolve()),'sha256':sha(p)}
 storage=root/'mcap.yaml';storage.write_text(json.dumps({'noChunking':True,'enableDataCRC':True}))
 p=json.loads(Path(base_profile).read_text());p['profile_version']=5;p['reliability_v1']={'version':1,'base_profile':ref(base_profile),'base_scenario':ref(base_scenario),'candidate':ref(candidate),'policy':policy(duration),'prerequisites':{k:ref(v) for k,v in prerequisites.items()},'storage':ref(storage)};p['budgets']['work']=duration+380;p['budgets']['finalize']=180
 (root/'profile.json').write_text(json.dumps(p,indent=2));(root/'scenario.json').write_bytes(Path(base_scenario).read_bytes());return p


def recording_args(profile,run):
 if not profile.get('reliability_v1'):return []
 return ['--max-cache-size','0','--storage-config-file',str(Path(run)/'artifacts/reliability-mcap.yaml')]
