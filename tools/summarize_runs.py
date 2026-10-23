"""Read-only index of every matching run directory; never repairs records."""
from pathlib import Path
import argparse,json,sys
from runner_core import write_json,digest,strict_json
import validate_run

def summarize(root,prefix):
 rows=[];classes={'real':0,'fixture':0,'incomplete_or_missing':0,'invalid':0,'validator_error':0};successes=0;normal=0;controlled=0
 for path in sorted(Path(root).iterdir()):
  if not path.is_dir() or not path.name.startswith(prefix):continue
  row={'run_id':path.name,'path':str(path.resolve()),'method':None,'origin':None,'status':None,'behavior_verified':False,'normal_pass':False}
  if not (path/'manifest.json').exists():row['contract']='MISSING_MANIFEST';group='incomplete_or_missing'
  else:
   try:
    m=strict_json((path/'manifest.json').read_text());row['origin']=m.get('record_origin');row['method']=m.get('baseline',{}).get('method')
    d=validate_run.validate_run_dir(path,allow_fixtures=row['origin']=='fixture');row['contract']=d['status'];row['diagnostic']=d
    if row['origin']=='fixture':group='fixture'
    elif d['status']=='INCOMPLETE':group='incomplete_or_missing'
    elif d['status']=='VALID':group='real'
    else:group='invalid'
    if d['status']=='VALID':
     r=strict_json((path/'result.json').read_text());row['status']=r['status'];row['first_reason']=r['status']['primary_reason']
     bag=path/'runtime/bag-check.json';exitfile=path/'runtime/runner-exit.json';behavior=path/'runtime/behavior.json';faultfile=path/'artifacts/profile.json'
     if bag.exists():row['bag_check']={'path':str(bag),'sha256':digest(bag),'value':strict_json(bag.read_text())}
     if behavior.exists():row['behavior_verified']=all(strict_json(behavior.read_text()).get('checks',{}).values()) and bool(strict_json(behavior.read_text()).get('checks'))
     if exitfile.exists():row['normal_pass']=strict_json(exitfile.read_text()).get('normal_pass') is True and row['behavior_verified'] and row.get('bag_check',{}).get('value',{}).get('valid') is True and r['status']['cleanup']=='SUCCEEDED' and r['status']['startup']=='READY' and r['status']['action']=='SUCCEEDED' and r['status']['harness']=='COMPLETED'
     fault=strict_json(faultfile.read_text()).get('fault') if faultfile.exists() else None
     first_failure=strict_json((path/'runtime/failures.json').read_text()).get('first',{}) if (path/'runtime/failures.json').exists() else {}
     row['controlled_fault_case_pass']=bool(first_failure and first_failure.get('code')=='CONTROLLED_INITIALPOSE_TIMEOUT' and r['status']['harness']=='STARTUP_FAILED' and fault and r['status']['startup']=='FAILED' and r['status']['action']=='NOT_SENT' and r['status']['cleanup']=='SUCCEEDED' and row.get('bag_check',{}).get('value',{}).get('valid') and 'FAULT_ACTIVATED' in row['bag_check']['value'].get('events',[]))
     if group=='real':successes+=r['status']['action']=='SUCCEEDED';normal+=row['normal_pass'];controlled+=row['controlled_fault_case_pass']
   except (ValueError,KeyError,OSError) as exc:group='invalid';row['contract']='INVALID';row['error']=str(exc)
   except Exception as exc:group='validator_error';row['contract']='VALIDATOR_ERROR';row['error']=str(exc)
  classes[group]+=1;row['classification']=group;rows.append(row)
 assert sum(classes.values())==len(rows)
 return {'total':len(rows),'classes':classes,'successful_actions_real':successes,'complete_normal_runs_real':normal,'controlled_failure_cases_passed_real':controlled,'runs':rows,'note':'VALID alone does not establish navigation behavior. No performance ranking or collision inference.'}
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',required=True);p.add_argument('--prefix',required=True);p.add_argument('--output',required=True);a=p.parse_args();r=summarize(a.root,a.prefix);write_json(a.output,r);print(json.dumps({k:v for k,v in r.items() if k!='runs'}));return 0
if __name__=='__main__':sys.exit(main())
