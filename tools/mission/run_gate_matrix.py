#!/usr/bin/env python3
"""Serial P1-03 plan runner. Delegates lifecycle and sole Goal to original Runner."""
if __package__ in (None,''):
 from pathlib import Path
 from runpy import run_path
 run_path(str(Path(__file__).resolve().parents[1]/'__init__.py'))['bootstrap']()
import argparse,json,subprocess,sys
from pathlib import Path
from tools import PROJECT_ROOT
from tools.runner.trial import load_inputs
from tools.runner.runner_core import write_json
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--plan',required=True);p.add_argument('--output-dir',required=True);p.add_argument('--cases');p.add_argument('--check',action='store_true');a=p.parse_args()
 plan=json.loads(Path(a.plan).read_text());wanted=a.cases.split(',') if a.cases else [x['case'] for x in plan];rows=[x for x in plan if x['case'] in wanted];out=Path(a.output_dir)
 if len(rows)!=len(wanted) or len(set(wanted))!=len(wanted):raise ValueError('case selection')
 if out.exists():raise ValueError('output exists; refuse overwrite')
 commands=[]
 for x in rows:
  cfg,_=load_inputs(x['scenario'],x['profile'],x['method'],x['run_id'])
  if cfg['execution_v1']['case']!=x['case']:raise ValueError('case/profile mismatch')
  if (PROJECT_ROOT/'evidence/runs'/x['run_id']).exists():raise ValueError('run exists')
  c=[sys.executable,'-B',str(PROJECT_ROOT/'tools/run_trial.py'),'--scenario',x['scenario'],'--profile',x['profile'],'--method',x['method'],'--run-id',x['run_id']]
  commands.append((x,c))
 if a.check:print(json.dumps({'check_has_no_side_effects':True,'commands':[{'case':x['case'],'argv':c} for x,c in commands]}));return 0
 out.mkdir(parents=True);write_json(out/'commands.json',[{'case':x['case'],'argv':c} for x,c in commands]);results=[]
 for x,c in commands:
  print('BEGIN',x['case'],x['run_id'],flush=True)
  with (out/(x['case']+'.log')).open('x') as f:status=subprocess.run(c,cwd=PROJECT_ROOT,stdout=f,stderr=subprocess.STDOUT).returncode
  result=PROJECT_ROOT/'evidence/runs'/x['run_id']/'runtime/execution-check.json';r=json.loads(result.read_text()) if result.exists() else {}
  entry={'case':x['case'],'run_id':x['run_id'],'exit':status,'case_pass':r.get('case_pass',False),'report':str(result)};results.append(entry);write_json(out/(x['case']+'.json'),entry);print('END',json.dumps(entry),flush=True)
  if status or not entry['case_pass']:write_json(out/'results.json',results);return 3
 write_json(out/'results.json',results);return 0
if __name__=='__main__':raise SystemExit(main())
