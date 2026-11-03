#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Freeze, prepare, check, run or audit reliability using the original CMR Runner."""
if __package__ in (None,''):
 from pathlib import Path
 from runpy import run_path
 run_path(str(Path(__file__).resolve().parents[1]/'__init__.py'))['bootstrap']()
import argparse,json,sys
from pathlib import Path
from tools.reliability.candidate import freeze,sha,runtime_manifest,source_id,candidate_identity

def main():
 p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='operation',required=True)
 f=sub.add_parser('freeze');f.add_argument('--output-dir',required=True);f.add_argument('--install-prefix',required=True)
 b=sub.add_parser('prepare');b.add_argument('--base-profile',required=True);b.add_argument('--base-scenario',required=True);b.add_argument('--candidate',required=True);b.add_argument('--output-dir',required=True);b.add_argument('--duration',type=int,choices=[300,600,3600],required=True);b.add_argument('--prerequisites',required=True,help='JSON object: evidence name to actual completed audit/report path')
 r=sub.add_parser('run');r.add_argument('--profile',required=True);r.add_argument('--scenario',required=True);r.add_argument('--run-id',required=True);r.add_argument('--check',action='store_true');r.add_argument('--supervision-record')
 a=sub.add_parser('audit');a.add_argument('--session',required=True);a.add_argument('--output',required=True)
 args=p.parse_args()
 if args.operation=='freeze':print(json.dumps({'candidate_id':freeze(args.output_dir,args.install_prefix)['candidate_id']}));return 0
 if args.operation=='prepare':
  from tools.reliability.plan import prepare
  prepare(args.base_profile,args.base_scenario,args.candidate,args.output_dir,json.loads(Path(args.prerequisites).read_text()),args.duration);print('PREPARED');return 0
 if args.operation=='audit':
  from tools.delivery.records import audit_session
  from tools.reliability.audit import audit_data
  out=Path(args.output)
  if out.exists():p.error('refuse overwrite')
  try:
   result=audit_session(args.session);result['reliability']=audit_data(args.session,result['database'],json.loads((Path(args.session)/'runtime/bag-check.json').read_text()))
  except Exception as ex:result={'valid':False,'session_pass':False,'errors':[type(ex).__name__+': '+str(ex)]}
  out.open('x').write(json.dumps(result,indent=2));print('AUDIT_PASS' if result['session_pass'] else 'AUDIT_FAIL');return 0 if result['session_pass'] else 3
 from tools.runner import trial
 try:
  profile,scene=trial.load_inputs(args.scenario,args.profile,'mppi',args.run_id)
  if not profile.get('reliability_v1'):raise ValueError('explicit reliability profile required')
  refs=profile['reliability_v1'];candidate=json.loads(Path(refs['candidate']['path']).read_text())
  from tools import PROJECT_ROOT
  if candidate['source_id']!=source_id(candidate['files']) or candidate['candidate_id']!=candidate_identity(candidate['source_id'],candidate['binaries'],candidate.get('external_files',[])):raise ValueError('candidate manifest identity')
  runtime=lambda rows:{r['path']:r['sha256'] for r in rows if r['path'].startswith(('src/','tools/','tests/'))}
  if runtime(candidate['files'])!=runtime(runtime_manifest()):raise ValueError('candidate runtime source set changed')
  for row in candidate['files']:
   if row['path'].startswith(('src/','tools/','tests/')) and sha(PROJECT_ROOT/row['path'])!=row['sha256']:raise ValueError('candidate runtime source changed '+row['path'])
  for ref in candidate['binaries']+candidate.get('external_files',[]):
   if sha(ref['path'])!=ref['sha256']:raise ValueError('candidate binary changed')
  for name,ref in refs['prerequisites'].items():
   evidence=json.loads(Path(ref['path']).read_text())
   if evidence.get('valid') is not True and evidence.get('passed') is not True:raise ValueError('prerequisite not passed '+name)
  if not {'delegated_admission','core_regression'}.issubset(refs['prerequisites']):raise ValueError('missing qualified admission/core regression')
  if refs['policy']['duration_s']==3600:
   if 'short_qualification' not in refs['prerequisites']:raise ValueError('short qualification missing')
   if not args.check:
    if not args.supervision_record:raise ValueError('actual supervision arrangement required')
    supervision=json.loads(Path(args.supervision_record).read_text())
    if supervision.get('user_supervision_available') is not True or not supervision.get('user_statement'):raise ValueError('supervision is unconfirmed')
 except Exception as ex:print(json.dumps({'preflight_rejected':str(ex)}),file=sys.stderr);return 2
 old=sys.argv
 try:
  sys.argv=['run_trial.py','--mission','--profile',args.profile,'--scenario',args.scenario,'--method','mppi','--run-id',args.run_id]+(['--check'] if args.check else [])
  return trial.main()
 finally:sys.argv=old
if __name__=='__main__':raise SystemExit(main())
