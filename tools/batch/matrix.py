"""Thin serial dispatcher: only starts the existing single-trial parent."""
import argparse,hashlib,json,os,shutil,signal,subprocess,sys,time
from pathlib import Path
from runner_core import strict_json,write_json,encode,digest,run_id
from runner_processes import ProjectLock,identity
from batch_core import *
ROOT=Path(__file__).resolve().parents[1]

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--matrix');ap.add_argument('--batch-dir');ap.add_argument('--resume');ap.add_argument('--limit',type=int,default=1);ap.add_argument('--check',action='store_true');ap.add_argument('--resume-reason');a=ap.parse_args()
 try:
  if bool(a.resume)==bool(a.matrix) or bool(a.matrix)!=bool(a.batch_dir) or a.limit<1:raise ValueError('new matrix/batch-dir OR resume; positive limit')
  batch=Path(a.resume or a.batch_dir).resolve();run_id(batch.name)
  if batch.parent!=ROOT/'evidence/batches':raise ValueError('batch must be directly under evidence/batches')
  if a.resume:
   manifest=strict_json((batch/'batch-manifest.json').read_text());matrix=strict_json((batch/'matrix-frozen.yaml').read_text())
   if digest(batch/'matrix-frozen.yaml')!=manifest['matrix_sha256']:raise ValueError('frozen matrix changed')
  else:
   if batch.exists():raise FileExistsError('batch exists; explicit resume required')
   matrix=strict_json(Path(a.matrix).read_text());manifest=None
  validate_matrix(matrix,ROOT);inventory=source_inventory(ROOT,matrix)
  if manifest and inventory!=manifest['inputs']:raise ValueError('cohort drift; preserve old batch and create a new version')
  if a.check:
   if a.resume:choose_slot(matrix,journal_read(batch/'journal.jsonl'),a.resume_reason)
   print(json.dumps({'check':'VALID','slots':len(matrix['slots']),'purpose':matrix['purpose'],'methods':{m:sum(s['method']==m for s in matrix['slots']) for m in ['mppi','dwb']},'batch_directory_exists':batch.exists(),'no_processes_or_directories_created':True}));return 0
  # Global batch dispatch lock is intentionally NOT the single-trial project lock.
  (ROOT/'evidence/batches').mkdir(exist_ok=True)
  with ProjectLock(ROOT/'evidence/env/p0-08-dispatch.lock'):
   if not a.resume:
    batch.mkdir();write_json(batch/'matrix-frozen.yaml',matrix)
    manifest={'version':1,'batch_id':batch.name,'matrix_sha256':digest(batch/'matrix-frozen.yaml'),'inputs':inventory,'cohort_sha256':hashlib.sha256(encode(inventory)).hexdigest(),'created_unix_s':time.time(),'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'simulation_only':True}
    write_json(batch/'batch-manifest.json',manifest)
   journal=Journal(batch/'journal.jsonl')
   if a.resume_reason:journal.append('REVIEW_DECISION',reason=a.resume_reason)
   # Recovery never launches an unresolved intent. Closed output can be reconciled;
   # absent CALL_STARTED cannot prove whether fork occurred, hence no silent replay.
   for aid,record in attempts(journal.rows).items():
    if record['finished']:continue
    if not record['started']:raise ValueError('intent without durable child identity; explicit forensic resolution required')
    birth=record['started']['identity'];current=identity(birth['pid'])
    if current and current['start_ticks']==birth['start_ticks'] and current['state']!='Z':raise ValueError('previous runner still alive; no dispatch')
    run=ROOT/'evidence/runs'/record['intent']['run_id'];exitfile=run/'runtime/runner-exit.json'
    if not exitfile.exists() or live_owned(run):raise ValueError('previous attempt lacks closed record or still owns processes')
    code=strict_json(exitfile.read_text())['exit'];out=audit_run(ROOT,record['intent'],code)
    if not out['safe_to_resume']:raise ValueError('recovery evidence unsafe')
    journal.append('RECOVERED',attempt_id=aid,exit_code=code,outcome=out)
   invoked=0
   while invoked<a.limit:
    slot=choose_slot(matrix,journal.rows,a.resume_reason)
    if slot is None:journal.append('COMPLETE');print('COMPLETE',flush=True);return 0
    if source_inventory(ROOT,matrix)!=inventory:raise ValueError('cohort drift before dispatch')
    if shutil.disk_usage(ROOT).free<3*1024**3:raise ValueError('less than 3 GiB free; preserve cleanup reserve')
    for prev in attempts(journal.rows).values():
     if live_owned(ROOT/'evidence/runs'/prev['intent']['run_id']):raise ValueError('owned descendant remains')
    number=len(attempts(journal.rows))+1;aid=f'a{number:03d}';rid=run_id(f'{batch.name}-{slot["slot_id"]}-{aid}')
    argv=[sys.executable,'-B',str(ROOT/'tools/run_trial.py'),'--scenario',slot['scenario']['path'],'--method',slot['method'],'--profile',slot['profile']['path'],'--run-id',rid]
    if slot['fault']:argv+=['--fault',slot['fault'],'--ack-test-fault']
    intent=journal.append('INTENT',batch_id=batch.name,slot_id=slot['slot_id'],attempt_id=aid,run_id=rid,method=slot['method'],purpose=slot['purpose'],argv=argv,profile=slot['profile'],scenario=slot['scenario'],cohort_sha256=manifest['cohort_sha256'])
    env=os.environ.copy();env.update(CMR_BATCH_ID=batch.name,CMR_ATTEMPT_ID=aid,CMR_SLOT_ID=slot['slot_id']);env['PYTHONDONTWRITEBYTECODE']='1'
    with (batch/f'{aid}-runner.log').open('xb') as output:
     child=subprocess.Popen(argv,cwd=ROOT,env=env,stdin=subprocess.DEVNULL,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
     birth=identity(child.pid);journal.append('CALL_STARTED',attempt_id=aid,identity=birth,boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip());invoked+=1;interrupted=[False]
     def interrupt(signum,frame):
      if interrupted[0]:return
      interrupted[0]=True;now=identity(child.pid)
      if now and now['start_ticks']==birth['start_ticks']:child.send_signal(signal.SIGINT)
     old=signal.signal(signal.SIGINT,interrupt);oldterm=signal.signal(signal.SIGTERM,interrupt);start=time.monotonic()
     try:
      while child.poll() is None:
       if time.monotonic()-start>510:
        journal.append('OUTER_DEADLINE_UNRESOLVED',attempt_id=aid,identity=birth);raise RuntimeError('runner exceeds outer bound; no new dispatch and no forced kill')
       time.sleep(.2)
     finally:signal.signal(signal.SIGINT,old);signal.signal(signal.SIGTERM,oldterm)
    outcome=audit_run(ROOT,intent,child.returncode);write_json(batch/f'{aid}-audit.json',outcome);journal.append('CALL_FINISHED',attempt_id=aid,exit_code=child.returncode,outcome=outcome)
    print(json.dumps({'attempt':aid,'run':rid,'exit_code':child.returncode,'pass':outcome['pass'],'reason':outcome['first_reason'],'audit_error':outcome.get('audit_error')},ensure_ascii=False),flush=True)
    if not outcome['pass'] or interrupted[0]:journal.append('PAUSED_FAILURE',attempt_id=aid);return 3
    a.resume_reason=None
   complete=choose_slot(matrix,journal.rows) is None;journal.append('COMPLETE' if complete else 'PAUSED_AT_LIMIT');print('COMPLETE' if complete else 'PAUSED_AT_LIMIT',flush=True);return 0
 except Exception as exc:print(json.dumps({'batch_refused':str(exc)},ensure_ascii=False),file=sys.stderr);return 2

if __name__=='__main__':sys.exit(main())
