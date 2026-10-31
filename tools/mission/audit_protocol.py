#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Read-only independent evidence audit; writes a NEW report supplied by the user."""
import argparse,json,re,sqlite3
from pathlib import Path

def audit(root):
 results=json.loads((root/'results.json').read_text());summary=[]
 for result in results:
  case=root/result['id'];assert result['status']=='PASS',result
  server=''.join(p.read_text() for p in sorted(case.glob('server*.log')))
  mission=''.join(p.read_text() for p in sorted(case.glob('mission*.log')))
  requests=re.findall(r'GOAL_REQUEST uuid=([0-9a-f]{32})',server)
  send_calls=re.findall(r'^SEND_CALL (.+)$',mission,re.M)
  assert len(send_calls)==len(requests),(case,'external request count',send_calls,requests)
  assert len(requests)==len(set(requests)),(case,'duplicate external UUID')
  with sqlite3.connect('file:'+str((case/'mission.db').resolve())+'?mode=ro',uri=True) as db:
   accepted=[identity.split(' detail=')[0].rsplit('/',1)[1] for (identity,) in db.execute("SELECT identity FROM events WHERE kind='1' AND identity LIKE '%ACTION_ACCEPTED%'")]
   intents=db.execute('SELECT count(*) FROM operations WHERE kind=0').fetchone()[0]
   active=db.execute('SELECT count(*) FROM missions WHERE terminal=0').fetchone()[0]
  assert set(accepted)<=set(requests),(case,'UUID has no independent server origin')
  assert intents>=len(requests) and active<=1
  canceled=re.findall(r'CANCEL_REQUEST uuid=([0-9a-f]{32})',server)
  assert set(canceled)<=set(requests),(case,'cancel unknown UUID')
  peak=[int(x) for x in re.findall(r'ACTIVE peak=(\d+)',server)];assert max(peak or [0])<=1
  cleanup=json.loads((case/'cleanup.json').read_text());assert all(x['exit']==x['expected_exit'] for x in cleanup)
  trace=json.loads((case/'trace.json').read_text())
  for entry in trace:
   if 'spawn' in entry:
    stat=Path('/proc')/str(entry['pid'])/'stat'
    if stat.exists():assert stat.read_text().split(') ')[1].split()[19]!=entry['start_ticks'],'owned process still alive'
  row={'id':result['id'],'native_send_calls':len(send_calls),'server_goal_requests':len(requests),'persisted_send_intents':intents,'persisted_accepted_uuids':accepted,'server_uuids':requests,'cancel_uuids':canceled,'peak_active':max(peak or [0]),'cleanup':'PASS'}
  if result['id']=='A17':
   inputs=[json.loads(line) for line in (case/'replay-input.jsonl').read_text().splitlines()]
   mapping={x['request']['command_id']:x for x in inputs}
   received=[]
   for seq,op,command,mission_id in re.findall(r'BUSINESS_RX seq=(\d+) operation=(\w+) command=(\w*) mission=(\w+)',mission):
    if command in mapping:received.append({'receive_seq':int(seq),'operation':op,'mission_id':mission_id,'input':mapping[command]})
   assert len(received)==len(inputs) and len({x['input']['request']['command_id'] for x in received})==len(inputs)
   row['business_inputs']=len(inputs);row['receive_order']=received
  summary.append(row)
 return summary
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('root');p.add_argument('new_report');args=p.parse_args()
 report=audit(Path(args.root))
 with Path(args.new_report).open('x') as f:json.dump(report,f,indent=2)
 print('AUDIT_PASS',len(report),'cases')
