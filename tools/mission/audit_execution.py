#!/usr/bin/env python3
"""Read-only independent E-matrix audit: native logs, SQLite, traces, owned cleanup."""
import argparse,json,re,sqlite3
from pathlib import Path
def audit(root):
 results=json.loads((root/'results.json').read_text());assert len(results)==16 and {r['id'] for r in results}=={'E%02d'%i for i in range(1,17)}
 assert all(r['status']=='PASS' and r['cleanup']=='PASS' for r in results)
 summary=[]
 for result in results:
  case=root/result['id'];entries=[]
  for path in case.rglob('cleanup.json'):
   values=json.loads(path.read_text());assert values and all('escalation' not in x and x['exit']==x['expected_exit'] for x in values)
   for p in values:
    stat=Path('/proc')/str(p['pid'])/'stat'
    if stat.exists():assert stat.read_text().split(') ')[1].split()[19]!=next((str(x.get('start_ticks')) for x in json.loads((path.parent/'trace.json').read_text()) if x.get('pid')==p['pid'] and 'start_ticks' in x),'unknown')
   trace=json.loads((path.parent/'trace.json').read_text());states=[x['data'] for x in trace if x.get('kind')=='gate_state']
   assert states
   assert all(all(abs(s['output'][v][a])<=.001 for v in ['linear','angular'] for a in 'xyz') for s in states if not s['allowed'])
   assert all(s['synthetic'] for s in states)
   entry={'subcase':str(path.parent.relative_to(root)),'states':len(states),'reasons':sorted({s['reason'] for s in states}),'cleanup':'PASS'}
   db=path.parent/'mission.db'
   if db.exists():
    with sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True) as con:
     entry['sqlite']={t:con.execute('SELECT COUNT(*) FROM '+t).fetchone()[0] for t in ['missions','events','operations']}
     kinds=dict(con.execute('SELECT kind,count(*) FROM operations GROUP BY kind'));assert kinds.get(0)==1 and kinds.get(3)==1 and kinds.get(5)==1 and kinds.get(4,0)>0;entry['persisted_effect_kinds']=kinds
    sends=sum(p.read_text().count('GOAL_REQUEST ') for p in path.parent.glob('server*.log'));entry['native_server_goal_requests']=sends;assert sends==1
    text='\n'.join(p.read_text() for p in path.parent.glob('mission*.log'));assert 'GATE_GRANTED' in text or any(x.get('database') for x in trace)
   entries.append(entry)
  summary.append({'id':result['id'],'layer':result['layer'],'safety_protection':result['safety_protection'],'subcases':entries})
 return summary
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('root');p.add_argument('new_report');a=p.parse_args();report=audit(Path(a.root))
 with Path(a.new_report).open('x') as f:json.dump(report,f,indent=2)
 print('AUDIT_PASS',len(report),'E cases')
