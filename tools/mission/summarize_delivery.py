#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Offline, independently re-audited delivery-session summary; no ROS Node/playback."""
if __package__ in (None,''):
 from pathlib import Path
 from runpy import run_path
 run_path(str(Path(__file__).resolve().parents[1]/'__init__.py'))['bootstrap']()
import argparse,json
from pathlib import Path
from tools.delivery.records import audit_session
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--sessions',nargs='+',required=True);p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output)
 if out.exists():p.error('refuse overwrite')
 rows=[]
 for value in a.sessions:
  root=Path(value).resolve()
  try:
   audit=audit_session(root);db=audit['database'];rows.append({'session_id':root.name,'oracle_stop_assisted':audit.get('oracle_stop_assisted',False),'valid':audit['valid'],'session_pass':audit['session_pass'],'errors':audit['errors'],'unique_admitted':db['unique_admitted'],'submit_calls':db['submit_calls'],'states':{id:r['state'] for id,r in db['records'].items()},'handover_sources':{id:r.get('handover_source','') for id,r in db['records'].items()},'business_ok':audit['business_ok'],'cleanup_ok':audit['cleanup_ok'],'bag_ok':audit['bag_ok']})
  except Exception as e:rows.append({'session_id':root.name,'valid':False,'session_pass':False,'errors':[str(e)]})
 report={'schema_version':1,'record_family':'cmr.delivery-summary','sessions':rows,'passed_sessions':sum(r['session_pass'] for r in rows),'total_sessions':len(rows),'unique_admitted':sum(r.get('unique_admitted',0) for r in rows),'submit_calls':sum(r.get('submit_calls',0) for r in rows),'cross_session_ids_are_not_merged':True,'C':'CONSUMER_ACK_UNKNOWN','collision':None}
 with out.open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
 print(json.dumps({k:v for k,v in report.items() if k!='sessions'}));return 0 if all(r['session_pass'] for r in rows) else 3
if __name__=='__main__':raise SystemExit(main())
