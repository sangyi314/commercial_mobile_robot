#!/usr/bin/env python3
"""Read-only delivery audit; does not create ROS nodes, migrate DBs or play bags."""
if __package__ in (None,''):
 from pathlib import Path
 from runpy import run_path
 run_path(str(Path(__file__).resolve().parents[1]/'__init__.py'))['bootstrap']()
import argparse,json
from pathlib import Path
from tools.delivery.records import audit_protocol_case,audit_session
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case');p.add_argument('--session');p.add_argument('--output',required=True);a=p.parse_args()
 if bool(a.case)==bool(a.session):p.error('one case or session required')
 out=Path(a.output)
 if out.exists():p.error('refuse overwrite')
 try:r=audit_protocol_case(a.case) if a.case else audit_session(a.session)
 except Exception as exc:r={'valid':False,'session_pass':False,'errors':[type(exc).__name__+': '+str(exc)]}
 passed=r.get('session_pass',r['valid']);out.open('x').write(json.dumps(r,indent=2));print('AUDIT_PASS' if passed else 'AUDIT_FAIL',r['errors']);return 0 if passed else 3
if __name__=='__main__':raise SystemExit(main())
