#!/usr/bin/env python3
"""Read-only independent stop analysis; never launches ROS or replays a bag."""
if __package__ in (None,''):
 from pathlib import Path
 from runpy import run_path
 run_path(str(Path(__file__).resolve().parents[1]/'__init__.py'))['bootstrap']()
import argparse,json
from pathlib import Path
from tools.execution.stop_analysis import evaluate
from tools.runner.runner_core import write_json
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--observations',required=True);p.add_argument('--marker',required=True);p.add_argument('--output',required=True);a=p.parse_args();rows=[json.loads(x) for x in Path(a.observations).read_text().splitlines()]
 marks=[r for r in rows if r['kind']=='execution_marker' and r['data']['name']==a.marker]
 if not marks:raise ValueError('requested marker absent')
 proved=[r for r in rows if r['kind']=='execution_marker' and r['data']['name']=='EXECUTION_STOP_PROVED'];end=proved[-1]['receive_monotonic_s'] if proved else float('inf')
 report=evaluate([r for r in rows if r['receive_monotonic_s']<=end],marks[-1]['receive_monotonic_s']);write_json(a.output,report);print(json.dumps(report,ensure_ascii=False));raise SystemExit(0 if all(report['checks'].values()) else 3)
