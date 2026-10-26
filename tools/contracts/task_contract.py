"""Versioned frozen task identities; legacy evidence is resource provenance only."""
import json,math
from pathlib import Path
from tools.runner.runner_core import digest
VERSION='p0_long_route_v1'
INITIAL={'frame_id':'map','x_m':-2.0,'y_m':-.5,'yaw_rad':0.0}
GOAL={'frame_id':'map','x_m':1.75,'y_m':-.5,'yaw_rad':0.0}
def request_pose(request,initial=False):
 p=request['pose']['pose'];q=p['orientation']
 if set(q)!={'x','y','z','w'} or any(type(x) not in (int,float) or not math.isfinite(x) for x in q.values()) or abs(math.hypot(*q.values())-1)>1e-9:raise ValueError('task quaternion')
 if p['position'].get('z')!=0 or q['x']!=0 or q['y']!=0:raise ValueError('task planar request')
 header=request['header'] if initial else request['pose']['header']
 return {'frame_id':header['frame_id'],'x_m':p['position']['x'],'y_m':p['position']['y'],'yaw_rad':math.atan2(2*q['w']*q['z'],1-2*q['z']**2)}
def validate_task(spec,profile):
 if profile.get('task_version')!=VERSION:raise ValueError('unknown task version')
 binding=profile['task_inputs'];path=Path(binding['path'])
 if not path.is_absolute() or digest(path)!=binding['sha256']:raise ValueError('task input hash')
 frozen=json.loads(path.read_text())
 if frozen['task_version']!=VERSION or frozen['initial_pose']!=INITIAL or frozen['goal_pose']!=GOAL:raise ValueError('unreviewed versioned task')
 if spec['task']['initial_pose']!=INITIAL or spec['task']['goal_pose']!=GOAL:raise ValueError('scenario task version mismatch')
 for name in ['goal','initialpose','constraints','robot','map_yaml','map_image']:
  actual=profile['files'][name];expected=frozen['files'][name]
  if actual['sha256']!=expected['sha256'] or digest(actual['path'])!=expected['sha256']:raise ValueError('frozen task resource '+name)
 for name in ['robot','map_yaml','map_image']:
  if spec['resources'][name]['sha256']!=frozen['files'][name]['sha256']:raise ValueError('scenario task resource '+name)
 if spec['task']['common_constraints_ref']['sha256']!=frozen['files']['constraints']['sha256']:raise ValueError('task constraints')
 goal=json.loads(Path(profile['files']['goal']['path']).read_text());initial=json.loads(Path(profile['files']['initialpose']['path']).read_text())
 if request_pose(goal)!=GOAL or goal['pose']['pose']['orientation']!={'x':0,'y':0,'z':0,'w':1} or goal['behavior_tree']!='':raise ValueError('task request mismatch')
 if request_pose(initial,True)!=INITIAL:raise ValueError('initial request mismatch')
 sp=profile['spawn']
 if [sp[k] for k in ['x','y','yaw']]!=[-2.,-.5,0.]:raise ValueError('spawn task mismatch')
 return frozen

def poses_equal(a,b):
 return a['frame_id']==b['frame_id'] and all(abs(a[k]-b[k])<=1e-9 for k in ['x_m','y_m','yaw_rad'])
