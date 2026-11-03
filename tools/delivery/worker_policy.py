# SPDX-License-Identifier: MIT
"""Pure ownership guard; executed before any worker Goal operation."""
FORBIDDEN_GOAL_COMMANDS=frozenset({'send','cancel','fault_ready','deactivate_controller','fault_deactivate','failure_stop'})
def require_worker_operation(mission_observer,operation):
 if mission_observer and operation in FORBIDDEN_GOAL_COMMANDS:raise RuntimeError('MISSION_OBSERVER_FORBIDS_GOAL_CONTROL: '+operation)

def motion_qualification(rows,statuses,goal_uuid,now_ros,now_steady):
 import math
 if len(goal_uuid)!=32 or not any(x['goal_uuid']==goal_uuid and x['status']==2 for x in statuses):raise ValueError('NOT_EXERCISED: no matching ACTIVE Goal')
 samples=[x for x in rows if x['kind']=='odom' and x['data']['stamp_s'] is not None and 0<=now_ros-x['data']['stamp_s']<=1.5]
 if len(samples)<3:raise ValueError('NOT_EXERCISED: insufficient motion samples')
 stamps=[x['data']['stamp_s'] for x in samples]
 if any(b<=a for a,b in zip(stamps,stamps[1:])):raise ValueError('NOT_EXERCISED: nonadvancing motion')
 first,last=samples[0],samples[-1];displacement=math.dist(first['data']['position'][:2],last['data']['position'][:2]);speed=math.hypot(*last['data']['linear'][:2])
 if not 0<=now_ros-stamps[-1]<.2 or not 0<=now_steady-last['receive_monotonic_s']<.2 or displacement<.05 or speed<.01:raise ValueError('NOT_EXERCISED: not fresh moving')
 return {'goal_uuid':goal_uuid,'active_status':2,'samples':len(samples),'odom_lines':[first['line'],last['line']],'displacement_m':displacement,'linear_mps':speed,'source_sim_s':stamps[-1],'observer_monotonic_s':now_steady,'synthetic':False}


def validate_body_feedback_graph(publishers,subscribers,namespace):
 if len(publishers)!=1 or publishers[0].node_name!='execution_bridge' or publishers[0].node_namespace!=namespace:raise RuntimeError('Oracle body feedback source not unique owned Bridge')
 if not subscribers:raise RuntimeError('Oracle body feedback has no stop evaluator')
 for sub in subscribers:
  allowed=(sub.node_namespace==namespace and sub.node_name=='execution_observer') or (sub.node_namespace=='/' and (sub.node_name=='cmr_worker' or sub.node_name.startswith('cmr_recorder_')))
  if not allowed:raise RuntimeError('Oracle body feedback leaked outside stop evaluator/recorders: '+sub.node_namespace+'/'+sub.node_name)
 if not any(s.node_namespace==namespace and s.node_name=='execution_observer' for s in subscribers):raise RuntimeError('Oracle stop evaluator subscription absent')


def body_motion_qualification(rows,statuses,goal_uuid,now_ros,now_steady):
 body=[{**r,'kind':'odom'} for r in rows if r['kind']=='body_odom']
 result=motion_qualification(body,statuses,goal_uuid,now_ros,now_steady)
 result.update(source='Oracle private body_odom; not wheel integration',oracle_stop_assisted=True)
 return result
