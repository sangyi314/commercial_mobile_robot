# SPDX-License-Identifier: MIT
import pytest
from tools.delivery.worker_policy import require_worker_operation,motion_qualification

def test_mission_worker_control_guard():
 for op in ['send','cancel','fault_ready','deactivate_controller','fault_deactivate','failure_stop']:
  with pytest.raises(RuntimeError,match='MISSION_OBSERVER_FORBIDS'):require_worker_operation(True,op)
  require_worker_operation(False,op)
 require_worker_operation(True,'manager');require_worker_operation(True,'mission_motion_ready')

def test_qualification_uses_active_identity_and_actual_displacement():
 rows=[{'kind':'odom','line':i+1,'receive_monotonic_s':20+i*.1,'data':{'stamp_s':10+i*.1,'position':[i*.02,0,0],'linear':[.1,0,0]}} for i in range(5)]
 uid='1'*32;statuses=[{'goal_uuid':uid,'status':2}]
 assert motion_qualification(rows,statuses,uid,10.4,20.4)['displacement_m']==.08
 for states,ros,steady in [([],10.4,20.4),(statuses,11.,20.4),(statuses,10.4,21.)]:
  with pytest.raises(ValueError,match='NOT_EXERCISED'):motion_qualification(rows,states,uid,ros,steady)


def test_private_oracle_feedback_never_reaches_navigation():
 from types import SimpleNamespace as Endpoint
 from tools.delivery.worker_policy import validate_body_feedback_graph
 import pytest
 ns='/cmr_execution/r';pub=Endpoint(node_name='execution_bridge',node_namespace=ns);observer=Endpoint(node_name='execution_observer',node_namespace=ns)
 validate_body_feedback_graph([pub],[observer,Endpoint(node_name='cmr_worker',node_namespace='/'),Endpoint(node_name='cmr_recorder_r',node_namespace='/')],ns)
 for name in ['amcl','controller_server','planner_server','mission_node','unknown']:
  with pytest.raises(RuntimeError,match='leaked'):validate_body_feedback_graph([pub],[observer,Endpoint(node_name=name,node_namespace=ns)],ns)
 with pytest.raises(RuntimeError,match='source'):validate_body_feedback_graph([pub,pub],[observer],ns)
 with pytest.raises(RuntimeError,match='source'):validate_body_feedback_graph([Endpoint(node_name='execution_bridge',node_namespace='/foreign')],[observer],ns)


def test_fault_needs_body_displacement_even_if_wheels_qualify():
 from tools.delivery.worker_policy import body_motion_qualification
 rows=[{'kind':'body_odom','line':i+1,'receive_monotonic_s':20+i*.1,'data':{'stamp_s':10+i*.1,'position':[i*.02,0,0],'linear':[.1,0,0]}} for i in range(5)]
 uid='1'*32;statuses=[{'goal_uuid':uid,'status':2}]
 assert body_motion_qualification(rows,statuses,uid,10.4,20.4)['displacement_m']==.08
 for r in rows:r['data']['position'][0]*=.5
 with pytest.raises(ValueError,match='NOT_EXERCISED'):body_motion_qualification(rows,statuses,uid,10.4,20.4)
