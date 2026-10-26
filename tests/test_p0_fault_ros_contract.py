"""Explicit fixture interfaces in isolated domain 72; never real fault evidence."""
from test_runner_ros_contract import ros,session
from runner_core import DeadlineBudget
import pytest

@pytest.mark.parametrize('stuck',[False,True])
def test_shutdown_actual_mixed_states_and_bounded_failure(session,stuck):
 from lifecycle_msgs.srv import GetState,ChangeState
 fake,run,link,supervisor=session()
 names=['controller_server','smoother_server','planner_server','route_server','behavior_server','velocity_smoother','collision_monitor','bt_navigator','waypoint_follower','docking_server']
 states={n:[1,2,3,4][i%4] for i,n in enumerate(names)};calls=[];initial=states.copy()
 for name in names:
  def state(req,res,name=name):res.current_state.id=states[name];return res
  def change(req,res,name=name):
   calls.append((name,req.transition.id));res.success=not stuck
   if not stuck:states[name]=4
   return res
  fake.n.create_service(GetState,'/'+fake.ns+'/'+name+'/get_state',state,callback_group=fake.group)
  fake.n.create_service(ChangeState,'/'+fake.ns+'/'+name+'/change_state',change,callback_group=fake.group)
 if stuck:
  with pytest.raises(RuntimeError,match='repeated'):link.rpc('shutdown_actual',{'phase':'navigation'},DeadlineBudget.after(5))
  assert len(calls)==1
 else:
  result=link.rpc('shutdown_actual',{'phase':'navigation'},DeadlineBudget.after(5));assert result['success'] and all(s==4 for s in states.values())
  assert all(t=={1:5,2:6,3:7}[initial[n]] for n,t in calls)
  assert len(calls)==len([s for s in initial.values() if s!=4])
  again=link.rpc('shutdown_actual',{'phase':'navigation'},DeadlineBudget.after(3));assert again['transitions']==[]

def test_new_fault_whitelist_rejects_legacy_profile(session):
 fake,run,link,s=session()
 with pytest.raises(RuntimeError,match='whitelist'):link.rpc('fault_deactivate',{'uuid':'1'*32},DeadlineBudget.after(2))
 assert fake.count==0

@pytest.mark.parametrize('stable',[False,True])
def test_coordinated_manager_waits_before_shutdown(session,stable):
 from lifecycle_msgs.srv import GetState
 from nav2_msgs.srv import ManageLifecycleNodes
 from std_srvs.srv import Trigger
 fake,run,link,s=session()
 names=['controller_server','smoother_server','planner_server','route_server','behavior_server','velocity_smoother','collision_monitor','bt_navigator','waypoint_follower','docking_server']
 calls=[];state_id=[3]
 def state(req,res):res.current_state.id=state_id[0];return res
 def gate(req,res):res.success=stable;return res
 def shutdown(req,res):
  calls.append(req.command);assert req.command==req.SHUTDOWN
  state_id[0]=4;res.success=True;return res
 for name in names:fake.n.create_service(GetState,'/'+fake.ns+'/'+name+'/get_state',state,callback_group=fake.group)
 fake.n.create_service(Trigger,'/'+fake.ns+'/lifecycle_manager_navigation/is_active',gate,callback_group=fake.group)
 fake.n.create_service(ManageLifecycleNodes,'/'+fake.ns+'/lifecycle_manager_navigation/manage_nodes',shutdown,callback_group=fake.group)
 if stable:
  value=link.rpc('shutdown_actual',{'phase':'navigation','coordinate_manager':True},DeadlineBudget.after(4));assert value['manager_coordinated'] and len(calls)==1
 else:
  with pytest.raises((RuntimeError,TimeoutError)):link.rpc('shutdown_actual',{'phase':'navigation','coordinate_manager':True},DeadlineBudget.after(1))
  assert not calls

