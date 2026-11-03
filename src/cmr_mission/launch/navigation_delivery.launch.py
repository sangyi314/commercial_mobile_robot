# SPDX-License-Identifier: MIT
"""P1 delivery composition. Original Runner owns this launch process."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
NODES=[('nav2_controller','controller_server'),('nav2_smoother','smoother_server'),('nav2_planner','planner_server'),('nav2_route','route_server'),('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator'),('nav2_velocity_smoother','velocity_smoother'),('nav2_collision_monitor','collision_monitor')]
def setup(context):
 for key in ['autostart','use_composition','use_respawn']:
  if LaunchConfiguration(key).perform(context).lower()!='false':raise RuntimeError('delivery requires owned noncomposed no-respawn manual lifecycle')
 params=RewrittenYaml(source_file=LaunchConfiguration('params_file'),root_key='',param_rewrites={'use_sim_time':'True','autostart':'False'},convert_types=True)
 nodes=[]
 for package,name in NODES:
  remaps=[('/tf','tf'),('/tf_static','tf_static')]
  if name in ['controller_server','behavior_server','velocity_smoother']:remaps.append(('cmd_vel','cmd_vel_nav'))
  if name=='bt_navigator':remaps.append(('goal_pose',LaunchConfiguration('disabled_goal_pose')))
  nodes.append(Node(package=package,executable=name,name=name,output='screen',parameters=[params,{'use_sim_time':True}],remappings=remaps))
 nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',output='screen',parameters=[{'use_sim_time':True,'autostart':False,'node_names':[n for p,n in NODES]}]))
 return nodes
def generate_launch_description():
 return LaunchDescription([DeclareLaunchArgument('params_file'),DeclareLaunchArgument('disabled_goal_pose'),*[DeclareLaunchArgument(k,default_value=v) for k,v in [('use_sim_time','true'),('autostart','false'),('use_composition','false'),('use_respawn','false')]],OpaqueFunction(function=setup)])
