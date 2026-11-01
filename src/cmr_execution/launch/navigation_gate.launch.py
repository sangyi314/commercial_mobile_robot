# SPDX-License-Identifier: MIT
"""Minimal Jazzy node composition, based on official Navigation2 launch design.
Reference d5d768b7b38a05e7cfd2b6c693b786b402161bed navigation_launch.py
(Apache-2.0). Independent assembly; docking is intentionally absent.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
NODES=[('nav2_controller','controller_server'),('nav2_smoother','smoother_server'),('nav2_planner','planner_server'),('nav2_route','route_server'),('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator'),('nav2_waypoint_follower','waypoint_follower'),('nav2_velocity_smoother','velocity_smoother'),('nav2_collision_monitor','collision_monitor')]
def setup(context):
 for key in ['autostart','use_composition','use_respawn']:
  if LaunchConfiguration(key).perform(context).lower()!='false':raise RuntimeError('gate profile requires explicit non-composed, no-respawn, manual lifecycle')
 params=RewrittenYaml(source_file=LaunchConfiguration('params_file'),root_key='',param_rewrites={'use_sim_time':'True','autostart':'False'},convert_types=True)
 nodes=[]
 for package,name in NODES:
  remaps=[('/tf','tf'),('/tf_static','tf_static')]
  if name in ['controller_server','behavior_server','velocity_smoother']:remaps.append(('cmd_vel','cmd_vel_nav'))
  nodes.append(Node(package=package,executable=name,name=name,output='screen',parameters=[params,{"use_sim_time":True}],remappings=remaps))
 nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',output='screen',parameters=[{'use_sim_time':True,'autostart':False,'node_names':[n for p,n in NODES]}]))
 return nodes
def generate_launch_description():
 return LaunchDescription([DeclareLaunchArgument('params_file'),*[DeclareLaunchArgument(k,default_value=v) for k,v in [('use_sim_time','true'),('autostart','false'),('use_composition','false'),('use_respawn','false')]],OpaqueFunction(function=setup)])
