#!/usr/bin/env python3
"""P0-03: attach to an existing TB3 Gazebo world, localize, then start Nav2.

Source the simulation's session.bash and tools/p0_03_env.bash, then supply its verified world pose:
  python3 tools/p0_03_navigation.py --initial-pose X Y YAW
No world or goal is created. Ctrl-C shuts down this launcher's processes only.
"""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Trigger
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory

NAV_NODES = ['controller_server', 'smoother_server', 'planner_server',
             'route_server', 'behavior_server', 'velocity_smoother',
             'collision_monitor', 'bt_navigator', 'waypoint_follower', 'docking_server']



class Startup:
    def __init__(self, pose):
        self.pose = pose
        self.children, self.managers, self.clients = [], [], {}
        self.last, self.clock, self.ticks = {}, None, 0
        self.node = Node('cmr_p0_03_start', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self.node)
        for key, cls in [('scan', LaserScan), ('odom', Odometry), ('clock', Clock)]:
            self.node.create_subscription(cls, '/' + key,
                lambda msg, k=key: self.receive(k, msg), qos_profile_sensor_data)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for key, cls in [('map', OccupancyGrid), ('amcl_pose', PoseWithCovarianceStamped)]:
            self.node.create_subscription(cls, '/' + key,
                lambda msg, k=key: self.receive(k, msg), durable)
        self.pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)

    def receive(self, key, msg):
        self.last[key] = time.monotonic()
        if key == 'clock':
            stamp = msg.clock.sec + msg.clock.nanosec * 1e-9
            self.ticks += int(self.clock is not None and stamp > self.clock)
            self.clock = stamp

    def call(self, client, request, timeout=2):
        if not client.wait_for_service(timeout_sec=0.2):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if future.done():
            return future.result()
        client.remove_pending_request(future)
        return None

    def in_state(self, names, state=3):
        for name in names:
            if name not in self.clients:
                self.clients[name] = self.node.create_client(GetState, '/' + name + '/get_state')
            response = self.call(self.clients[name], GetState.Request())
            if response is None or response.current_state.id != state:
                return False
        return True

    def wait(self, label, predicate, timeout=45):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            for name, process in self.children:
                if process.poll() is not None:
                    raise RuntimeError(f'{name} exited: {process.returncode}')
            if predicate():
                print('READY:', label, flush=True)
                return
        raise RuntimeError('Timed out: ' + label)

    def start(self, label, command):
        print('START:', label, flush=True)
        self.children.append((label, subprocess.Popen(command, start_new_session=True)))

    def wait_manager(self, phase, names):
        # Query the manager first; inspect node states after transitions finish.
        client = self.node.create_client(Trigger, f'/lifecycle_manager_{phase}/is_active')
        def ready():
            response = self.call(client, Trigger.Request())
            return response is not None and response.success
        self.wait(phase + ' manager active', ready, 60)
        self.wait(phase + ' nodes active', lambda: self.in_state(names), 10)

    def localized(self):
        if 'map' not in self.last or 'amcl_pose' not in self.last:
            return False
        try:
            stamp = self.tf.lookup_transform('map', 'base_link', Time()).header.stamp
            return abs(self.clock - stamp.sec - stamp.nanosec * 1e-9) < 2
        except Exception:
            return False

    def run(self):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        conflicts = set(self.node.get_node_names()).intersection(NAV_NODES + [
            'amcl', 'map_server', 'robot_state_publisher',
            'lifecycle_manager_localization', 'lifecycle_manager_navigation'])
        if conflicts or self.ticks:
            raise RuntimeError(f'Existing ROS owners: {sorted(conflicts)}; stop their launcher first')
        sim = Path(get_package_share_directory('nav2_minimal_tb3_sim'))
        nav = Path(get_package_share_directory('nav2_bringup'))
        config = Path(__file__).resolve().parents[1] / 'src/cmr_bringup/config/nav2_mppi.yaml'
        self.start('robot_state_publisher', [
            '/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher', '--ros-args',
            '-p', 'use_sim_time:=true', '-p', 'robot_description:=' +
            (sim / 'urdf/turtlebot3_waffle.urdf').read_text()])
        self.start('bridge', ['/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge', '--ros-args',
            '-p', 'use_sim_time:=true', '-p', 'expand_gz_topic_names:=true',
            '-p', f'config_file:={sim / "configs/turtlebot3_waffle_bridge.yaml"}'])
        self.wait('advancing real clock, scan and odom', lambda: self.ticks >= 10 and all(
            time.monotonic() - self.last.get(k, 0) < 1.5 for k in ('clock', 'scan', 'odom')))
        common = [f'params_file:={config}', 'use_sim_time:=true', 'autostart:=true',
                  'use_composition:=False', 'use_respawn:=False']
        self.managers.append('localization')
        self.start('localization', ['ros2', 'launch', '--noninteractive', 'nav2_bringup', 'localization_launch.py',
                   *common, f'map:={nav / "maps/tb3_sandbox.yaml"}'])
        self.wait_manager('localization', ['map_server', 'amcl'])
        self.wait('map and initialpose subscriber', lambda: 'map' in self.last and
                  self.pub.get_subscription_count() > 0)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        x, y, yaw = self.pose
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685389
        self.pub.publish(msg)
        print('INITIALPOSE sent once:', self.pose, msg.header.stamp, flush=True)
        self.wait('AMCL and fresh map -> base_link TF', self.localized, 30)
        # Do not activate global_costmap until localization has supplied map TF.
        self.managers.append('navigation')
        self.start('navigation', ['ros2', 'launch', '--noninteractive', 'nav2_bringup', 'navigation_launch.py', *common])
        self.wait_manager('navigation', NAV_NODES)
        print('P0-03 READY; no goal sent. Ctrl-C to shut down.', flush=True)
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            for name, proc in self.children:
                if proc.poll() is not None:
                    raise RuntimeError(f'{name} exited: {proc.returncode}')

    def close(self):
        for name, proc in reversed(self.children):
            # Keep localization and TF alive until navigation has fully exited.
            if name in self.managers and proc.poll() is None:
                client = self.node.create_client(ManageLifecycleNodes, f'/lifecycle_manager_{name}/manage_nodes')
                request = ManageLifecycleNodes.Request()
                request.command = 4
                response = self.call(client, request, 15)
                print('SHUTDOWN', name, response.success if response else 'no response', flush=True)
            if proc.poll() is None:
                # Noninteractive ros2 launch forwards this signal to its children.
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    print('Forced shutdown of owned process group:', name, flush=True)
                    os.killpg(proc.pid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()
            print('EXIT', name, proc.returncode, flush=True)
        self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--initial-pose', type=float, nargs=3, required=True, metavar=('X', 'Y', 'YAW'))
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '71' or not os.environ.get('GZ_PARTITION'):
        parser.error('Source the real CMR simulation session: domain 71 and GZ_PARTITION required')
    if not all(math.isfinite(v) for v in args.initial_pose):
        parser.error('Initial pose must be finite')
    profile = Path(__file__).resolve().parents[1] / 'src/cmr_bringup/config/fastdds_loopback_udp.xml'
    if (os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'SYSTEM_DEFAULT' or
            os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE') != str(profile) or
            os.environ.get('RMW_IMPLEMENTATION') != 'rmw_fastrtps_cpp'):
        parser.error('Source tools/p0_03_env.bash in every P0-03 terminal first')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    startup = Startup(args.initial_pose)
    try:
        startup.run()
    except KeyboardInterrupt:
        pass
    finally:
        startup.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
