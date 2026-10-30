"""Harmonic + gz_ros2_control; no second drivetrain plugin or odometry source."""
import os
import tempfile
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mecanum_bringup.configuration import controller_config


def setup(context):
    share = get_package_share_directory('mecanum_bringup')
    geometry = LaunchConfiguration('geometry_file').perform(context)
    config = controller_config(os.path.join(share, 'config/controllers.yaml'), geometry)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(config, stream)
        controller_file = stream.name
    description = xacro.process_file(os.path.join(share, 'urdf/mecanum.urdf.xacro'), mappings={
        'geometry_file': geometry, 'controllers_file': controller_file, 'simulation': 'true',
    }).toxml()

    def cleanup(event, ctx):
        if os.path.exists(controller_file):
            os.unlink(controller_file)
        return []

    def start_if_success(next_actions):
        def callback(event, ctx):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='Robot/controller startup failed'))]
            return next_actions
        return callback

    spawn = Node(package='ros_gz_sim', executable='create', arguments=[
        '-name', 'inspection_mecanum', '-topic', 'robot_description', '-z', '0.03'], output='screen')
    jsb = Node(package='controller_manager', executable='spawner', arguments=[
        'joint_state_broadcaster', '--controller-manager-timeout', '90'], output='screen')
    drive = Node(package='controller_manager', executable='spawner', arguments=[
        'mecanum_drive_controller', '--controller-manager-timeout', '90'], output='screen')
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=start_if_success([jsb]))),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=start_if_success([drive]))),
        RegisterEventHandler(OnProcessExit(target_action=drive, on_exit=start_if_success([]))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', LaunchConfiguration('world')], output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
             parameters=[{'use_sim_time': True}], output='screen'),
        spawn,
    ]


def generate_launch_description():
    share = get_package_share_directory('mecanum_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds/acceptance.sdf')),
        DeclareLaunchArgument('geometry_file', default_value=os.path.join(share, 'config/geometry.yaml')),
        OpaqueFunction(function=setup),
    ])
