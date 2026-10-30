"""Opt-in reconstruction of RF2O + SLAM; existing Camsense driver starts externally."""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    share = get_package_share_directory('mecanum_bringup')
    description = xacro.process_file(os.path.join(share, 'urdf/mecanum.urdf.xacro'), mappings={
        'geometry_file': LaunchConfiguration('geometry_file').perform(context),
        'simulation': 'false'}).toxml()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': False}]),
        # Wheel positions are for visualization only, not wheel odometry.
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'use_sim_time': False}]),
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry', parameters=[os.path.join(share, 'config/rf2o.yaml')], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch/online_async_launch.py')),
            launch_arguments={'use_sim_time': 'false', 'autostart': 'true',
                'slam_params_file': os.path.join(share, 'config/slam.yaml')}.items()),
    ]


def generate_launch_description():
    share = get_package_share_directory('mecanum_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('geometry_file', default_value=os.path.join(share, 'config/geometry.yaml')),
        OpaqueFunction(function=setup),
    ])
