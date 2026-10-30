"""Nav2 only. Start simulation or accepted hardware odometry/lidar separately."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('mecanum_bringup')
    nav2 = get_package_share_directory('nav2_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to a map YAML matching the environment'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config/nav2_omni.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav2, 'launch/bringup_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'use_composition': 'False', 'autostart': 'true', 'slam': 'False'}.items()),
    ])
