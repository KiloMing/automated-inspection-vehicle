"""Start the sensor world and spawn the same description used by ros2_control."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = get_package_share_directory('mybot_description')
    description = xacro.process_file(os.path.join(
        share, 'urdf', 'second_robot', 'second_robot.urdf.xacro')).toxml()
    world = os.path.join(share, 'worlds', 'my_sensor_world.sdf')
    gui = LaunchConfiguration('gui')
    env = {
        'GZ_IP': '127.0.0.1',
        'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib' + os.pathsep
        + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        'LIBGL_DRI3_DISABLE': '1',
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'QT_QPA_PLATFORM': 'xcb',
    }
    # Keep the proven VM renderer; sensor definitions in the world stay intact.
    command = ['gz', 'sim', '--render-engine', 'ogre', '-r', '-v', '3', world]
    gui_config = os.path.expanduser('~/.gz/sim/8/gui-lite.config')
    gui_command = ['gz', 'sim', '-g', '--render-engine', 'ogre'] + (
        ['--gui-config', gui_config] if os.path.isfile(gui_config) else [])
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': description, 'use_sim_time': True}],
            output='screen'),
        ExecuteProcess(cmd=gui_command, additional_env=env,
                       condition=IfCondition(gui), output='screen'),
        ExecuteProcess(cmd=command + ['-s'], additional_env=env,
                       output='screen'),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            additional_env=env, output='screen'),
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-world', 'my_world', '-topic', '/robot_description',
                       '-name', 'second_robot', '-z', '0.2'],
            additional_env=env, output='screen'),
    ])
