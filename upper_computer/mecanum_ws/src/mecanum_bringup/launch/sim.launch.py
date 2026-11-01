from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = PathJoinSubstitution([FindPackageShare("mecanum_description"), "urdf", "mecanum_robot.urdf.xacro"])
    world = PathJoinSubstitution([FindPackageShare("mecanum_bringup"), "worlds", "mecanum_world.sdf"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
    )
    publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen",
        parameters=[{"robot_description": ParameterValue(Command(["xacro ", model]), value_type=str), "use_sim_time": True}],
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )
    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-world", "mecanum_world", "-topic", "/robot_description", "-name", "mecanum_robot", "-z", "0.15"],
    )
    controllers = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["joint_state_broadcaster", "mecanum_drive_controller", "--controller-manager-timeout", "60"],
    )
    return LaunchDescription([
        gazebo, publisher, bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[controllers])),
        spawn,
    ])
