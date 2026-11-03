from launch import LaunchDescription
from launch_ros.actions import Node # for launch Node
from launch.substitutions import Command 
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_path = get_package_share_directory("mybot_description")
    xacro_file = os.path.join(package_path, "urdf", "second_robot", "second_robot.urdf.xacro")
    robot_description = Command(["xacro ", xacro_file]) # get the src of the urdf

    robot_state_publisher = Node(
        package="robot_state_publisher", #?
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description
            }
        ]
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        additional_env={"LIBGL_ALWAYS_SOFTWARE" : "1"
        }
    )


    # these things will be launch 
    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        rviz
    ])