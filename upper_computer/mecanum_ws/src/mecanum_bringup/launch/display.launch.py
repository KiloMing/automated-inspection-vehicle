from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Find the robot Xacro model
    robot_file = PathJoinSubstitution([
        FindPackageShare("mecanum_description"),
        "urdf",
        "mecanum_robot.urdf.xacro"
    ])

    # Convert Xacro into URDF
    robot_description = Command([
        "xacro ",
        robot_file
    ])

    # Publish robot TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description
        }]
    )

    # Publish test joint positions
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )

    # Open RViz2
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
