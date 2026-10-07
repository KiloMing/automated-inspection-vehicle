from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    initial_pose_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="mybot_description",
                executable="local_init",
                name="local_init",
                parameters=[
                    {
                        "use_sim_time": True
                    }
                ],
                output="screen"
            )
        ]
    )

    patrol_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="mybot_description",
                executable="PatrolController",
                name="patrol_controller",
                parameters=[
                    {
                        "use_sim_time": True
                    }
                ],
                output="screen"
            )
        ]
    )
    speaker_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="autopatrol_speaker",
                executable="speaker",
                name="speaker",
                output="screen"
            )
        ]
    )
    return LaunchDescription([
        speaker_node,
        initial_pose_node,
        patrol_node,
    ])