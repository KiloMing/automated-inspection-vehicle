import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory("mybot_description")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_share,
                "launch",
                "gz_sim_control.launch.py"
            )
        )
    )

    nav2_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_share,
                        "launch",
                        "nav2_bringup.launch.py"
                    )
                )
            )
        ]
    )
    autopatrol_launch = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_share,
                        "launch",
                        "autopatrol.launch.py"
                    )
                )
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        nav2_launch,
        autopatrol_launch,
    ])