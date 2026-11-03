import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description(): 
    #get the nav2_launch path
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch_file = os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
    rviz_config_file = os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz")

    #map and the params of Nav2
    map_file = "/home/kiloming/my_ros/chapt6_ws/maps/test_map.yaml"
    nav2_params_file = ("/home/kiloming/my_ros/chapt6_ws/src/" "mybot_description/config/nav2_params.yaml")

    #launch Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "map": map_file,
            "params_file": nav2_params_file,
            "use_sim_time" : "true",
            "use_composition" : "False",
            "autostart" : "true",
        }.items()
    )

    #launch RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2", 
        name="rviz2",
        arguments=[
            "-d",
            rviz_config_file
        ],
        parameters=[
            {
                "use_sim_time": True
            }
        ],
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "1"
        },
        output="screen"
    )
    #launch Description
    return LaunchDescription([
        nav2_launch,
        rviz_node,
    ])