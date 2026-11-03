import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # ============================================================
    # 1. 获取 package share 路径
    # ============================================================

    pkg_share = get_package_share_directory("mybot_description")

    xacro_file = os.path.join(
        pkg_share,
        "urdf",
        "second_robot",
        "second_robot.urdf.xacro"
    )

    world_file = os.path.join(
        pkg_share,
        "worlds",
        "testworld.sdf"
    )


    # ============================================================
    # 2. Xacro -> robot_description
    # ============================================================

    robot_description = xacro.process_file(
        xacro_file
    ).toxml()


    # ============================================================
    # 3. GUI 参数
    # ============================================================

    gui = LaunchConfiguration("gui")


    # ============================================================
    # 4. Gazebo 软件 GPU 环境
    #
    # 等价于我们之前 gzsafe 做的事情
    # ============================================================

    gz_env = {

        # Gazebo Transport
        "GZ_IP": "127.0.0.1",

        # 让 Gazebo 找到 gz_ros2_control 插件
        "GZ_SIM_SYSTEM_PLUGIN_PATH":
            "/opt/ros/jazzy/lib"
            + os.pathsep
            + os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),

        # 软件渲染
        "LIBGL_ALWAYS_SOFTWARE": "1",

        # Parallels / VM 下避免 DRI3 问题
        "LIBGL_DRI3_DISABLE": "1",

        # Qt
        "QT_QPA_PLATFORM": "xcb",
    }


    # ============================================================
    # 5. robot_state_publisher
    #
    # 负责：
    #
    # robot_description
    #       ↓
    # /robot_description
    #
    # 同时发布机器人 TF
    # ============================================================

    robot_state_publisher = Node(

        package="robot_state_publisher",

        executable="robot_state_publisher",

        name="robot_state_publisher",

        output="screen",

        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True
            }
        ]
    )


    # ============================================================
    # 6. Gazebo Server
    #
    # 注意：
    # 这里不用普通默认渲染
    # 使用软件 GPU + ogre
    # ============================================================

    gazebo_server = ExecuteProcess(

        cmd=[
            "gz",
            "sim",

            "--render-engine",
            "ogre",

            "-r",

            "-v",
            "3",

            "-s",

            world_file
        ],

        additional_env=gz_env,

        output="screen"
    )

    


    # ============================================================
    # 7. Gazebo GUI
    # ============================================================

    gazebo_gui = ExecuteProcess(

        cmd=[
            "gz",
            "sim",

            "-g",

            "--render-engine",
            "ogre"
        ],

        additional_env=gz_env,

        condition=IfCondition(gui),

        output="screen"
    )


    # ============================================================
    # 8. /clock Bridge
    #
    # Gazebo clock
    #     ↓
    # ROS2 /clock
    #
    # use_sim_time=true 依赖这个
    # ============================================================

    clock_bridge = Node(

        package="ros_gz_bridge",

        executable="parameter_bridge",

        name="clock_bridge",

        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],

        output="screen",

        additional_env=gz_env
    )

    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
        additional_env=gz_env,
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image"
        ],
        output="screen",
        additional_env=gz_env
    )
    # ============================================================
    # 9. Spawn Robot
    #
    # 不再使用：
    #
    #   create -file /tmp/second_robot.urdf
    #
    # 而是直接读取：
    #
    #   /robot_description
    #
    # 这样不会再出现昨天
    # robot_description 缺失的问题。
    # ============================================================

    spawn_robot = Node(

        package="ros_gz_sim",

        executable="create",

        name="spawn_second_robot",

        arguments=[
            "-world",
            "my_world",

            "-topic",
            "/robot_description",

            "-name",
            "second_robot",

            "-z",
            "0.2"
        ],

        output="screen",

        additional_env=gz_env
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # ============================================================
    # 10. 延迟 Spawn
    #
    # 先让 Gazebo world 启动，
    # 再把机器人放进去。
    # ============================================================

    delayed_spawn = TimerAction(

        period=3.0,

        actions=[
            spawn_robot
        ]
    )

    # ============================================================
    # 控制 diff_drive_controller_spawner 和 joint_state_broadcaster_spawner
    # 在spawn_robot之后进行
    # ============================================================

    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[
                        joint_state_broadcaster_spawner,
                        diff_drive_controller_spawner,
                    ]
                )
            ]
        )
    )


    # ============================================================
    # LaunchDescription
    # ============================================================

    return LaunchDescription([

        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo GUI"
        ),

        robot_state_publisher,

        gazebo_server,

        gazebo_gui,

        clock_bridge,

        delayed_spawn,

        scan_bridge,

        camera_bridge,
    
        start_controllers_after_spawn,
    ])