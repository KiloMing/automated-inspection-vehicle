
#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ========== 1. 启动参数 ==========

    # STM32 底盘
    chassis_port = LaunchConfiguration('chassis_port')
    chassis_baud = LaunchConfiguration('chassis_baud')
    chassis_cmd_topic = LaunchConfiguration('chassis_cmd_topic')

    # RPLIDAR C1
    channel_type = LaunchConfiguration('channel_type')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')

    # 雷达安装位置
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')

    # Nav2
    map_file = LaunchConfiguration('map_file')
    params_file = LaunchConfiguration('params_file')

    use_rviz = LaunchConfiguration('use_rviz')

    # ========== 2. 功能包路径 ==========

    nav2_share = get_package_share_directory(
        'nav2_bringup'
    )

    nav2_launch_file = os.path.join(
        nav2_share,
        'launch',
        'bringup_launch.py'
    )

    rviz_config = os.path.join(
        nav2_share,
        'rviz',
        'nav2_default_view.rviz'
    )

    # ========== 3. STM32 底盘 ==========

    chassis_node = Node(
        package='chassis_bridge',
        executable='chassis_bridge_node',
        name='chassis_bridge_node',
        output='screen',

        parameters=[{
            'port': chassis_port,
            'baud': ParameterValue(
                chassis_baud,
                value_type=int
            ),
            'use_sim_time': False,
        }],

        # 默认隔离导航速度，防止直接驱动电机
    )

    # ========== 4. RPLIDAR C1 ==========

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',

        parameters=[{
            'channel_type': channel_type,
            'serial_port': serial_port,

            'serial_baudrate': ParameterValue(
                serial_baudrate,
                value_type=int
            ),

            'frame_id': frame_id,

            'inverted': ParameterValue(
                inverted,
                value_type=bool
            ),

            'angle_compensate': ParameterValue(
                angle_compensate,
                value_type=bool
            ),

            'scan_mode': scan_mode,
            'use_sim_time': False,
        }],
    )

    # ========== 5. 雷达静态 TF ==========

    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_static_tf',
        output='screen',

        arguments=[
            '--x', laser_x,
            '--y', laser_y,
            '--z', laser_z,

            '--roll', '0',
            '--pitch', '0',
            '--yaw', laser_yaw,

            '--frame-id', 'base_link',
            '--child-frame-id', frame_id,
        ],
    )

    # ========== 6. Nav2 定位与导航 ==========

    # 该启动文件包含：
    # map_server
    # amcl
    # planner_server
    # controller_server
    # bt_navigator
    # velocity_smoother
    # collision_monitor
    # lifecycle managers

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            nav2_launch_file
        ),

        launch_arguments={
            'slam': 'False',
            'use_localization': 'True',

            'map': map_file,
            'params_file': params_file,

            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    # 延迟启动 Nav2
    delayed_nav2 = TimerAction(
        period=3.0,
        actions=[nav2_bringup]
    )

    # ========== 7. RViz2 ==========

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',

        arguments=[
            '-d',
            rviz_config
        ],

        parameters=[{
            'use_sim_time': False,
        }],

        condition=IfCondition(use_rviz),
    )

    # ========== 8. LaunchDescription ==========

    return LaunchDescription([

        # STM32 串口
        DeclareLaunchArgument(
            'chassis_port',
            default_value='/dev/ttyUSB1'
        ),

        DeclareLaunchArgument(
            'chassis_baud',
            default_value='115200'
        ),

        # 默认不将 Nav2 速度直接送给真实底盘
        DeclareLaunchArgument(
            'chassis_cmd_topic',
            default_value='/cmd_vel_manual'
        ),

        # RPLIDAR C1 串口
        DeclareLaunchArgument(
            'channel_type',
            default_value='serial'
        ),

        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0'
        ),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='460800'
        ),

        DeclareLaunchArgument(
            'frame_id',
            default_value='laser'
        ),

        DeclareLaunchArgument(
            'inverted',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'scan_mode',
            default_value='Standard'
        ),

        # 雷达静态 TF
        DeclareLaunchArgument(
            'laser_x',
            default_value='0.10'
        ),

        DeclareLaunchArgument(
            'laser_y',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'laser_z',
            default_value='0.15'
        ),

        DeclareLaunchArgument(
            'laser_yaw',
            default_value='0.0'
        ),

        # 已保存的地图
        DeclareLaunchArgument(
            'map_file',
            default_value=('/home/cat/mecanumws/maps/my_map.yaml')
        ),

        # Nav2 真实底盘参数
        DeclareLaunchArgument(
            'params_file',
            default_value=(
                '/home/cat/mecanumws/config/'
                'nav2_real.yaml'
            )
        ),

        # RViz2
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true'
        ),

        LogInfo(
            msg='Starting real robot Nav2 navigation system'
        ),

        # 启动节点
        chassis_node,
        lidar_node,
        laser_tf_node,
        delayed_nav2,
        rviz_node,
    ])

