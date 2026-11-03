
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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ========== 1. 启动参数 ==========

    # STM32 底盘串口
    chassis_port = LaunchConfiguration('chassis_port')
    chassis_baud = LaunchConfiguration('chassis_baud')

    # RPLIDAR C1 串口
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')

    # 雷达参数
    frame_id = LaunchConfiguration('frame_id')
    channel_type = LaunchConfiguration('channel_type')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')

    # 雷达安装位置
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')

    # SLAM 参数
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # RViz 开关
    use_rviz = LaunchConfiguration('use_rviz')

    # ========== 2. 配置路径 ==========

    sllidar_share = get_package_share_directory('sllidar_ros2')
    slam_share = get_package_share_directory('slam_toolbox')

    rviz_config = os.path.join(
        sllidar_share,
        'rviz',
        'sllidar_ros2.rviz'
    )

    slam_launch_file = os.path.join(
        slam_share,
        'launch',
        'online_async_launch.py'
    )

    # ========== 3. STM32 底盘节点 ==========

    chassis_node = Node(
        package='chassis_bridge',
        executable='chassis_bridge_node',
        name='chassis_bridge_node',
        output='screen',
        parameters=[{
            'port': chassis_port,
            'baud': ParameterValue(
                chassis_baud, value_type=int
            ),
            'use_sim_time': False,
        }]
    )

    # ========== 4. RPLIDAR C1 节点 ==========

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': channel_type,
            'serial_port': serial_port,
            'serial_baudrate': ParameterValue(
                serial_baudrate, value_type=int
            ),
            'frame_id': frame_id,
            'inverted': ParameterValue(
                inverted, value_type=bool
            ),
            'angle_compensate': ParameterValue(
                angle_compensate, value_type=bool
            ),
            'scan_mode': scan_mode,
            'use_sim_time': False,
        }]
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
        ]
    )

    # ========== 6. slam_toolbox 建图 ==========

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            slam_launch_file
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items()
    )

    # 启动后稍作延迟，让串口及 TF 先初始化
    delayed_slam = TimerAction(
        period=3.0,
        actions=[slam_node]
    )

    # ========== 7. RViz2 ==========

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': False,
        }],
        condition=IfCondition(use_rviz)
    )

    # ========== 8. LaunchDescription ==========

    return LaunchDescription([

        # 底盘参数
        DeclareLaunchArgument(
            'chassis_port',
            default_value='/dev/ttyUSB1'
        ),

        DeclareLaunchArgument(
            'chassis_baud',
            default_value='115200'
        ),

        # 雷达参数
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

        # 雷达安装坐标
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

        # SLAM 配置文件
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=(
                '/home/cat/mecanumws/config/'
                'mapper_params.yaml'
            )
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        # 是否启动 RViz
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true'
        ),

        LogInfo(
            msg='Starting STM32 chassis, RPLIDAR C1 and SLAM'
        ),

        # 启动各节点
        chassis_node,
        lidar_node,
        laser_tf_node,
        delayed_slam,
        rviz_node,
    ])
