# 自动巡检小车

当前在做四轮麦克纳姆底盘。先用 ROS 2 Jazzy 和 Gazebo Sim 验证全向运动，再接到真实底盘上。

截至 2026-11-01，仿真已验证前进、横移和旋转，`odom → base_link` TF 可以查询。轮式里程计已接通，但精度还没有验收。下一步先用鲁班猫控制实体麦克纳姆底盘，打通速度下发和四轮反馈，再做里程计标定。

## 目录

| 目录 | 内容 |
| --- | --- |
| `upper_computer/mecanum_ws/` | 当前四轮麦克纳姆仿真：模型、控制器、启动文件和场景 |
| `upper_computer/chapt6_ws/` | 原双轮差速仿真、Nav2 和巡检程序 |
| `upper_computer/real_lidar/` | Camsense X1 雷达代码 |
| `upper_computer/docs/` | 仿真进度、测试步骤和雷达实测记录 |
| `chassis/` | 原 STM32 双轮底盘固件和开发记录 |
| `data/` | 原电机测试数据、标定结果和分析脚本 |

## 麦克纳姆仿真

使用 Gazebo Sim、`gz_ros2_control` 和 `MecanumDriveController`。四轮独立接收速度命令，提供位置和速度状态；控制频率为 100 Hz。

模型轮半径为 0.05 m，轮心相对底盘中心的前后偏移为 ±0.11 m，左右偏移为 ±0.15 m。车轮采用圆柱外观、球形碰撞和方向性摩擦近似，没有逐个建模滚子。

| 接口 | 类型 / 用途 |
| --- | --- |
| `/mecanum_drive_controller/reference` | `geometry_msgs/msg/TwistStamped`，使用 `linear.x`、`linear.y`、`angular.z` |
| `/mecanum_drive_controller/odometry` | `nav_msgs/msg/Odometry`，轮式里程计 |
| `/joint_states` | 四轮状态 |
| `/tf` | 包含 `odom → base_link` 和轮子关节变换 |

里程计 TF 的重映射写在 Gazebo ros2_control 插件内部：

```xml
<ros>
  <remapping>/mecanum_drive_controller/tf_odometry:=/tf</remapping>
</ros>
```

在已经装好 ROS 2 Jazzy 和所需依赖的 Ubuntu 24.04 环境中，从仓库根目录执行：

```bash
cd upper_computer/mecanum_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
ros2 launch mecanum_bringup sim.launch.py
```

依赖安装、运动指令和 TF 检查见 [麦克纳姆仿真测试指南](upper_computer/docs/mecanum-simulation-testing.md)。本次整理只提交源码，不提交 `build/`、`install/` 和 `log/`。

## 进度与下一步

| 项目 | 状态 |
| --- | --- |
| 仿真前进、横移、旋转 | 2026-11-01 已验证 |
| `odom → base_link` TF 查询 | 2026-11-01 已验证 |
| 轮式里程计精度 | 待测量与标定 |
| 麦克纳姆底盘接入 SLAM Toolbox | 尚未验收 |
| Nav2 全向导航 | 尚未验收 |
| 鲁班猫控制实体麦克纳姆底盘 | 下一步，尚未验收 |

以上运动和 TF 结果来自当天的仿真联调反馈。本次源码整理的检查范围单独记在 [11 月 1 日进度](upper_computer/docs/mecanum-progress-2026-11-01.md)。

实车先确认鲁班猫型号、系统和 ROS 2 版本，以及电机驱动与通信接口；随后核对四轮编号、转向、编码器符号，完成速度指令、轮速反馈和停车测试。单轮闭环稳定后，再验证整车前进、横移和旋转。仿真尺寸需要按实物重新测量，不能直接当作标定值。

仓库此前记录的 ELF3588 / Ubuntu 22.04 环境作为历史记录保留，不代表这次鲁班猫的部署环境。原差速导航和雷达建图成果也不代表麦克纳姆底盘已经完成导航验收。

## 已有工作

原差速仿真包含 AMCL、Nav2、多点巡航、语音和拍照，使用方法见 [上位机说明](upper_computer/README.md)。2026-10-23 已在电脑端完成 Camsense X1 真实雷达建图基本测试，见 [代码与现场记录](upper_computer/docs/camsense-x1-mapping-2026-10-23.md)。

## 原双轮底盘

使用 STM32 标准外设库、TB6612 双电机驱动和 HWT101 陀螺仪。电机1为左轮，电机2为右轮，两轮正向均为前进；从上方看，航向角逆时针增加、顺时针减小。

| 功能 | 引脚 / 外设 |
| --- | --- |
| 蓝牙串口 | USART1，PA9 TX / PA10 RX，115200，8N1 |
| HWT101 | USART2，PA2 TX / PA3 RX，115200 |
| 左轮编码器 | TIM2，PA0 / PA1 |
| 右轮编码器 | TIM3，PA6 / PA7 |
| 左轮 / 右轮 PWM | TIM4，PB6 / PB7，20 kHz |
| 左轮方向 | PB12 / PB13 |
| 右轮方向 | PB14 / PB15 |
| TB6612 STBY | 外部拉高，不占用 PB5 |
| OLED | PB8 / PB9，保留驱动，当前主程序未使用 |

### 蓝牙遥控

当前主程序运行双摇杆遥控，串口只接收，不输出调试数据。

```text
[j,LX,LY,RX,RY]
[j, 0, 100, 0, 0]
```

也接受 `joystick` 前缀。四个轴取值为 -100～100，允许字段间带空格。左摇杆 Y 控制前进后退，右摇杆 X 控制左右转向，另外两轴不参与控制。直行时保持航向，转向时更新目标航向；两轴回中停车。

APP 在摇杆变化时发送数据。底盘保持最后一条有效指令，推住不动无需重复发包。`REMOTE_TIMEOUT_MS` 默认设为 0，因此蓝牙断开不会因收包超时自动停车。角度数据异常、串口接收错误和电机控制超时仍会停车，故障恢复后需要摇杆回中。

### 控制参数

编码器按每轮一圈 2500 计数、轮周长 20 cm 换算，电机采样周期为 20 ms：

```text
车轮 RPM = 每周期编码增量 × 1.2
轮缘速度（m/s）= 每周期编码增量 × 0.004
```

两台电机使用独立 PID 状态，共用 `MotorPID.h` 中的增益：KP=8500、KI=2000、KD=0。PID 输入单位为 m/s，输出为千分制 PWM，参数没有额外乘以 100。

遥控默认最高直行速度 0.4 m/s，转向目标变化速度 45°/s，起步 PWM 下限 20%。这些参数在 `RemoteControl.h`，角度环增益在 `HeadingControl.h`。航向保持用于减小车头偏转，不能修正横向位移或陀螺仪长期漂移。

### 编译与文档

用 Keil µVision 打开 [project.uvprojx](chassis/project.uvprojx)，使用 ARM Compiler 5 和 STM32F1 设备包编译、下载。生成的目标文件和本机调试配置不入库。

- [10月19日开发记录](chassis/docs/progress-2026-10-19.md)
- [10月18日开发记录](chassis/docs/progress-2026-10-18.md)
- [蓝牙遥控逻辑](chassis/docs/RemoteControl.md)
- [双电机 PID 接口](chassis/docs/MotorControl.md)
- [独立航向控制接口](chassis/docs/HeadingControl.md)
- [HWT101 驱动](chassis/docs/HWT101.md)
- [PWM 阶梯扫描](chassis/docs/MotorScan.md)
- [电机1标定结果](data/motor1_calibration.md)

`chassis/Tests/` 保存主机侧测试，使用 GCC 验证解析和控制算法；电机输出、接线和实际运动效果仍需实车验证。阶梯扫描和独立定角接口保留，当前主程序不同时运行这些测试模式。
