# 自动巡检小车

项目分为 STM32 底盘和 ROS 2 上位机。底盘负责电机、编码器、航向和蓝牙遥控，上位机负责仿真、导航和巡检任务。

截至 2026-10-19，底盘已完成双电机速度闭环、HWT101 航向闭环和蓝牙双摇杆控制。实车已确认航向测试通过、蓝牙遥控移动正常。上位机与实车底盘的通信接口还未接通。

## 目录

| 目录 | 内容 |
| --- | --- |
| `chassis/` | STM32F103C8 底盘固件，Keil 工程 |
| `upper_computer/` | ROS 2、Gazebo、Nav2 上位机工程 |
| `data/` | 两台电机的 VOFA 原始数据、电机1标定结果与分析脚本 |

## 底盘

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

## 上位机

实车上位机使用 ELF3588 开发板。截至 2026-10-20，已安装 Ubuntu 22.04 和 ROS 2，并配置 SSH。ROS 2 的具体发行版待确认，尚未记录板端工程编译和底盘通信联调结果。

原仿真工程在 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Harmonic 下开发，包含差速小车模型、AMCL 定位、Nav2 导航和多点巡航；到达巡检点后语音播报并保存相机画面。该环境与实车开发板不同，板端部署时需核对依赖版本。

```bash
git clone git@github.com:KiloMing/automated-inspection-vehicle.git
cd automated-inspection-vehicle/upper_computer/chapt6_ws
colcon build
```

原仿真环境的依赖安装和启动方法见 [上位机 README](upper_computer/README.md)。其中 Jazzy 和 Gazebo Harmonic 的安装命令不作为 ELF3588 板端的安装说明。

下一步接通上位机与底盘的速度指令、轮速和航向反馈，再进行实车里程计与导航联调。
