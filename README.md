# 自动巡检小车：上位机与底盘

本仓库包含 ROS 2 自动巡检上位机和 STM32 底盘下位机，分目录维护。

| 目录 | 内容 | 入口 |
|---|---|---|
| chassis/ | STM32F103C8 标准库底盘固件 | [Keil 工程](chassis/project.uvprojx) |
| upper_computer/ | ROS 2、Gazebo、Nav2 自动巡检工程 | [上位机说明](upper_computer/README.md) |

## 底盘

已实现双路编码器、TB6612 双电机驱动、HWT101 串口接收与角度处理。
- [2026-10-18 开发进度](chassis/docs/progress-2026-10-18.md)
- [HWT101 接线、接口和测试](chassis/docs/HWT101.md)
- 主机解析测试：在仓库根目录运行 `./chassis/Tests/run_hwt101.ps1`（Windows PowerShell，需要 gcc）。
- 固件编译：Keil 打开 `chassis/project.uvprojx`，使用 ARM Compiler 5 和 STM32F1 设备包。
- Keil 编译输出及个人调试配置不提交，由开发工具在本地生成。

当前 main.c 中 Motor_Init() 被注释，而 Motor_SetPWM() 调用仍在。
这是现有调试状态，不能直接认为电机已初始化或正在正常运行。
角度 PID、速度 PID、上下位机实车通信尚未实现。

## 上位机

上位机源代码位于 upper_computer/chapt6_ws，原目录结构在 upper_computer 内保留。
原有 ROS 2 工程说明及启动方式见 [上位机 README](upper_computer/README.md)。
原 README 中的仓库路径需要按当前布局进入：

```bash
git clone git@github.com:KiloMing/automated-inspection-vehicle.git
cd automated-inspection-vehicle/upper_computer/chapt6_ws
colcon build
```

依赖环境按上位机 README 配置（Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic）。
2026-10-18 的整理仅验证底盘编译与主机解析测试，未重新运行 ROS 仿真或实车联调。
上位机与底盘尚未建立速度指令、轮速、航向和里程计的正式通信接口。

## 仓库管理

原上位机提交历史保留；2026-10-18 起，仓库根目录同时管理两个工程。
本地更新请在仓库根目录执行 `git pull --ff-only`。
当前 Windows 工作区为 E:/rosfish，.vscode 配置仅本地保留。
由于当前网络 SSH 22 端口超时，本机仅为本仓库配置 SSH 443，远端地址保持不变；
该配置及主机验证文件位于 .git，不随源代码提交，也不修改全局 SSH 设置。
