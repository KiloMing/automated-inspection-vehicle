# F103 二轮差速系统

本目录归档原 F103 底盘、二轮差速仿真、巡检示例及旧电机标定资料。当前主要用于历史参考，后续开发主线位于相邻的 `mecanum_f407`。

## 内容

- [firmware](firmware/)：原 chassis，STM32F103 固件、Tests 与调试文档。
- [ros2_ws](ros2_ws/)：原 chapt6_ws，差速模型、Gazebo、Nav2 和巡检示例。
- [calibration](calibration/)：原 data 中的电机采样、分析脚本与标定记录。
- [legacy_upper_computer](legacy_upper_computer/)：旧上位机说明、Camsense 实验及照片，保留历史归档关系，不代表当前麦轮硬件配置。

## 环境与边界

历史仿真环境为 Ubuntu 24.04 ARM64、ROS 2 Jazzy。旧文档中还包含 ELF3588 等历史设备描述，应以对应实验日期理解。当前麦轮主机已由用户确认为鲁班猫 RK3588。

差速模型不能验证麦克纳姆横移。旧电机参数不能直接套用到四轮系统。此分类不表示 F103 与仿真已完成实车全链路验证。

本次仅移动目录和补充文档，未运行代码。旧 launch 中的绝对路径仍需按实际部署位置调整。详细历史功能见[原上位机说明](legacy_upper_computer/README.md)。
