# 麦克纳姆迁移设计

用户授权目标：独立麦克纳姆方案、野火鲁班猫 5 RK3588S / Ubuntu 24.04 / ROS 2 Jazzy、STM32 四电机，保留原差速及 Camsense X1 + RF2O + SLAM Toolbox 历史成果，分支 PR 审阅。

采用并行新增 `mecanum_bringup` 包：旧工程不改动。原地替换会破坏对照；复制整个工作空间会重复维护，因此不采用。仿真使用 Harmonic、gz_ros2_control 和官方 mecanum_drive_controller；Nav2 使用 MPPI Omni、AMCL Omni，统一 base_link。模型尺寸是可配置示例，不能冒充实测。

坐标：x 前、y 左、z 上，轮序 FL FR RL RR，X 型滚子。几何共用 YAML，半轴距 lx、半轮距 ly、轮半径 r。速度为 m/s、rad/s，轮速 rad/s。仿真控制器唯一发布 odom→base_link；实车建图由 RF2O 唯一发布，SLAM 与 AMCL 不同时发布 map→odom。

实车只有双电机固件和雷达替换源码存档，尚无可用四电机通信契约。新增上位机配置及验证工具，不虚构引脚、串口帧或已完成四电机驱动；记录 STM32 接口验收要求。实车 launch 不自动发送速度。

六阶段路线原文无法读取，按离线运动学、仿真底盘、仿真导航、架空四轮、建图回归、实车导航重建并显著标注。macOS 本地执行数学、结构和原 C 测试；Ubuntu CI 执行 colcon/Xacro/launch 检查，实际仿真运动及板端性能独立验收。
