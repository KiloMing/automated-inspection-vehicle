# F407 麦克纳姆巡检平台

当前开发主线：STM32F407 底盘 + 鲁班猫 RK3588 上位机 + ROS 2 / Nav2。

## 当前状态

2026-11-04 用户提供 `mecanumws2`，确认为最新可使用的上位机代码，已原样同步至 `ros2_ws/`。2026-11-03 的导航偏差作为历史记录保留。本次未代为运行代码，具体实测指标以开发者后续记录为准。

## 目录

- [firmware](firmware/README.md)：四轮 PID、麦轮运动学、编码器、HWT101 相关驱动、串口协议；原硬件设计资料仍在 firmware/docs/hardware。
- [ros2_ws](ros2_ws/)：实车底盘桥、雷达驱动、建图导航 launch、配置与地图。
- [hardware](hardware/README.md)：新增硬件参数来源、鲁班猫说明与照片。
- [最新每日记录](docs/daily/2026-11-04.md)：实验进展及待解决问题。

## 工作区现状

`chassis_bridge` 接收 `/cmd_vel`，发送 ASCII 速度命令，解析 CRC 位姿帧，发布 `/odom`、`/chassis/pose` 及 TF。`sllidar_ros2` 内包含 `map_start.launch.py` 和 `start_nv2.launch.py`。`config/nav2_real.yaml` 当前启用 Navfn 全局规划器和 Regulated Pure Pursuit 控制器。新增 `mecanum_straight_planner` 包，其配置仍被注释，未默认启用。其他参数文件不能直接互换使用。默认地图更新为 `maps/my_map.yaml`，配套图片为 `my_map.pgm`。

默认设备为底盘 `/dev/ttyUSB1`、雷达 `/dev/ttyUSB0`，实际设备号需核实。默认配置与地图路径仍指向 `/home/cat/mecanumws/`。`chassis_cmd_topic` 尚未真正接入重映射，不能依赖其默认值隔离真实电机。

## 目标架构

后续逐步增加 `mecanum_description`、`mecanum_bringup`、`mecanum_navigation`、`mecanum_simulation` 和 `inspection_manager`。这些是规划包，尚未在本次提交中实现。仿真与实车计划共用上层导航与巡检逻辑，只替换底盘和传感器来源。

当前优先事项是运动隔离、故障处置和标定，再进行导航参数优化。先独立测试前进、横移、旋转，再比较实际位移和里程计，然后核对 TF 与定位，最后分析规划和路径跟踪。不能仅凭导航失败判断官方算法不适用。

## 构建与验证状态

按用户要求，本次未运行项目代码、构建或测试。Keil 工程与包内相对目录保持原有结构；部署和构建是否成功需要在目标环境另行记录。ROS 2 版本、Ubuntu 版本与板型仍待补充。
