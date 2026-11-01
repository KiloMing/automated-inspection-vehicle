# 麦克纳姆底盘进度 · 2026-11-01

## 本次保存的内容

将当前 `mecanum_ws/src` 放入 `upper_computer/mecanum_ws/src`，包含 `mecanum_description` 和 `mecanum_bringup`。原差速仿真、STM32 底盘、雷达代码及测试数据保留。

- 四轮半径 0.05 m，前后轮心偏移 ±0.11 m，左右轮心偏移 ±0.15 m。
- 圆柱外观、球形碰撞，使用 `mu=1.0`、`mu2=0.0` 和相对 `base_link` 的方向性摩擦近似麦克纳姆轮。
- 每轮一个 velocity 命令接口，以及 position、velocity 状态接口。
- 控制器管理器更新频率 100 Hz，`reference_timeout=1.0` 秒。
- `MecanumDriveController` 接收 `/mecanum_drive_controller/reference` 上的 `TwistStamped`，发布轮式里程计。
- 在 Gazebo ros2_control 插件内部将 `/mecanum_drive_controller/tf_odometry` 重映射到 `/tf`。

入库时去掉了 CMake 中对空 `rviz` 目录的安装引用，补充 `launch` 运行依赖和包描述，并清理重复注释与行尾空格。没有改动运动参数和控制链路。

## 验证状态

当天仿真联调已确认前进、横移、旋转，且 `odom → base_link` TF 可以查询。这些结果来自操作者反馈，不是本次 macOS 整理环境重新运行得到的结果。

本次源码检查：

| 检查 | 结果 |
| --- | --- |
| XML/Xacro/SDF 语法、Python launch 语法、YAML 解析 | 通过 |
| CMake 安装目录中有可入库文件 | 通过，已移除空 rviz 目录引用 |
| 离线 Xacro 展开 | 通过；使用 xacro 2.1.1，在临时副本中将包查找替换为本地路径 |
| 四轮位置、关节名、命令/状态接口、摩擦方向 | 静态一致性检查通过 |
| 控制器几何参数、100 Hz、TF 插件重映射 | 静态一致性检查通过 |
| CMake 配置尝试 | 未完成：缺少 `ament_cmake` |
| colcon 构建、Gazebo 运动和 TF 复测 | 未执行：本机没有 ROS 2、colcon、Gazebo |

离线展开没有验证 ROS 包索引、插件加载、Gazebo 对摩擦参数的实际处理或运行时行为。复测步骤见 [测试指南](mecanum-simulation-testing.md)。

轮式里程计精度、主动停车与超时停车的独立验收、麦克纳姆底盘上的 SLAM Toolbox、Nav2 全向导航和真实底盘仍待验证。原差速导航与 Camsense X1 建图记录不计入这些验收项。

## 下一步：鲁班猫控制实体底盘

先把鲁班猫和真实麦克纳姆底盘接通，不急着接导航。

1. 确认鲁班猫型号、系统、ROS 2 版本，以及四路电机驱动、编码器和通信方式。原 ELF3588 环境记录仅供参考。
2. 按实车确定前左、前右、后左、后右轮序，检查麦克纳姆轮安装方向、电机正转和编码器正负号；测量轮径与轮距。
3. 确定速度指令和反馈协议，接通四轮目标速度、实际轮速及故障状态。控制周期和断连停车规则需要一起确认。
4. 先完成单轮速度闭环，再做低速前进、横移、旋转、主动停车和失联停车测试。
5. 用实测距离和角度标定里程计，检查实体底盘的 `odom → base_link`，之后再接雷达、SLAM Toolbox 和 Nav2 全向导航。

现有 `chassis/` 是双轮固件，不能直接当作四轮底盘驱动。是否复用其中的编码器、PID、航向和串口实现，要等实体底盘接口确定后再改。
