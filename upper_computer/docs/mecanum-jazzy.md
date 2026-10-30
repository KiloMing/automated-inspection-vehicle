# 麦克纳姆方案：野火鲁班猫 5 RK3588S / Jazzy

## 当前范围与平台

新方案实车上位机明确为 **野火鲁班猫 5 RK3588S，Ubuntu 24.04 ARM64，ROS 2 Jazzy**；STM32 负责四轮电机闭环、编码器采集及失联停车。Gazebo Harmonic 可先运行在性能足够的 Ubuntu 24.04 电脑，不要求板端承担仿真渲染。

这是一套新增的软件迁移基线，不是已验收的四轮实车。仓库原 STM32F103C8/TB6612 工程目前只有两个电机通道；四电机驱动器、定时器/编码器资源、引脚和串口协议尚需按实物确定。不要把旧双电机固件直接当四轮固件烧录。现有 `chassis/`、`mybot_description`、巡航程序、地图、雷达源码和历史测试证据均保留。

- 新包：`upper_computer/chapt6_ws/src/mecanum_bringup`
- 差速对照：`ros2 launch mybot_description start.launch.py`（保留原路径与历史限制）
- 旧建图证据：[2026-10-23 测试记录](camsense-x1-mapping-2026-10-23.md)
- 新六阶段验收：[测试路线](mecanum-six-stage-test-route.md)
- 实际执行记录：[验证状态](mecanum-validation.md)

历史文档中的 ELF3588 / Ubuntu 22.04 仅描述当时状态；新目标以本文件为准。此处不声称已在鲁班猫上安装或运行成功。

## 坐标、轮序与尺寸

机身 x 向前、y 向左、z 向上；绕 z 逆时针为正。X 型滚子布局，所有关节正转定义为向前滚动，轮序固定 **FL（左前）、FR（右前）、RL（左后）、RR（右后）**。轮轴全部沿 +y；右侧电机实际安装若反向，由 STM32 单独的电机/编码器符号映射纠正。

令 `k=lx+ly`，`lx` 为半轴距、`ly` 为半轮距，`r` 为轮半径：

```text
wFL = (vx - vy - k*wz) / r
wFR = (vx + vy + k*wz) / r
wRL = (vx + vy - k*wz) / r
wRR = (vx - vy + k*wz) / r

vx = r*(wFL+wFR+wRL+wRR)/4
vy = r*(-wFL+wFR+wRL-wRR)/4
wz = r*(-wFL+wFR-wRL+wRR)/(4*k)
```

机身速度单位 m/s、rad/s；轮速 rad/s，RPM = rad/s × 60/(2π)。限幅应对四轮等比例缩放，不逐轮独立裁剪，以保持运动方向。纯 Python 模块提供正逆解和限幅参考，它不直接驱动 STM32。

`config/geometry.yaml` 是模型与仿真控制器共用的尺寸源；默认 r=0.05 m、lx=0.15 m、ly=0.125 m，**均为示例值**。同文件包含雷达外参。修改后还必须测量并更新 `nav2_omni.yaml` 两个 costmap 的 footprint（默认包络 ±0.21 × ±0.19 m，覆盖球形碰撞近似）、速度/加速度及负载惯量。自定义几何通过 `geometry_file:=/absolute/geometry.yaml` 传入。

模型以球形接触及机身坐标系下 ±45° 各向异性摩擦近似滚子；不是完整滚子接触模型。必须以 Gazebo ground truth 检查横移，不能只看控制器里程计。单独展开模型默认为实车展示模式，不会加载仿真驱动；simulation launch 生成临时控制器文件并在退出时清理。

## 安装与构建（Ubuntu 24.04 / Jazzy）

先安装与板卡适配的 Ubuntu 24.04 ARM64 系统及 ROS 2 Jazzy，确认串口、USB 和供电稳定；本仓库不提供或替换板卡镜像。

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-pytest \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-mecanum-drive-controller ros-jazzy-joint-state-broadcaster \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
# 首次使用 rosdep 才运行 sudo rosdep init
rosdep update --rosdistro jazzy
cd automated-inspection-vehicle/upper_computer/chapt6_ws
rosdep install --from-paths src/mecanum_bringup --ignore-src --rosdistro jazzy -y
bash src/mecanum_bringup/scripts/verify_jazzy.sh
```

新包可单独构建，不依赖旧工程中本机绝对路径或语音/相机节点。全工作空间验证另运行 `colcon build` / `colcon test`，不能把单包通过当作全部历史包通过。

## 仿真底盘

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mecanum_bringup simulation.launch.py gui:=true
# 另一个同环境终端
ros2 control list_controllers
ros2 topic echo /odom --once
ros2 run mecanum_bringup motion_smoke       # 默认只打印测试计划
```

无界面仿真验收（会在独立 ROS domain 87 发送运动指令，不接实车）：

```bash
bash src/mecanum_bringup/scripts/simulation_acceptance.sh
```

该脚本启动仿真并执行前进、左横移、斜移、原地旋转和失联停车测试。它需要 `/clock`、`/ground_truth`、`/odom` 和控制器订阅者，等待超时或反馈过期会失败。检查独立 ground truth 和轮式里程计的方向/位移，而不是只测试命令发布成功。每段 2 秒，线速度 0.1 m/s、角速度 0.3 rad/s；默认误差门限 max(0.06, 期望量的35%)，仅为仿真冒烟门限。仿真日志保存在当前目录 `simulation.log`。无头 GPU 雷达依赖可用渲染环境，必要时在 Xvfb 或有图形会话的主机运行；不能把雷达缺失忽略后进行导航验收。

## 导航与话题契约

```text
Nav2 controller / behavior (TwistStamped)
  → /cmd_vel_nav
  → velocity_smoother（x/y/z 都限速）
  → /cmd_vel_smoothed
  → collision_monitor
  → /mecanum_drive_controller/reference (TwistStamped)
  → 四轮 velocity command interfaces (rad/s)
```

控制器超时 0.3 s。AMCL 为 `nav2_amcl::OmniMotionModel`；MPPI 为 `Omni`，保留 y 方向采样与速度，移除前进偏好、关闭路径角度惩罚，避免强迫先转头再横移。所有速度节点使用带时间戳消息。默认最大 x/y 为 0.3 m/s、角速度 0.6 rad/s；这是待调参起点。RK3588S 的 MPPI 实时性、batch_size=1000 的开销必须实测。

使用与当前世界对应的地图，不能直接把旧差速测试地图当成新验收世界地图：

```bash
ros2 launch mecanum_bringup navigation.launch.py \
  use_sim_time:=true map:=/absolute/path/to/matching_map.yaml
```

需要仿真建图时，在 simulation launch 后单独启动 SLAM Toolbox，使用 `config/slam.yaml` 并将 `use_sim_time` 改为 true；**不要启动 RF2O**，由控制器提供 odom。例如：

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:=$(ros2 pkg prefix --share mecanum_bringup)/config/slam.yaml
ros2 run nav2_map_server map_saver_cli -f /absolute/path/to/matching_map
```

保存后停止 SLAM，再启动 AMCL/Nav2。通过 RViz 设置初始位姿，再发送导航目标；没有硬编码旧初始位姿或自动巡航点。原语音/拍照巡航仍保留在旧包，尚未对新模型联调。

## 保留实车 Camsense X1 + RF2O + SLAM Toolbox 链路

原始档案不具备独立包的全部构建文件，已完成测试的现场 `ros_ladar_sv`、serial-ros2、RF2O 工作空间需继续保留。先在板端加载这些**现有环境**，确认以下包可查到：

```bash
ros2 pkg prefix ros_ladar_sv
ros2 pkg prefix rf2o_laser_odometry
ros2 run ros_ladar_sv camsense_x1_node
```

串口路径/115200 参数沿用现有驱动的实际参数名，建议固定 `/dev/serial/by-id/...`；不要照搬历史 `/dev/ttyUSB1`。确认 `/scan` 为 `laser_link`、频率/时间戳正确后再启动：

```bash
ros2 launch mecanum_bringup real_mapping.launch.py geometry_file:=/absolute/measured_geometry.yaml
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic hz /odom_rf2o
ros2 run nav2_map_server map_saver_cli -f /absolute/path/to/real_map
```

这个 launch 是依据历史记录**新重建的待实测配置**，不是恢复出的原现场参数；需对所安装 RF2O 版本核对参数。它不启动串口驱动、不连接电机、不发送速度。不要同时启动现场旧 RF2O、静态雷达 TF 或 SLAM launch，以免重复节点/变换。

| 场景 | map→odom | odom→base_link | base_link→laser_link |
|---|---|---|---|
| 仿真导航 | AMCL | mecanum_drive_controller | robot_state_publisher |
| 仿真建图 | SLAM Toolbox | mecanum_drive_controller | robot_state_publisher |
| 实车 RF2O 建图 | SLAM Toolbox | RF2O | robot_state_publisher（实测外参） |
| 后续实车导航 | AMCL | 选定且唯一的轮式/融合里程计，或已验收 RF2O | robot_state_publisher |

`/odom_rf2o` 不等于 `/odom`。若导航使用 RF2O，另建实车参数文件，将 bt_navigator 和 velocity_smoother 的 odom_topic 改为 `/odom_rf2o`，停用轮式 odom TF；若采用融合里程计则各原始源停止 TF，仅融合节点发布。AMCL 与 SLAM 不可同时运行。

## STM32 四轮接口与落地边界

ROS 侧推荐以与仿真相同的四个 joint 和 velocity state/command 接口实现独立 ros2_control 硬件插件；也可选板端机身速度串口桥，但两者不能同时做逆运动学或同时驱动四轮。本次**未实现未定义协议的串口桥/硬件插件，也未改动旧双电机固件**。

待双方确定的最小契约：

- FL/FR/RL/RR 固定轮序，四路独立 PID 与编码器；目标/反馈都用带符号 rad/s，明确减速比和每转计数。
- 帧包含版本、序号、长度、命令/反馈类型、校验和；反馈包含四轮实测速率、采样时间、故障状态，禁用时目标为零。
- MCU 单调时钟独立看门狗（建议 300 ms 起点）、畸形帧不刷新有效指令时间；失联/重启默认停车，急停优先于 ROS 命令，恢复需显式使能。
- 四轮限速等比例缩放、加减速限制；电机和编码器符号分别标定。上位机发送超时、掉电和串口拔插都要做架空测试。
- 在已确认的具体 STM32 型号、驱动器与引脚资源表上实现；现有 F103 双通道资源不能推断出另两路可用。

未完成该契约与架空验收前，本工程不提供可直接运行的实车导航电机输出。

## 官方接口参考

- [Jazzy mecanum_drive_controller](https://control.ros.org/jazzy/doc/ros2_controllers/mecanum_drive_controller/doc/userdoc.html)
- [Jazzy gz_ros2_control](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
- [Gazebo Harmonic mecanum contact example](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/mecanum_drive.sdf)
- [Nav2 Jazzy navigation launch](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bringup/launch/navigation_launch.py)
- [SLAM Toolbox Jazzy lifecycle launch](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/launch/online_async_launch.py)
