# automated-inspection-vehicle

基于 ROS 2、Gazebo 和 Nav2 实现的自动巡检小车仿真项目。

目前已经实现从仿真环境启动、机器人控制、AMCL 定位到多点自动巡航的完整流程。小车到达巡检点后会进行语音播报，并保存当前相机画面。

项目目前主要在 Ubuntu 24.04 ARM64 + ROS 2 Jazzy 环境下开发和测试。

## 已实现功能

- 二轮差速小车 URDF / Xacro 模型
- Gazebo 仿真环境
- `ros2_control` 差速驱动控制
- LaserScan 激光雷达
- RGB Camera
- Gazebo / ROS 2 Topic Bridge
- 静态地图加载
- AMCL 定位
- Nav2 路径规划与导航
- `FollowWaypoints` 多点巡航
- 自动发布初始位姿
- 到达巡检点后语音播报
- 到达巡检点后保存相机图片
- Launch 文件统一启动整套系统

目前默认巡检点为：

```text
A -> B -> C -> D
```

巡航程序默认执行 3 圈。

---

## 环境

当前开发环境：

```text
Ubuntu 24.04
ROS 2 Jazzy
Gazebo Harmonic
C++17
```

主要依赖：

```text
Nav2
ros2_control
gz_ros2_control
ros_gz_sim
ros_gz_bridge
robot_state_publisher
tf2
OpenCV
cv_bridge
espeak-ng
```

安装 Nav2：

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

安装 Gazebo ROS 相关组件：

```bash
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-gz-ros2-control
```

安装语音工具：

```bash
sudo apt install espeak-ng
```

---

## 项目结构

```text
automated-inspection-vehicle/
└── chapt6_ws/
    ├── maps/
    │   ├── test_map.pgm
    │   └── test_map.yaml
    │
    ├── photo/
    │
    └── src/
        ├── autopatrol_interface/
        │   └── srv/
        │       └── SpeechText.srv
        │
        ├── autopatrol_speaker/
        │   └── src/
        │       └── speaker.cpp
        │
        └── mybot_description/
            ├── config/
            │   ├── nav2_params.yaml
            │   └── ros2_control.yaml
            │
            ├── launch/
            │   ├── gz_sim_control.launch.py
            │   ├── nav2_bringup.launch.py
            │   ├── autopatrol.launch.py
            │   └── start.launch.py
            │
            ├── src/
            │   ├── local_init.cpp
            │   ├── tf_listen.cpp
            │   ├── action_client.cpp
            │   ├── follow_way_point.cpp
            │   └── PatrolController.cpp
            │
            ├── urdf/
            └── worlds/
                └── testworld.sdf
```

三个 ROS 2 package 的作用：

### `mybot_description`

项目主体。

包含机器人模型、Gazebo world、控制器配置、Nav2 参数、Launch 文件以及巡航控制程序。

### `autopatrol_interface`

定义语音服务接口：

```text
string text
---
bool result
```

### `autopatrol_speaker`

提供 `/speech_text` 服务，收到文本后使用 `espeak-ng` 播放语音。

---

## 编译

克隆仓库：

```bash
git clone https://github.com/KiloMing/automated-inspection-vehicle.git
cd automated-inspection-vehicle/chapt6_ws
```

编译：

```bash
colcon build
```

加载环境：

```bash
source install/setup.bash
```

如果重新打开终端，需要重新执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

---

## 一键启动

项目已经把 Gazebo、Nav2 和自动巡航分别拆成独立 Launch，并通过 `start.launch.py` 统一启动。

运行：

```bash
ros2 launch mybot_description start.launch.py
```

启动顺序：

```text
start.launch.py
│
├── gz_sim_control.launch.py
│
├── 10 s
│   └── nav2_bringup.launch.py
│
└── 25 s
    └── autopatrol.launch.py
```

这样不需要再分别打开多个终端启动 Gazebo、Nav2、RViz、初始化节点和巡航节点。

---

## Gazebo 部分

`gz_sim_control.launch.py` 主要负责：

```text
Gazebo Server / GUI
        |
robot_state_publisher
        |
    Spawn Robot
        |
 gz_ros2_control
        |
controller_manager
        |
+------------------------+
|                        |
joint_state          diff_drive
broadcaster          controller
```

同时建立以下 Gazebo -> ROS 2 Bridge：

```text
/clock
/scan
/camera/image
```

项目针对当前虚拟机环境启用了软件渲染：

```text
LIBGL_ALWAYS_SOFTWARE=1
LIBGL_DRI3_DISABLE=1
```

Gazebo Transport 使用：

```text
GZ_IP=127.0.0.1
```

---

## Nav2

`nav2_bringup.launch.py` 启动 Nav2 和 RViz，并加载静态地图与导航参数。

当前使用地图：

```text
chapt6_ws/maps/test_map.yaml
```

Nav2 参数：

```text
chapt6_ws/src/mybot_description/config/nav2_params.yaml
```

RViz 使用 Nav2 默认配置：

```text
nav2_default_view.rviz
```

---

## 自动定位

`local_init.cpp` 启动后会向：

```text
/initialpose
```

发布一次初始位姿。

当前初始位置：

```text
x   = -1.0
y   = 3.66
yaw = 1.0503 rad
```

消息发布完成后 Timer 会停止，不会在巡航过程中反复重置 AMCL。

---

## 自动巡航

主要逻辑位于：

```text
chapt6_ws/src/mybot_description/src/PatrolController.cpp
```

程序使用 Nav2：

```text
/follow_waypoints
```

完成多点巡航。

当前巡检点：

```text
A  (-0.507, -0.09)
B  (-3.14,   3.11)
C  (-0.497,  6.65)
D  (1.616,   3.33)
```

巡航次数目前设置为：

```cpp
goal_msg.number_of_loops = 3;
```

整体流程：

```text
FollowWaypoints
      |
      v
到达巡检点
   /      \
  /        \
语音播报    保存图片
  |           |
  v           v
/speech_text  /camera/image
  |           |
espeak-ng   cv_bridge
              |
            OpenCV
              |
           A.jpg
           B.jpg
           C.jpg
           D.jpg
```

---

## 相机拍照

巡航控制器订阅：

```text
/camera/image
```

并缓存最新一帧图像。

到达 A、B、C、D 点时，通过 `cv_bridge` 转为 OpenCV 图像并保存。

当前代码中的保存目录为：

```text
/home/kiloming/my_ros/chapt6_ws/photo/
```

生成文件：

```text
A.jpg
B.jpg
C.jpg
D.jpg
```

如果项目放在其他路径，需要修改 `PatrolController.cpp` 中的图片保存路径。

---

## 语音播报

巡航控制器到达巡检点后调用：

```text
/speech_text
```

例如：

```text
Reached A
Reached B
Reached C
Reached D
```

`speaker` 节点收到文本后调用 `espeak-ng` 进行语音播放。

可以单独测试：

```bash
ros2 service call /speech_text \
autopatrol_interface/srv/SpeechText \
"{text: 'hello'}"
```

---

## 单独启动各模块

调试时也可以不使用 `start.launch.py`。

Gazebo：

```bash
ros2 launch mybot_description gz_sim_control.launch.py
```

Nav2：

```bash
ros2 launch mybot_description nav2_bringup.launch.py
```

自动初始化、语音和巡航：

```bash
ros2 launch mybot_description autopatrol.launch.py
```

这种方式比较适合排查某一部分的问题。

---

## 常用检查命令

检查控制器：

```bash
ros2 control list_controllers
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

检查雷达：

```bash
ros2 topic echo /scan --once
```

检查相机：

```bash
ros2 topic echo /camera/image --once
```

检查里程计：

```bash
ros2 topic echo /odom --once
```

检查 Nav2 Action：

```bash
ros2 action list
```

检查语音服务：

```bash
ros2 service list | grep speech
```

---

## 当前注意事项

项目目前仍有部分路径写成了本机绝对路径。

`nav2_bringup.launch.py` 中：

```text
/home/kiloming/my_ros/chapt6_ws/maps/test_map.yaml
/home/kiloming/my_ros/chapt6_ws/src/mybot_description/config/nav2_params.yaml
```

`PatrolController.cpp` 中：

```text
/home/kiloming/my_ros/chapt6_ws/photo/
```

如果用户名或工作区位置不同，需要先修改这些路径。

后续会把这些路径改成 package share 路径或 ROS 2 参数，减少对固定目录的依赖。

---

## 后续

接下来准备继续做：

- 去掉代码中的绝对路径
- 将巡检点改成参数或配置文件
- 图片文件名加入时间戳和巡检轮次
- 增加巡检异常检测
- 增加相机图像处理
- 完善导航失败、服务不可用等异常处理
- 从 Gazebo 仿真逐步迁移到真实小车
