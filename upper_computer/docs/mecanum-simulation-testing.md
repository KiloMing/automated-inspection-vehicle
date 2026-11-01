# 麦克纳姆仿真测试

工作区：`upper_computer/mecanum_ws`。以下命令用于 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic，不是鲁班猫实车部署说明。测试时不要同时启动原差速工作区。

## 安装与构建

先安装 ROS 2 Jazzy，并完成 rosdep 初始化。然后安装仿真和控制依赖：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-ros2controlcli \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2 \
  ros-jazzy-tf2-ros liburdfdom-tools
```

从仓库根目录进入工作区：

```bash
cd upper_computer/mecanum_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
xacro src/mecanum_description/urdf/mecanum_robot.urdf.xacro > /tmp/mecanum_robot.urdf
check_urdf /tmp/mecanum_robot.urdf
```

`colcon test` 的包检查不等于运动验收。下面的步骤需要另外运行。

## 启动与接口

```bash
ros2 launch mecanum_bringup sim.launch.py
```

启动文件加载世界、发布机器人描述、生成模型、桥接 `/clock`，随后启动两个控制器。每个新终端都需要加载 `/opt/ros/jazzy/setup.bash` 和当前工作区的 `install/setup.bash`。

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 param get /controller_manager update_rate
ros2 topic info /mecanum_drive_controller/reference
ros2 topic echo /clock --once
```

预期 `joint_state_broadcaster` 和 `mecanum_drive_controller` 都为 `active`；四轮 velocity 命令接口被占用，position/velocity 状态接口可用；控制频率参数为 100。命令话题类型应为 `geometry_msgs/msg/TwistStamped`。

## 前进、横移、旋转

一次只运行一条指令，观察数秒后按 Ctrl-C，再发送停车指令。`x` 向前，`y` 向左，正 `angular.z` 为逆时针。发布端使用仿真时钟，`stamp: now` 为每条消息填入当前时间。

前进：

```bash
ros2 topic pub -r 10 /mecanum_drive_controller/reference geometry_msgs/msg/TwistStamped \
  "{header: {stamp: now, frame_id: base_link}, twist: {linear: {x: 0.1}}}" \
  --use-sim-time
```

向左横移：

```bash
ros2 topic pub -r 10 /mecanum_drive_controller/reference geometry_msgs/msg/TwistStamped \
  "{header: {stamp: now, frame_id: base_link}, twist: {linear: {y: 0.1}}}" \
  --use-sim-time
```

原地逆时针旋转：

```bash
ros2 topic pub -r 10 /mecanum_drive_controller/reference geometry_msgs/msg/TwistStamped \
  "{header: {stamp: now, frame_id: base_link}, twist: {angular: {z: 0.3}}}" \
  --use-sim-time
```

停车：

```bash
ros2 topic pub --once /mecanum_drive_controller/reference geometry_msgs/msg/TwistStamped \
  "{header: {stamp: now, frame_id: base_link}, twist: {linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}}" \
  --use-sim-time
```

配置中的 `reference_timeout` 为 1.0 秒。停止发布后，应在仿真时钟继续运行的条件下检查超时停车；这项检查需要单独记录，不能用“运动正常”代替。不要把仿真的超时设置视为实体底盘已经具备失联停车。

## 里程计与 TF

```bash
ros2 topic echo /mecanum_drive_controller/odometry --once
ros2 run tf2_ros tf2_echo odom base_link --ros-args -p use_sim_time:=true
ros2 topic info /tf --verbose
```

预期里程计的 frame 为 `odom`，child frame 为 `base_link`；运动时位姿随之变化。TF 查询应持续返回变换。启动初期短暂等待可以接受，持续找不到 frame 则需要排查。

TF 修复位于 `mecanum_description/urdf/ros2_control.xacro` 的 Gazebo 插件内部：

```xml
<ros>
  <remapping>/mecanum_drive_controller/tf_odometry:=/tf</remapping>
</ros>
```

控制器管理器由 Gazebo 插件创建，只给 spawner 或其他独立节点加 remap 不能替代这里的配置。修改后重建、重新加载环境，并退出旧仿真后重新启动。不要再启动另一个节点重复发布 `odom → base_link`。

## 如何记录结果

记录日期、提交号、系统和包版本，以及控制器状态、运动现象、里程计和 TF 输出。分别记下前进、横移、旋转、主动停车、超时停车是否通过。

里程计精度需要将轮式里程计与 Gazebo 实际位姿做定量比较，包括直线距离、横移距离、旋转角和累计漂移。当前球形接触与方向性摩擦是简化模型，不能凭“能横移”判断精度达标。SLAM Toolbox、Nav2 全向导航和实体底盘另行验收。

接口参考：[MecanumDriveController](https://control.ros.org/jazzy/doc/ros2_controllers/mecanum_drive_controller/doc/userdoc.html)、[gz_ros2_control](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)。
