# 验证记录（2026-10-30）

## 已执行

- 当前执行主机：macOS ARM64，Python 3.9；不是鲁班猫/Ubuntu 实车环境。
- 新包 pytest：10 项通过，包含200组确定性随机正逆解、基向量、非法输入/几何、限幅、Omni速度链配置、实际 Xacro 展开及四轮接口检查。
- 原底盘6组主机 C 测试通过：MotorPID、HWT101 parser、MotorScan、HeadingControl、RemoteControl、RemoteSerial。新增 `scripts/test_legacy_chassis.sh` 可复现；原固件不改动。
- Python Xacro 来自独立测试环境的 PyPI 2.1.1，不等同于目标 Jazzy apt 版本；出现一个第三方 Python 3.9 DeprecationWarning。
- 本地执行 `verify_jazzy.sh` 在 `/opt/ros/jazzy/setup.bash` 缺失处失败：本机无 ROS，不能报告本地 colcon 或 Gazebo 通过。

## Ubuntu CI 已观察结果

[4a38bee 的 CI 日志](https://github.com/KiloMing/automated-inspection-vehicle/actions/runs/41028471548) 确认 Ubuntu 24.04 amd64 / ROS 2 Jazzy 下新包 colcon build 成功、8项 colcon test 全通过、三个 launch 参数解析及安装后的工具 dry run 成功。后续新增2项 SDF 解析回归测试。本地 Python wheel 构建也通过。

首轮依赖声明错误已修复。该轮 CI 最后在 SDF XML 检查器失败：Gazebo 输出含 gz:expressed_in 但无 xmlns 声明。现使用外层命名空间绑定解析，并断言原属性确实存在；未向输出补造摩擦坐标系。最终检查状态以最新提交 CI 为准。

## 自动化与待验证项

`.github/workflows/mecanum-jazzy.yml` 在 Ubuntu 24.04 / ROS Jazzy 容器安装依赖，构建新包、运行 colcon test、检查 launch 参数和安装后工具。CI 的实际状态以对应 commit 的 GitHub Actions 为准，添加工作流不等于构建已经成功。

以下尚无验证结果：Gazebo 接触模型实际横移、gz_ros2_control 运行时加载/超时、完整 Nav2 导航、完整旧工作空间 colcon 构建、RF2O 重建配置、鲁班猫板端构建及性能、STM32 四轮闭环/通信、实车里程计/建图回归/导航/急停。`motion_smoke --execute` 会真实发送运动命令，只在独立仿真域运行；本次没有连接硬件。

## 保留性

历史 `chassis/`、`upper_computer/real_lidar/`、`mybot_description`、已有测试记录和照片保持不变。README 的当前目标平台更新为野火鲁班猫5 RK3588S；历史记录仍明确保留原环境描述。
