#!/usr/bin/env bash
set -eo pipefail
# Run from upper_computer/chapt6_ws. Fails if the required runtime is absent.
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mecanum_bringup --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select mecanum_bringup --event-handlers console_direct+
colcon test-result --verbose
ros2 launch mecanum_bringup simulation.launch.py --show-args
ros2 launch mecanum_bringup navigation.launch.py --show-args
ros2 launch mecanum_bringup real_mapping.launch.py --show-args
ros2 run mecanum_bringup motion_smoke
python3 "$(ros2 pkg prefix --share mecanum_bringup)/scripts/check_sdf.py"
