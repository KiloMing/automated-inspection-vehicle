#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Isolate this simulation and its publishers from any real robot.
export ROS_DOMAIN_ID="${MECANUM_TEST_DOMAIN:-87}"
export GZ_PARTITION="mecanum-test-$$"
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch mecanum_bringup simulation.launch.py gui:=false > simulation.log 2>&1 &
launch_pid=$!
trap 'kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" || true' EXIT
# Tool waits for telemetry/controller; 30 seconds is its bounded startup timeout.
ros2 run mecanum_bringup motion_smoke --execute
