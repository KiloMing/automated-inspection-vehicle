#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../chassis"
result_dir=$(mktemp -d)
trap 'rm -rf "$result_dir"' EXIT
compile_run() {
  local name=$1
  shift
  cc -std=c99 -Wall -Wextra -Werror -IHardware -ISystem "$@" -lm -o "$result_dir/$name"
  "$result_dir/$name"
}
compile_run motor_pid -DMOTOR_PID_KP=100 -DMOTOR_PID_KI=50 -DMOTOR_PID_KD=1 Tests/test_motor_pid.c Hardware/MotorPID.c
compile_run hwt101 Tests/test_hwt101.c Hardware/HWT101_Parser.c
compile_run motor_scan Tests/test_motor_scan.c Hardware/MotorScan_Core.c
compile_run heading Tests/test_heading_control.c Hardware/HeadingControl.c
compile_run remote Tests/test_remote_control.c Hardware/RemoteControl.c Hardware/HeadingControl.c
compile_run serial -ITests/remote_serial_stubs Tests/test_remote_serial.c Hardware/RemoteSerial.c
