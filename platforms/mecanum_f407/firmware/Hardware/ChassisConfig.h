#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H
#include "MotorATune.h"
/* Geometry in meters; counts measured per wheel revolution. */
/* Telemetry period: 20 ms = 50 Hz; busy UART frames are skipped. */
#define CHASSIS_TELEMETRY_PERIOD_MS 20U
/* Host sends velocity at 20 Hz; invalid packets do not refresh timeout. */
#define CHASSIS_COMMAND_TIMEOUT_MS 300U
#define CHASSIS_RADIUS 0.040f
#define CHASSIS_K 0.195f
#define CHASSIS_COUNTS_REV 4000.0f
#define CHASSIS_WHEEL_MAX_MPS 0.48f
/* Copied from the user's current A tuning, output per-mille, speed counts/s. */
#define CHASSIS_KP MOTOR_A_PID_KP
#define CHASSIS_KI MOTOR_A_PID_KI
#define CHASSIS_KD MOTOR_A_PID_KD
#define CHASSIS_HEADING_DEFAULT 0
#define CHASSIS_HEADING_KP 2.0f
#define CHASSIS_RATE_KP 0.3f
#define CHASSIS_HEADING_MAX 0.5f
#endif
