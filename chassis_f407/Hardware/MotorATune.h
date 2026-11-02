#ifndef MOTOR_A_TUNE_H
#define MOTOR_A_TUNE_H
#include "main.h"
/* Edit these gains in code, then rebuild. Initial tuning values only. */
#define MOTOR_A_PID_KP 1.5f
#define MOTOR_A_PID_KI 0.2f
#define MOTOR_A_PID_KD 0.00f
#define MOTOR_A_PWM_MAX 1000.0f /* per-mille: 100% */
HAL_StatusTypeDef MotorATune_Init(void);
void MotorATune_Update(void);
/* Units: counts/s; output per-mille; Ki per second, Kd seconds. */
uint8_t MotorATune_SetPID(float kp,float ki,float kd);
uint8_t MotorATune_SetTarget(float counts_per_second);
void MotorATune_Stop(void);
#endif
