#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H
#include "main.h"
#include "Mecanum.h"
/* Boot frame: x forward, y left, yaw counterclockwise. valid=0 requires reboot. */
typedef struct {
 float x_m,y_m,yaw_rad;
 uint8_t valid;
} ChassisPose;
void Chassis_GetPose(ChassisPose *value);
HAL_StatusTypeDef Chassis_Init(void);
void Chassis_Update(void);
uint8_t Chassis_SetVelocity(float x_mps,float y_mps,float z_radps);
void Chassis_GetMeasuredVelocity(ChassisVelocity *value);
void Chassis_EnableHeadingAssist(uint8_t enabled);
uint8_t WheelPID_SetGains(unsigned wheel,float kp,float ki,float kd);
#endif
