#ifndef MECANUM_H
#define MECANUM_H
#include "ChassisConfig.h"
typedef struct {float x,y,z;} ChassisVelocity;
typedef struct {float v[4];} WheelVelocity; /* A,B,C,D wheel rim m/s */
void Mecanum_Inverse(const ChassisVelocity *body,WheelVelocity *wheel);
void Mecanum_Forward(const WheelVelocity *wheel,ChassisVelocity *body);
float Mecanum_Limit(WheelVelocity *wheel,float max_mps);
float Mecanum_CountsToMps(float cps);
float Mecanum_MpsToCounts(float mps);
#endif
