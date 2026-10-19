#ifndef MOTOR_SCAN_H
#define MOTOR_SCAN_H
#include "MotorScan_Core.h"
typedef struct {
    uint32_t time_ms;
    uint32_t dt_ms;
    int32_t delta;
    uint32_t dropped;
    uint16_t pwm;
    uint8_t steady;
    uint8_t state;
} MotorScan_Sample;
/* Owns TIM1 sampling and motor 1; Encoder_Init and Motor_Init must precede Init.
 * Main context only, except SampleIRQ. Do not reset TIM2 during a scan.
 */
void MotorScan_Init(void);
void MotorScan_Start(void);
void MotorScan_Stop(void);
void MotorScan_Poll(void);
uint8_t MotorScan_Read(MotorScan_Sample *sample);
void MotorScan_SampleIRQ(void);
#endif
