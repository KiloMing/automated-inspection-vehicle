#ifndef MOTOR_SCAN_CORE_H
#define MOTOR_SCAN_CORE_H
#include <stdint.h>
#define SCAN_STEP 20U
#define SCAN_DWELL_MS 1000U
#define SCAN_SETTLE_MS 500U
#define SCAN_COUNTS_PER_REV 2500.0f
#define SCAN_CIRCUMFERENCE_M 0.20f
#define SCAN_IDLE 0U
#define SCAN_RUNNING 1U
#define SCAN_DONE 2U
#define SCAN_STOPPED 3U
#define SCAN_OVERFLOW 4U
typedef struct {
    uint32_t step_ms;
    uint16_t pwm;
    uint8_t state;
} MotorScan_Core;
void MotorScan_CoreStart(MotorScan_Core *scan, uint32_t now);
void MotorScan_CoreStop(MotorScan_Core *scan, uint8_t reason);
void MotorScan_CoreUpdate(MotorScan_Core *scan, uint32_t now);
int32_t MotorScan_Delta(uint16_t current, uint16_t previous);
float MotorScan_RPM(int32_t delta, uint32_t dt_ms);
float MotorScan_Speed(int32_t delta, uint32_t dt_ms);
#endif
