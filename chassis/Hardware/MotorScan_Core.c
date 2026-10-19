#include "MotorScan_Core.h"
void MotorScan_CoreStart(MotorScan_Core *s, uint32_t now)
{
    s->pwm = 0U;
    s->state = SCAN_RUNNING;
    s->step_ms = now;
}
void MotorScan_CoreStop(MotorScan_Core *s, uint8_t reason)
{
    s->pwm = 0U;
    s->state = reason;
}
void MotorScan_CoreUpdate(MotorScan_Core *s, uint32_t now)
{
    if (s->state != SCAN_RUNNING || (uint32_t)(now - s->step_ms) < SCAN_DWELL_MS) return;
    s->step_ms = now;
    if (s->pwm >= 1000U) MotorScan_CoreStop(s, SCAN_DONE);
    else s->pwm += SCAN_STEP;
}
int32_t MotorScan_Delta(uint16_t current, uint16_t previous)
{
    uint16_t difference = (uint16_t)(current - previous);
    return (difference < 32768U) ? (int32_t)difference : (int32_t)difference - 65536L;
}
float MotorScan_RPM(int32_t delta, uint32_t dt_ms)
{
    if (dt_ms == 0U) return 0.0f;
    return (float)delta * 60000.0f / (SCAN_COUNTS_PER_REV * (float)dt_ms);
}
float MotorScan_Speed(int32_t delta, uint32_t dt_ms)
{
    return MotorScan_RPM(delta, dt_ms) * SCAN_CIRCUMFERENCE_M / 60.0f;
}
