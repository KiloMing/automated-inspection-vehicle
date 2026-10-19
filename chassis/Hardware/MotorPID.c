#include "MotorPID.h"

static float MotorPID_Clamp(float value, float low, float high)
{
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

void MotorPID_Reset(MotorPID *pid)
{
    if (!pid) return;
    pid->integral = 0.0f;
    pid->previous_speed = 0.0f;
    pid->initialized = 0U;
}

int16_t MotorPID_Update(MotorPID *pid, float target_m_s, float measured_m_s)
{
    float error, derivative, candidate, output, low, high;
    if (!pid) return 0;
    if (target_m_s == 0.0f)
    {
        MotorPID_Reset(pid);
        return 0;
    }
    low = (target_m_s > 0.0f) ? 0.0f : -MOTOR_PID_OUTPUT_MAX;
    high = (target_m_s > 0.0f) ? MOTOR_PID_OUTPUT_MAX : 0.0f;
    error = target_m_s - measured_m_s;
    /* Derivative on measurement avoids a kick on target changes. */
    derivative = pid->initialized ?
        -MOTOR_PID_KD * (measured_m_s - pid->previous_speed) / MOTOR_PID_PERIOD_S : 0.0f;
    candidate = MotorPID_Clamp(pid->integral + MOTOR_PID_KI * error * MOTOR_PID_PERIOD_S,
                              -MOTOR_PID_OUTPUT_MAX, MOTOR_PID_OUTPUT_MAX);
    output = MOTOR_PID_KP * error + candidate + derivative;
    /* Reject integration that pushes an already saturated output further. */
    if (!((output > high && error > 0.0f) || (output < low && error < 0.0f)))
        pid->integral = candidate;
    output = MotorPID_Clamp(MOTOR_PID_KP * error + pid->integral + derivative, low, high);
    pid->previous_speed = measured_m_s;
    pid->initialized = 1U;
    return (int16_t)output;
}
