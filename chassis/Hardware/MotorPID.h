#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <stdint.h>

/* Speed input: m/s. Output: PWM thousandths, matching Motor_SetPWM.
 * Call Update every 20 ms. Gains are shared by both motor controllers.
 * KP: PWM/(m/s), KI: PWM/((m/s)*s), KD: PWM*s/(m/s).
 */
#ifndef MOTOR_PID_KP
#define MOTOR_PID_KP 8500.0f
#endif
#ifndef MOTOR_PID_KI
#define MOTOR_PID_KI 2000.0f
#endif
#ifndef MOTOR_PID_KD
#define MOTOR_PID_KD 000.0f
#endif

#define MOTOR_PID_PERIOD_S 0.02f
#define MOTOR_PID_OUTPUT_MAX 1000.0f

/* Use separate state objects for the two motors. */
typedef struct
{
    float integral;
    float previous_speed;
    uint8_t initialized;
} MotorPID;

void MotorPID_Reset(MotorPID *pid);
/* Pure PID, no feedforward or hardware writes. Zero target resets state.
 * Output follows target direction; overspeed reduces duty to zero instead
 * of commanding reverse torque. Stop before changing target direction.
 */
int16_t MotorPID_Update(MotorPID *pid, float target_m_s, float measured_m_s);

#endif
