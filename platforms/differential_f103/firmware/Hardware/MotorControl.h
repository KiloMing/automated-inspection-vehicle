#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdint.h>

/* Dual independent PID loops. Units: signed counts per 20 ms.
 * Same gains for both motors are configured in MotorPID.h.
 * Init owns encoder timers TIM2/TIM3, PWM TIM4 and sampling TIM1.
 * Call after Delay_TickInit; do not use MotorScan concurrently.
 * All APIs except SampleIRQ are main-context only.
 */
typedef struct
{
    int32_t encoder_delta;
    int32_t target_counts;
    int16_t pwm; /* signed thousandths */
} MotorControl_State;

void MotorControl_Init(void);
/* Optional startup duty floor, thousandths (0 disables). Only applied while
 * encoder magnitude <=2 counts/20ms and PID requests nonzero drive. */
void MotorControl_SetStartupPWM(uint16_t pwm);
/* IDs 1/2. Zero stops immediately and resets integral.
 * Stop the shaft before commanding reverse direction.
 */
void MotorControl_SetTarget(uint8_t motor_id, int16_t counts_per_20ms);
void MotorControl_Stop(void); /* Both motors; clears targets and PID state. */
void MotorControl_Process(void); /* Call continuously; must service every 20 ms. */
uint8_t MotorControl_GetState(uint8_t motor_id, MotorControl_State *state);
void MotorControl_SampleIRQ(void); /* TIM1 update interrupt entry. */
uint32_t MotorControl_FaultSequence(void); /* Increments on missed deadlines. */

#endif
