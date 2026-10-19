#ifndef MOTOR_H
#define MOTOR_H
#include "stm32f10x.h"

/* TB6612: PWMA=PB6, PWMB=PB7, AIN1/2=PB12/13, BIN1/2=PB14/15.
 * STBY is tied high externally; no GPIO is used for it.
 * Main-loop calls only. Motor_Init starts both channels in brake mode.
 */
#define MOTOR_1 1U
#define MOTOR_2 2U
void Motor_Init(void);
/* Signed duty in thousandths, clamped to [-1000,1000]. Zero = brake.
 * Sign selects electrical direction; stop the shaft before reversing.
 * Invalid motor IDs are ignored. Call Motor_Init before other functions.
 */
void Motor_SetPWM(uint8_t motor_id, int16_t pwm);
void Motor_Brake(uint8_t motor_id);
void Motor_Coast(uint8_t motor_id);
#endif
