#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H
#include <stdint.h>

/* For INCREASING HWT yaw, motor target signs must be +1 or -1.
 * Zero means wiring/direction not configured; commands are rejected.
 */
#ifndef HEADING_MOTOR1_SIGN
#define HEADING_MOTOR1_SIGN -1
#endif
#ifndef HEADING_MOTOR2_SIGN
#define HEADING_MOTOR2_SIGN 1
#endif

/* Output units: encoder counts/20ms. Provisional outer-loop gains. */
#define HEADING_KP 1.0f
#define HEADING_KI 0.0f
#define HEADING_KD 0.10f
#define HEADING_MAX_COUNTS 50.0f
#define HEADING_ENTER_DEG 1.0f
#define HEADING_EXIT_DEG 2.0f
#define HEADING_SETTLED_DPS 3.0f
#define HEADING_DATA_TIMEOUT_MS 300U

typedef enum { HEADING_IDLE, HEADING_TURNING, HEADING_HOLDING,
               HEADING_BRAKING, HEADING_FAULT } HeadingControl_Mode;
typedef struct {
    float target_deg; /* continuous angle, preserves requested turn direction */
    float current_deg;
    float error_deg;
    HeadingControl_Mode mode;
} HeadingControl_State;

/* Init after MotorControl_Init and HWT101_Init. All calls main context only.
 * Process polls HWT101; call continuously alongside MotorControl_Process.
 * Do not reset/zero HWT101 or command motors directly while active.
 */
void HeadingControl_Init(void);
uint8_t HeadingControl_Rotate(float relative_deg); /* Current angle + delta; stays holding. */
uint8_t HeadingControl_Hold(void); /* Capture current heading and reject disturbances. */
void HeadingControl_Stop(void);
void HeadingControl_Process(void);
void HeadingControl_GetState(HeadingControl_State *state);
#endif
