#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include <stdint.h>

#define REMOTE_DEADZONE 5
/* Event-driven app: keep command until next packet (including neutral).
 * Set >0 only when the app sends periodic heartbeat packets. */
#ifndef REMOTE_TIMEOUT_MS
#define REMOTE_TIMEOUT_MS 0U
#endif
#define REMOTE_MAX_SPEED_M_S 0.4f
#define REMOTE_MAX_TURN_DPS 45.0f
#define REMOTE_MAX_LEAD_DEG 20.0f
#define REMOTE_ACCEL_M_S2 0.5f
#define REMOTE_WHEEL_LIMIT_COUNTS 150.0f
#define REMOTE_STARTUP_PWM 200U /* 20% while starting from near standstill */

/* Main-context API. Owns motor targets, not UART. Do not run
 * HeadingControl_Process or another motor command source concurrently.
 * Feed accepts [j,LX,LY,RX,RY] and [joystick,LX,LY,RX,RY], integers +/-100.
 */
void RemoteControl_Init(void);
void RemoteControl_Feed(uint8_t byte);
void RemoteControl_FeedAt(uint8_t byte, uint32_t received_ms);
void RemoteControl_Stop(void); /* Fault/disconnect: requires neutral to rearm. */
void RemoteControl_Process(void);
/* Inspect remote_debug in Keil Watch; no serial output.
 * State: 0=idle,1=wait packet,2=wait neutral,3=IMU unavailable,
 * 4=IMU continuity lost,5=motor deadline,6=braking,7=running.
 */
typedef struct {
    uint32_t rx_bytes, valid_packets, rejected_packets, uart_errors;
    uint32_t packet_age_ms, imu_age_ms;
    uint8_t state;
    int16_t throttle, steering;
} RemoteControl_Debug;
extern volatile RemoteControl_Debug remote_debug;
void RemoteControl_RxError(void);
#endif
