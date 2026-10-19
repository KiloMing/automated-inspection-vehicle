#ifndef REMOTE_SERIAL_H
#define REMOTE_SERIAL_H
#define REMOTE_SERIAL_MAX_AGE_MS 300U /* Queue backlog, not joystick inactivity */
/* Adds buffered USART1 reception after the existing UART_Init(115200).
 * UART.c/h, pins and transmit functions are unchanged. No UART polling may
 * consume DR while this receiver is enabled. All parsing stays in main.
 */
void RemoteSerial_Init(void);
void RemoteSerial_Poll(void);
void RemoteSerial_RxIRQ(void);
#endif
