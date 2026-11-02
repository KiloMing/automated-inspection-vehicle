#ifndef BLUETOOTH_H
#define BLUETOOTH_H
#include "main.h"
#define BLUETOOTH_BAUD_RATE 115200U
typedef struct {
  uint32_t valid_packets, invalid_packets, rx_errors, connected;
  int32_t target[4];
} BluetoothStatus;
extern volatile BluetoothStatus g_bluetooth;
HAL_StatusTypeDef Bluetooth_Init(void);
/* Diagnostic TX-only mode: no receive commands and no binary status frames. */
HAL_StatusTypeDef Bluetooth_InitTxOnly(void);
HAL_StatusTypeDef Bluetooth_SendText(const char *text, uint16_t length);
/* Raw diagnostic receive: 1 byte, 0 empty, -1 UART fault. */
int Bluetooth_ReadByte(uint8_t *byte);
HAL_StatusTypeDef Bluetooth_SendAsync(const char *data,uint16_t length);
void Bluetooth_Update(void);
#endif
