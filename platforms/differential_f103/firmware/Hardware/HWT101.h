#ifndef HWT101_H
#define HWT101_H
#include "HWT101_Parser.h"

/* USART2: module TX -> PA3, module RX -> PA2, 115200 8N1, TTL.
 * Initialize the millisecond tick first. Call Poll frequently in main.
 * All public functions except RxIRQ must be called from main only.
 */
void HWT101_Init(void);
void HWT101_Poll(void);
void HWT101_RxIRQ(void);
/* Returns 1 only for a fresh angle (<500 ms old). Always copies status.
 * Inspect sequence to distinguish new frames from repeated snapshots.
 */
uint8_t HWT101_Read(HWT101_Data *data);
/* Software reference only. Stationary, settled sensor recommended.
 * Reestablishes continuous-angle validity after a stream interruption.
 */
uint8_t HWT101_Zero(void);
/* Asynchronous module commands: 1=accepted locally, 0=busy/TX failed.
 * Requires PA2 -> module RX. Keep calling Poll. No module ACK is implied.
 * These reset the software reference when reception resumes.
 */
uint8_t HWT101_ModuleZero(void);
uint8_t HWT101_Reboot(void);
uint8_t HWT101_IsBusy(void);
#endif
