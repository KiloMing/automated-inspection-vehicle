#ifndef ENCODER_H
#define ENCODER_H
#include "stm32f10x.h"

/* Encoder 1: PA0/PA1 (TIM2); encoder 2: PA6/PA7 (TIM3).
 * Quadrature x4 counting. Call GetCount frequently: fewer than 32768
 * counts may occur between calls on each channel. Main-loop use only.
 * Signed total must remain within the int32_t range; reset between tests.
 */
void Encoder_Init(void);
int32_t Encoder_GetCount(uint8_t channel);
void Encoder_Reset(void);
#endif
