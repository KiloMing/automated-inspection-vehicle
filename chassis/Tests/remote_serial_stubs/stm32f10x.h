#ifndef TEST_STM32_SERIAL_H
#define TEST_STM32_SERIAL_H
#include <stdint.h>
typedef struct { uint32_t SR, DR; } USART_TypeDef;
extern USART_TypeDef test_usart;
#define USART1 (&test_usart)
#define USART1_IRQn 37
#define ENABLE 1
#define USART_SR_RXNE 0x20U
#define USART_SR_ORE 0x08U
#define USART_SR_FE 0x02U
#define USART_SR_NE 0x04U
#define USART_SR_PE 0x01U
#define USART_IT_RXNE 1
#define USART_IT_ERR 2
typedef struct {
    uint8_t NVIC_IRQChannel, NVIC_IRQChannelPreemptionPriority;
    uint8_t NVIC_IRQChannelSubPriority, NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;
static inline void NVIC_DisableIRQ(int n) { (void)n; }
static inline void NVIC_ClearPendingIRQ(int n) { (void)n; }
static inline void NVIC_Init(NVIC_InitTypeDef *n) { (void)n; }
static inline void USART_ITConfig(USART_TypeDef *u,int i,int e)
{ (void)u; (void)i; (void)e; }
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t m) { (void)m; }
#endif
