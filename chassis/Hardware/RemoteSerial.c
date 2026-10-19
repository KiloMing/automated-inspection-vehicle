#include "stm32f10x.h"
#include "RemoteSerial.h"
#include "RemoteControl.h"
#include "Delay.h"

#define RX_SIZE 256U
typedef struct { uint32_t ms; uint8_t byte; } RxItem;
static volatile RxItem queue[RX_SIZE];
static volatile uint16_t head, tail;
static volatile uint8_t lost;

void RemoteSerial_Init(void)
{
    NVIC_InitTypeDef irq;
    NVIC_DisableIRQ(USART1_IRQn);
    head=tail=0U;
    lost=0U;
    NVIC_ClearPendingIRQ(USART1_IRQn);
    irq.NVIC_IRQChannel=USART1_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority=1U;
    irq.NVIC_IRQChannelSubPriority=0U;
    irq.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&irq);
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
    USART_ITConfig(USART1,USART_IT_ERR,ENABLE);
}

void RemoteSerial_RxIRQ(void)
{
    uint32_t status=USART1->SR;
    uint16_t next;
    uint8_t byte;
    if (!(status & (USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE))) return;
    byte=(uint8_t)USART1->DR; /* SR then DR clears receive errors. */
    if (status & (USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE))
    { lost=1U; return; }
    if (lost) return;
    next=(uint16_t)((head+1U)&(RX_SIZE-1U));
    if (next==tail) { lost=1U; return; }
    queue[head].byte=byte;
    queue[head].ms=Delay_GetTick();
    head=next;
}

void RemoteSerial_Poll(void)
{
    uint32_t mask, ms;
    uint8_t byte, error, available, budget=64U;
    while (budget--)
    {
        mask=__get_PRIMASK();
        __disable_irq();
        error=lost;
        if (error) { tail=head; lost=0U; }
        available=(uint8_t)(tail!=head);
        byte=0U; ms=0U;
        if (available)
        {
            byte=queue[tail].byte;
            ms=queue[tail].ms;
            tail=(uint16_t)((tail+1U)&(RX_SIZE-1U));
        }
        __set_PRIMASK(mask);
        if (error) { RemoteControl_RxError(); return; }
        if (!available) return;
        if ((uint32_t)(Delay_GetTick()-ms)>=REMOTE_SERIAL_MAX_AGE_MS)
        {
            RemoteControl_RxError();
            continue;
        }
        RemoteControl_FeedAt(byte,ms);
    }
}
