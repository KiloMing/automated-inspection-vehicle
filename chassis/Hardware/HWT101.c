#include "stm32f10x.h"
#include "HWT101.h"
#include "Delay.h"

#define RX_CAPACITY 256U
#define RX_MASK (RX_CAPACITY - 1U)
typedef struct {
    uint32_t ms;
    uint8_t byte;
} HWT101_RxByte;
static volatile HWT101_RxByte rx[RX_CAPACITY];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint8_t rx_lost;
static HWT101_Parser parser;
/* 0=idle, 1=unlock settling, 2=module settling. Delays are conservative
 * host guard times, not confirmation that a command has been executed. */
static uint8_t command_state;
static uint8_t command_reboot;
static uint32_t command_time;
static void HWT101_CommandPoll(void);
static void HWT101_FlushRx(void);

void HWT101_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef serial;
    NVIC_InitTypeDef irq;
    NVIC_DisableIRQ(USART2_IRQn);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    USART_DeInit(USART2);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);
    USART_StructInit(&serial);
    serial.USART_BaudRate = 115200U;
    serial.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &serial);
    rx_head = 0U;
    rx_tail = 0U;
    rx_lost = 0U;
    HWT101_ParserInit(&parser);
    command_state = 0U;
    NVIC_ClearPendingIRQ(USART2_IRQn);
    irq.NVIC_IRQChannel = USART2_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 1U;
    irq.NVIC_IRQChannelSubPriority = 0U;
    irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_ERR, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

void HWT101_RxIRQ(void)
{
    uint32_t status = USART2->SR;
    uint8_t byte;
    uint16_t next;
    if (!(status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)))
        return;
    /* Reading SR then DR clears receive/error flags on STM32F1. */
    byte = (uint8_t)USART2->DR;
    if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    {
        rx_lost = 1U;
        return;
    }
    if (rx_lost) return;
    next = (uint16_t)((rx_head + 1U) & RX_MASK);
    if (next == rx_tail)
    {
        rx_lost = 1U;
        return;
    }
    rx[rx_head].byte = byte;
    rx[rx_head].ms = Delay_GetTick();
    rx_head = next;
}

void HWT101_Poll(void)
{
    uint32_t mask;
    uint32_t timestamp;
    uint8_t byte;
    uint8_t lost;
    uint8_t available;
    HWT101_CommandPoll();
    if (command_state)
    {
        HWT101_FlushRx();
        return;
    }
    for (;;)
    {
        mask = __get_PRIMASK();
        __disable_irq();
        lost = rx_lost;
        if (lost)
        {
            rx_tail = rx_head;
            rx_lost = 0U;
        }
        available = (uint8_t)(rx_tail != rx_head);
        byte = 0U;
        timestamp = 0U;
        if (available)
        {
            byte = rx[rx_tail].byte;
            timestamp = rx[rx_tail].ms;
            rx_tail = (uint16_t)((rx_tail + 1U) & RX_MASK);
        }
        __set_PRIMASK(mask);
        if (lost) HWT101_ParserBreak(&parser);
        if (!available) break;
        HWT101_ParserFeed(&parser, byte, timestamp);
    }
    HWT101_ParserExpire(&parser, Delay_GetTick());
}

uint8_t HWT101_Read(HWT101_Data *data)
{
    if (!data) return 0U;
    HWT101_ParserExpire(&parser, Delay_GetTick());
    *data = parser.data;
    return data->valid;
}

uint8_t HWT101_Zero(void)
{
    HWT101_Poll();
    if (command_state) return 0U;
    return HWT101_ParserZero(&parser, Delay_GetTick());
}

static void HWT101_FlushRx(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    rx_tail = rx_head;
    rx_lost = 0U;
    __set_PRIMASK(mask);
}

static uint8_t HWT101_SendCommand(uint8_t reg, uint16_t value)
{
    uint8_t bytes[5];
    uint8_t i;
    uint32_t start = Delay_GetTick();
    bytes[0] = 0xFFU;
    bytes[1] = 0xAAU;
    bytes[2] = reg;
    bytes[3] = (uint8_t)value;
    bytes[4] = (uint8_t)(value >> 8);
    for (i = 0U; i < 5U; i++)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
            if ((uint32_t)(Delay_GetTick() - start) >= 10U) return 0U;
        USART_SendData(USART2, bytes[i]);
    }
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
        if ((uint32_t)(Delay_GetTick() - start) >= 10U) return 0U;
    return 1U;
}

static uint8_t HWT101_StartCommand(uint8_t reboot)
{
    if (command_state) return 0U;
    HWT101_ParserBreak(&parser);
    HWT101_FlushRx();
    if (!HWT101_SendCommand(0x69U, 0xB588U)) return 0U;
    command_reboot = reboot;
    command_time = Delay_GetTick();
    command_state = 1U;
    return 1U;
}

uint8_t HWT101_ModuleZero(void)
{
    return HWT101_StartCommand(0U);
}

uint8_t HWT101_Reboot(void)
{
    return HWT101_StartCommand(1U);
}

uint8_t HWT101_IsBusy(void)
{
    return (uint8_t)(command_state != 0U);
}

static void HWT101_CommandPoll(void)
{
    uint32_t elapsed = Delay_GetTick() - command_time;
    uint32_t sequence;
    if (command_state == 1U && elapsed >= 20U)
    {
        /* Unlock expires after 10 seconds. Do not send if main stalled. */
        if (elapsed >= 10000U || !HWT101_SendCommand(command_reboot ? 0x00U : 0x76U,
                                                    command_reboot ? 0x00FFU : 0x0000U))
        {
            command_state = 0U;
            HWT101_FlushRx();
            return;
        }
        command_time = Delay_GetTick();
        command_state = 2U;
    }
    else if (command_state == 2U && elapsed >= (command_reboot ? 1000U : 500U))
    {
        HWT101_FlushRx();
        /* Do not interpret the commanded yaw discontinuity as a physical turn.
         * Preserve sequence so a future controller sees a fresh sample. */
        sequence = parser.data.sequence;
        HWT101_ParserInit(&parser);
        parser.data.sequence = sequence;
        command_state = 0U;
    }
}

