#include "Encoder.h"

static uint16_t previous[2];
static int32_t total[2];

static void Encoder_TimerInit(TIM_TypeDef *timer)
{
    TIM_TimeBaseInitTypeDef base;
    TIM_ICInitTypeDef input;

    TIM_TimeBaseStructInit(&base);
    base.TIM_Period = 65535U;
    base.TIM_Prescaler = 0U;
    TIM_TimeBaseInit(timer, &base);
    TIM_EncoderInterfaceConfig(timer, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&input);
    input.TIM_ICFilter = 6U;
    input.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(timer, &input);
    input.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(timer, &input);
    TIM_SetCounter(timer, 0U);
    TIM_Cmd(timer, ENABLE);
}

void Encoder_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);
    Encoder_TimerInit(TIM2);
    Encoder_TimerInit(TIM3);
    Encoder_Reset();
}

int32_t Encoder_GetCount(uint8_t channel)
{
    uint8_t index;
    uint16_t current;
    uint16_t difference;
    int32_t delta;
    if ((channel < 1U) || (channel > 2U))
    {
        return 0;
    }
    index = channel - 1U;
    current = TIM_GetCounter((channel == 1U) ? TIM2 : TIM3);
    difference = (uint16_t)(current - previous[index]);
    delta = (difference < 32768U) ? (int32_t)difference : (int32_t)difference - 65536L;
    previous[index] = current;
    total[index] += delta;
    return total[index];
}

void Encoder_Reset(void)
{
    /* Snapshot instead of clearing live hardware counters. Reset at rest. */
    previous[0] = TIM_GetCounter(TIM2);
    previous[1] = TIM_GetCounter(TIM3);
    total[0] = 0;
    total[1] = 0;
}
