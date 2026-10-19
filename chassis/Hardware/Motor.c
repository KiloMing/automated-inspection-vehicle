#include "Motor.h"
#include "Delay.h"

#define MOTOR_PWM_HZ 20000UL
static uint16_t pwm_period;
static int8_t direction[2];

static uint16_t Motor_Pins(uint8_t id)
{
    return (id == MOTOR_1) ? (GPIO_Pin_12 | GPIO_Pin_13) : (GPIO_Pin_14 | GPIO_Pin_15);
}

static void Motor_Compare(uint8_t id, uint16_t value)
{
    if (id == MOTOR_1)
        TIM_SetCompare1(TIM4, value);
    else
        TIM_SetCompare2(TIM4, value);
}

void Motor_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef base;
    TIM_OCInitTypeDef output;
    RCC_ClocksTypeDef clocks;
    uint32_t timer_clock;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    /* Set output latches before switching GPIO to output mode. */
    GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
    GPIO_SetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &gpio);

    TIM_DeInit(TIM4);
    RCC_GetClocksFreq(&clocks);
    timer_clock = clocks.PCLK1_Frequency;
    if (clocks.PCLK1_Frequency != clocks.HCLK_Frequency)
        timer_clock *= 2U;
    pwm_period = (uint16_t)(timer_clock / MOTOR_PWM_HZ);
    TIM_TimeBaseStructInit(&base);
    base.TIM_Period = pwm_period - 1U;
    base.TIM_Prescaler = 0U;
    TIM_TimeBaseInit(TIM4, &base);
    TIM_OCStructInit(&output);
    output.TIM_OCMode = TIM_OCMode_PWM1;
    output.TIM_OutputState = TIM_OutputState_Enable;
    output.TIM_Pulse = 0U;
    output.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &output);
    TIM_OC2Init(TIM4, &output);
    /* Immediate CCR writes ensure PWM goes low before direction changes. */
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Disable);
    TIM_Cmd(TIM4, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);
    direction[0] = 0;
    direction[1] = 0;
}

void Motor_Brake(uint8_t motor_id)
{
    if ((motor_id != MOTOR_1) && (motor_id != MOTOR_2)) return;
    Motor_Compare(motor_id, 0U);
    GPIO_SetBits(GPIOB, Motor_Pins(motor_id));
    direction[motor_id - 1U] = 0;
}

void Motor_Coast(uint8_t motor_id)
{
    if ((motor_id != MOTOR_1) && (motor_id != MOTOR_2)) return;
    Motor_Compare(motor_id, 0U);
    GPIO_ResetBits(GPIOB, Motor_Pins(motor_id));
    /* IN1=IN2=0, PWM=1 is high impedance (PWM=0 is brake). */
    Motor_Compare(motor_id, pwm_period);
    direction[motor_id - 1U] = 0;
}

void Motor_SetPWM(uint8_t motor_id, int16_t pwm)
{
    int8_t next_direction;
    uint16_t positive_pin;
    uint16_t negative_pin;
    uint16_t magnitude;
    uint16_t compare;
    if ((motor_id != MOTOR_1) && (motor_id != MOTOR_2)) return;
    if (pwm > 1000) pwm = 1000;
    if (pwm < -1000) pwm = -1000;
    if (pwm == 0)
    {
        Motor_Brake(motor_id);
        return;
    }
    next_direction = (pwm > 0) ? 1 : -1;
    if (direction[motor_id - 1U] != next_direction)
    {
        Motor_Compare(motor_id, 0U);
        /* Electrical transition pause only; this does not stop a spinning shaft. */
        Delay_us(100U);
        positive_pin = (motor_id == MOTOR_1) ? GPIO_Pin_12 : GPIO_Pin_14;
        negative_pin = (motor_id == MOTOR_1) ? GPIO_Pin_13 : GPIO_Pin_15;
        GPIO_ResetBits(GPIOB, Motor_Pins(motor_id));
        GPIO_SetBits(GPIOB, (next_direction > 0) ? positive_pin : negative_pin);
        direction[motor_id - 1U] = next_direction;
    }
    magnitude = (uint16_t)((pwm > 0) ? pwm : -pwm);
    compare = (uint16_t)(((uint32_t)pwm_period * magnitude) / 1000U);
    Motor_Compare(motor_id, compare);
}
