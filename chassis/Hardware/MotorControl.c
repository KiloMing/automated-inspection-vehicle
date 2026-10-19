#include "stm32f10x.h"
#include "MotorControl.h"
#include "MotorPID.h"
#include "Motor.h"
#include "Encoder.h"

#define COUNTS_TO_M_S 0.004f /* 2500 counts/rev, circumference 0.20 m */
static MotorPID pid[2];
static MotorControl_State state[2];
static uint16_t previous[2];
static volatile int32_t delta[2];
static volatile uint8_t pending;
static volatile uint8_t deadline_fault;
static volatile uint32_t fault_sequence;
static uint16_t startup_pwm;

void MotorControl_SetStartupPWM(uint16_t pwm)
{
    startup_pwm = pwm > 1000U ? 1000U : pwm;
}

uint32_t MotorControl_FaultSequence(void) { return fault_sequence; }

static void StopLocked(void)
{
    uint8_t i;
    for (i = 0U; i < 2U; ++i)
    {
        state[i].target_counts = 0;
        state[i].pwm = 0;
        MotorPID_Reset(&pid[i]);
        Motor_Brake(i + 1U);
    }
}

void MotorControl_Init(void)
{
    TIM_TimeBaseInitTypeDef base;
    RCC_ClocksTypeDef clocks;
    NVIC_InitTypeDef irq;
    uint32_t clock_hz;
    Encoder_Init();
    Motor_Init();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_DeInit(TIM1);
    StopLocked();
    state[0].encoder_delta = state[1].encoder_delta = 0;
    pending = deadline_fault = 0U;
    fault_sequence = 0U;
    startup_pwm = 0U;
    previous[0] = TIM_GetCounter(TIM2);
    previous[1] = TIM_GetCounter(TIM3);
    RCC_GetClocksFreq(&clocks);
    clock_hz = clocks.PCLK2_Frequency;
    if (clocks.PCLK2_Frequency != clocks.HCLK_Frequency) clock_hz *= 2U;
    TIM_TimeBaseStructInit(&base);
    base.TIM_Prescaler = (uint16_t)(clock_hz / 10000U - 1U);
    base.TIM_Period = 199U;
    TIM_TimeBaseInit(TIM1, &base);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
    irq.NVIC_IRQChannel = TIM1_UP_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 1U;
    irq.NVIC_IRQChannelSubPriority = 0U;
    irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

void MotorControl_SetTarget(uint8_t motor_id, int16_t counts_per_20ms)
{
    uint8_t index;
    uint32_t mask;
    if (motor_id < 1U || motor_id > 2U) return;
    index = motor_id - 1U;
    mask = __get_PRIMASK();
    __disable_irq();
    /* A missed deadline must stop both loops before accepting a new command. */
    if (deadline_fault)
    {
        StopLocked();
        deadline_fault = pending = 0U;
    }
    if (counts_per_20ms == 0 || state[index].target_counts == 0 ||
        ((counts_per_20ms > 0) != (state[index].target_counts > 0)))
        MotorPID_Reset(&pid[index]);
    state[index].target_counts = counts_per_20ms;
    if (counts_per_20ms == 0)
    {
        Motor_Brake(motor_id);
        state[index].pwm = 0;
    }
    __set_PRIMASK(mask);
}

void MotorControl_Stop(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    StopLocked();
    deadline_fault = 0U;
    __set_PRIMASK(mask);
}

void MotorControl_SampleIRQ(void)
{
    uint8_t i;
    uint16_t current, difference;
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == RESET) return;
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    if (pending)
    {
        TIM_SetCompare1(TIM4, 0U);
        TIM_SetCompare2(TIM4, 0U);
        deadline_fault = 1U;
        fault_sequence++;
    }
    for (i = 0U; i < 2U; ++i)
    {
        current = TIM_GetCounter(i == 0U ? TIM2 : TIM3);
        difference = (uint16_t)(current - previous[i]);
        previous[i] = current;
        delta[i] = difference < 32768U ? (int32_t)difference : (int32_t)difference - 65536L;
    }
    pending = 1U;
}

void MotorControl_Process(void)
{
    uint8_t i;
    uint32_t enabled = NVIC->ISER[(uint32_t)TIM1_UP_IRQn >> 5U] &
        (1UL << ((uint32_t)TIM1_UP_IRQn & 31U));
    /* Protect sampling state without blocking either serial receiver during
     * software floating-point PID or the driver's 100 us direction pauses. */
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    __DSB();
    __ISB();
    if (deadline_fault)
    {
        StopLocked();
        deadline_fault = 0U;
    }
    if (pending)
    {
        pending = 0U;
        for (i = 0U; i < 2U; ++i)
        {
            state[i].encoder_delta = delta[i];
            state[i].pwm = MotorPID_Update(&pid[i],
                (float)state[i].target_counts * COUNTS_TO_M_S,
                (float)state[i].encoder_delta * COUNTS_TO_M_S);
            if (state[i].encoder_delta >= -2 && state[i].encoder_delta <= 2)
            {
                if (state[i].target_counts > 0 && state[i].pwm > 0 &&
                    state[i].pwm < (int16_t)startup_pwm)
                    state[i].pwm = (int16_t)startup_pwm;
                else if (state[i].target_counts < 0 && state[i].pwm < 0 &&
                         state[i].pwm > -(int16_t)startup_pwm)
                    state[i].pwm = -(int16_t)startup_pwm;
            }
            Motor_SetPWM(i + 1U, state[i].pwm);
        }
    }
    if (enabled) NVIC_EnableIRQ(TIM1_UP_IRQn);
}

uint8_t MotorControl_GetState(uint8_t motor_id, MotorControl_State *result)
{
    if (motor_id < 1U || motor_id > 2U || !result) return 0U;
    *result = state[motor_id - 1U];
    return 1U;
}
