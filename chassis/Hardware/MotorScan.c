#include "stm32f10x.h"
#include "MotorScan.h"
#include "Motor.h"
#include "Delay.h"
#define SAMPLE_QUEUE_SIZE 16U
static volatile MotorScan_Sample queue[SAMPLE_QUEUE_SIZE];
static volatile uint8_t head;
static volatile uint8_t tail;
static volatile uint8_t overflow;
static volatile uint32_t dropped;
static volatile uint16_t applied_pwm;
static volatile uint8_t applied_state;
static volatile uint32_t changed_ms;
static uint16_t previous_counter;
static uint32_t previous_ms;
static uint32_t start_ms;
static MotorScan_Core scan;

/* Called with interrupts masked; no concurrent sample can get mismatched labels. */
static void MotorScan_Apply(void)
{
    Motor_SetPWM(MOTOR_2, (int16_t)scan.pwm);
    applied_pwm = scan.pwm;
    applied_state = scan.state;
    changed_ms = Delay_GetTick();
    scan.step_ms = changed_ms;
}

void MotorScan_Init(void)
{
    TIM_TimeBaseInitTypeDef base;
    RCC_ClocksTypeDef clocks;
    NVIC_InitTypeDef irq;
    uint32_t clock_hz;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_DeInit(TIM1);
    RCC_GetClocksFreq(&clocks);
    clock_hz = clocks.PCLK2_Frequency;
    if (clocks.PCLK2_Frequency != clocks.HCLK_Frequency) clock_hz *= 2U;
    TIM_TimeBaseStructInit(&base);
    base.TIM_Prescaler = (uint16_t)(clock_hz / 10000U - 1U);
    base.TIM_Period = 199U; /* 10 kHz / 200 = 50 Hz, one sample every 20 ms. */
    TIM_TimeBaseInit(TIM1, &base);
    scan.state = SCAN_IDLE;
    scan.pwm = 0U;
    Motor_Brake(MOTOR_1);
    Motor_Brake(MOTOR_2);
    applied_state = SCAN_IDLE;
    applied_pwm = 0U;
    head = tail = overflow = 0U;
    dropped = 0U;
    previous_counter = TIM_GetCounter(TIM3);
    previous_ms = start_ms = changed_ms = Delay_GetTick();
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

void MotorScan_Start(void)
{
    uint32_t mask;
    /* Ignore repeat starts while running: a repeated key must not restart ramp. */
    if (scan.state == SCAN_RUNNING) return;
    mask = __get_PRIMASK();
    __disable_irq();
    MotorScan_CoreStart(&scan, Delay_GetTick());
    MotorScan_Apply();
    start_ms = previous_ms = Delay_GetTick();
    previous_counter = TIM_GetCounter(TIM3);
    TIM_SetCounter(TIM1, 0U);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    head = tail = overflow = 0U;
    dropped = 0U;
    __set_PRIMASK(mask);
}

void MotorScan_Stop(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    MotorScan_CoreStop(&scan, SCAN_STOPPED);
    MotorScan_Apply();
    /* Do not print queued pre-stop RUNNING samples after the stop command. */
    tail = head;
    overflow = 0U;
    __set_PRIMASK(mask);
}

void MotorScan_Poll(void)
{
    uint16_t old_pwm;
    uint8_t old_state;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (overflow)
    {
        MotorScan_CoreStop(&scan, SCAN_OVERFLOW);
        MotorScan_Apply();
        tail = head;
        overflow = 0U;
    }
    old_pwm = scan.pwm;
    old_state = scan.state;
    MotorScan_CoreUpdate(&scan, Delay_GetTick());
    if (old_pwm != scan.pwm || old_state != scan.state) MotorScan_Apply();
    __set_PRIMASK(mask);
}

void MotorScan_SampleIRQ(void)
{
    uint16_t current;
    uint32_t now;
    uint32_t dt;
    int32_t delta;
    uint8_t next;
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == RESET) return;
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    current = TIM_GetCounter(TIM3);
    now = Delay_GetTick();
    dt = now - previous_ms;
    delta = MotorScan_Delta(current, previous_counter);
    previous_counter = current;
    previous_ms = now;
    next = (uint8_t)((head + 1U) % SAMPLE_QUEUE_SIZE);
    if (next == tail)
    {
        dropped++;
        overflow = 1U;
        /* Brake immediately even if main is stalled in output. */
        TIM_SetCompare2(TIM4, 0U);
        applied_pwm = 0U;
        applied_state = SCAN_OVERFLOW;
        return;
    }
    queue[head].time_ms = now - start_ms;
    queue[head].dt_ms = dt;
    queue[head].delta = delta;
    queue[head].pwm = applied_pwm;
    queue[head].state = applied_state;
    queue[head].dropped = dropped;
    /* Entire measurement window must be after the settling interval. */
    queue[head].steady = (uint8_t)(applied_state == SCAN_RUNNING &&
        (uint32_t)(now - changed_ms) >= SCAN_SETTLE_MS + dt);
    head = next;
}

uint8_t MotorScan_Read(MotorScan_Sample *sample)
{
    uint32_t mask;
    if (!sample) return 0U;
    mask = __get_PRIMASK();
    __disable_irq();
    if (tail == head)
    {
        __set_PRIMASK(mask);
        return 0U;
    }
    *sample = queue[tail];
    tail = (uint8_t)((tail + 1U) % SAMPLE_QUEUE_SIZE);
    __set_PRIMASK(mask);
    return 1U;
}
