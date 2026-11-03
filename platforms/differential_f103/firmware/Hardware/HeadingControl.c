#include "HeadingControl.h"
#include "HWT101.h"
#include "MotorControl.h"
#include "Delay.h"
#include <float.h>

static HeadingControl_State state;
static float integral;
static uint32_t sequence, timestamp, faults, stopped_since;
static int8_t last_direction;
static uint8_t stopped_timing;

static float Abs(float x) { return x < 0.0f ? -x : x; }
static uint8_t Finite(float x) { return x >= -FLT_MAX && x <= FLT_MAX; }
static float Clamp(float x)
{
    if (x > HEADING_MAX_COUNTS) return HEADING_MAX_COUNTS;
    if (x < -HEADING_MAX_COUNTS) return -HEADING_MAX_COUNTS;
    return x;
}
static uint8_t Read(HWT101_Data *data)
{
    return HWT101_Read(data) && data->continuous_valid &&
        !HWT101_IsBusy() && Finite(data->continuous_deg) &&
        (uint32_t)(Delay_GetTick() - data->timestamp_ms) < HEADING_DATA_TIMEOUT_MS;
}
static void Fail(void)
{
    MotorControl_Stop();
    integral = 0.0f;
    state.mode = HEADING_FAULT;
}
void HeadingControl_Stop(void)
{
    MotorControl_Stop();
    integral = 0.0f;
    state.mode = HEADING_IDLE;
    last_direction = 0;
    stopped_timing = 0U;
}
void HeadingControl_Init(void)
{
    HeadingControl_Stop();
    state.target_deg = state.current_deg = state.error_deg = 0.0f;
    faults = MotorControl_FaultSequence();
}
uint8_t HeadingControl_Rotate(float relative_deg)
{
    HWT101_Data data;
    HWT101_Poll();
    if ((HEADING_MOTOR1_SIGN != 1 && HEADING_MOTOR1_SIGN != -1) ||
        (HEADING_MOTOR2_SIGN != 1 && HEADING_MOTOR2_SIGN != -1) ||
        !Finite(relative_deg) || !Read(&data) ||
        !Finite(data.continuous_deg + relative_deg))
    {
        Fail();
        return 0U;
    }
    MotorControl_Stop();
    integral = 0.0f;
    state.target_deg = data.continuous_deg + relative_deg;
    state.current_deg = data.continuous_deg;
    state.error_deg = relative_deg;
    state.mode = HEADING_BRAKING;
    sequence = data.sequence;
    timestamp = data.timestamp_ms;
    faults = MotorControl_FaultSequence();
    last_direction = 0;
    stopped_timing = 0U;
    return 1U;
}
uint8_t HeadingControl_Hold(void) { return HeadingControl_Rotate(0.0f); }
void HeadingControl_GetState(HeadingControl_State *result)
{
    if (result) *result = state;
}
void HeadingControl_Process(void)
{
    HWT101_Data data;
    MotorControl_State a, b;
    float dt, error, candidate, output;
    int16_t counts;
    int8_t direction;
    uint32_t now = Delay_GetTick();
    HWT101_Poll();
    if (state.mode == HEADING_IDLE || state.mode == HEADING_FAULT) return;
    if (!Read(&data) || MotorControl_FaultSequence() != faults)
    {
        Fail();
        return;
    }
    state.current_deg = data.continuous_deg;
    state.error_deg = state.target_deg - state.current_deg;
    if (state.mode == HEADING_BRAKING)
    {
        MotorControl_GetState(1U, &a);
        MotorControl_GetState(2U, &b);
        if (a.encoder_delta >= -2 && a.encoder_delta <= 2 &&
            b.encoder_delta >= -2 && b.encoder_delta <= 2)
        {
            if (!stopped_timing) { stopped_since = now; stopped_timing = 1U; }
            if ((uint32_t)(now - stopped_since) >= 100U)
            {
                state.mode = HEADING_TURNING;
                last_direction = 0;
                integral = 0.0f;
            }
        }
        else stopped_timing = 0U;
        if (state.mode == HEADING_BRAKING) return;
    }
    /* Integrate only once per new sensor frame, using its actual interval. */
    if (data.sequence == sequence) return;
    dt = (float)(uint32_t)(data.timestamp_ms - timestamp) * 0.001f;
    sequence = data.sequence;
    timestamp = data.timestamp_ms;
    if (dt <= 0.0f || dt > 0.3f) dt = 0.0f;
    error = state.error_deg;
    if (!Finite(error)) { Fail(); return; }
    if (state.mode == HEADING_HOLDING)
    {
        if (Abs(error) <= HEADING_EXIT_DEG) return;
        state.mode = HEADING_TURNING;
    }
    if (Abs(error) <= HEADING_ENTER_DEG && data.rate_valid &&
        Abs(data.yaw_rate_dps) <= HEADING_SETTLED_DPS)
    {
        MotorControl_Stop();
        integral = 0.0f;
        state.mode = HEADING_HOLDING;
        return;
    }
    candidate = Clamp(integral + HEADING_KI * error * dt);
    output = HEADING_KP * error + candidate;
    if (data.rate_valid) output -= HEADING_KD * data.yaw_rate_dps;
    if (!((output > HEADING_MAX_COUNTS && error > 0.0f) ||
          (output < -HEADING_MAX_COUNTS && error < 0.0f))) integral = candidate;
    output = HEADING_KP * error + integral;
    if (data.rate_valid) output -= HEADING_KD * data.yaw_rate_dps;
    if (!Finite(output)) { Fail(); return; }
    output = Clamp(output);
    counts = (int16_t)(output >= 0.0f ? output + 0.5f : output - 0.5f);
    direction = counts > 0 ? 1 : (counts < 0 ? -1 : 0);
    if (direction && last_direction && direction != last_direction)
    {
        MotorControl_Stop();
        state.mode = HEADING_BRAKING;
        stopped_timing = 0U;
        return;
    }
    MotorControl_SetTarget(1U, counts * HEADING_MOTOR1_SIGN);
    MotorControl_SetTarget(2U, counts * HEADING_MOTOR2_SIGN);
    if (direction) last_direction = direction;
}
