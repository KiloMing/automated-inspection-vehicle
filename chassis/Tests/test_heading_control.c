#include <assert.h>
#include <stdio.h>
#include "HeadingControl.h"
#include "HWT101.h"
#include "MotorControl.h"
static uint32_t now, fault;
static HWT101_Data sensor;
static int16_t target[2];
uint32_t Delay_GetTick(void) { return now; }
void HWT101_Poll(void) {}
uint8_t HWT101_IsBusy(void) { return 0; }
uint8_t HWT101_Read(HWT101_Data *d) { *d = sensor; return sensor.valid; }
uint32_t MotorControl_FaultSequence(void) { return fault; }
void MotorControl_Stop(void) { target[0] = target[1] = 0; }
void MotorControl_SetTarget(uint8_t id, int16_t value) { target[id-1] = value; }
uint8_t MotorControl_GetState(uint8_t id, MotorControl_State *s)
{ (void)id; s->encoder_delta=0; return 1; }
static void frame(float angle, uint32_t time)
{
    now=time; sensor.timestamp_ms=time; sensor.continuous_deg=angle;
    sensor.sequence++; sensor.valid=sensor.continuous_valid=sensor.rate_valid=1;
    sensor.yaw_rate_dps=0;
    HeadingControl_Process();
}
int main(void)
{
    HeadingControl_State s;
    HeadingControl_Init();
    frame(179,0);
    assert(HeadingControl_Rotate(90));
    frame(179,1); frame(179,101);
    assert(target[0]==-50 && target[1]==50);
    frame(269,201);
    HeadingControl_GetState(&s);
    assert(s.mode==HEADING_HOLDING && target[0]==0);
    frame(259,301);
    assert(target[0]<0 && target[1]>0);
    frame(279,401);
    HeadingControl_GetState(&s);
    assert(s.mode==HEADING_BRAKING && target[0]==0);
    frame(279,402); frame(279,502);
    assert(target[0]>0 && target[1]<0);
    now=803; HeadingControl_Process();
    HeadingControl_GetState(&s);
    assert(s.mode==HEADING_FAULT && target[0]==0 && target[1]==0);
    frame(279,804);
    HeadingControl_GetState(&s); assert(s.mode==HEADING_FAULT);
    assert(HeadingControl_Hold());
    fault++; HeadingControl_Process();
    HeadingControl_GetState(&s); assert(s.mode==HEADING_FAULT);
    HeadingControl_Stop();
    sensor.valid=0;
    assert(!HeadingControl_Rotate(90));
    puts("Heading control tests passed");
    return 0;
}
