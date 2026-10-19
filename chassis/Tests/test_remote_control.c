#include <assert.h>
#include <stdio.h>
#include "RemoteControl.h"
#include "MotorControl.h"
#include "HWT101.h"
static uint32_t now, faults;
static HWT101_Data imu;
static int16_t targets[2];
static uint8_t advance_poll;
uint32_t Delay_GetTick(void) { return now; }
void HWT101_Poll(void)
{
    if (advance_poll) { now++; imu.timestamp_ms=now; imu.sequence++; }
}
uint8_t HWT101_IsBusy(void) { return 0; }
uint8_t HWT101_Read(HWT101_Data *d) { *d=imu; return imu.valid; }
uint32_t MotorControl_FaultSequence(void) { return faults; }
void MotorControl_Stop(void) { targets[0]=targets[1]=0; }
void MotorControl_SetTarget(uint8_t id,int16_t t) { targets[id-1]=t; }
uint8_t MotorControl_GetState(uint8_t id,MotorControl_State *s)
{ (void)id; s->encoder_delta=0; return 1; }
static void step(void)
{
    now+=20; imu.timestamp_ms=now; imu.sequence++;
    RemoteControl_Process();
}
static void send(const char *p)
{
    while (*p) { RemoteControl_Feed((uint8_t)*p++); RemoteControl_Process(); }
}
static void drive(const char *p)
{ int i; for(i=0;i<15;i++) { send(p); step(); } }
int main(void)
{
    imu.valid=imu.continuous_valid=imu.rate_valid=1;
    RemoteControl_Init();
    /* No neutral packet is sent by the app on initial connection. */
    drive("[j, -1, 100, 0, 0]"); assert(targets[0]>0 && targets[0]==targets[1]);
    send("[joystick,0,0,0,0]");
    drive("[j,0,100,0,0]"); assert(targets[0]>0 && targets[0]==targets[1]);
    imu.continuous_deg=5; step(); assert(targets[0]>targets[1]);
    send("[j,0,0,0,0]"); step(); assert(targets[0]==0 && targets[1]==0);
    drive("[j,0,0,100,0]"); assert(targets[0]>0 && targets[1]<0);
    /* Malformed/out-of-range frames must not refresh watchdog. */
    { int i; for(i=0;i<20;i++) { send("[j,0,101,0,0]"); step(); } }
    #if REMOTE_TIMEOUT_MS > 0U
    assert(targets[0]==0 && targets[1]==0);
    #else
    assert(targets[0]>0 && targets[1]<0); /* Bad frames do not replace command. */
    RemoteControl_RxError();
    #endif
    drive("[j,0,100,0,0]"); assert(targets[0]==0);
    send("[j,0,0,0,0]"); drive("[j,0,-100,0,0]");
    assert(targets[0]<0 && targets[1]<0);
    faults++; step(); assert(targets[0]==0 && targets[1]==0);
    /* Spaced long format, concatenated frames and tab whitespace. */
    send("[ joystick , 0 , 0 , 0 , 0 ][j,0,0,0,0]");
    drive("[ joystick ,\t0, +100 , 0 , 0 ]");
    assert(targets[0]>0 && targets[1]>0);
    /* Whitespace inside a number, missing/extra fields and garbage rejected. */
    { int i; for(i=0;i<20;i++) {
        send("[j,0,1 00,0,0][junk,0,100,0,0][j,0,100,0][j,0,100,0,0,0]");
        step();
    } }
    #if REMOTE_TIMEOUT_MS > 0U
    assert(targets[0]==0 && targets[1]==0);
    #else
    assert(targets[0]>0 && targets[1]>0);
    #endif
    /* An angle may arrive across a millisecond boundary inside Poll. */
    RemoteControl_Init();
    advance_poll=1U;
    drive("[j, 0, 100, 0, 0]");
    assert(targets[0]>0 && targets[1]>0);
    #if REMOTE_TIMEOUT_MS == 0U
    /* Holding a joystick can mean no further packets at all. */
    { int i; for(i=0;i<100;i++) step(); }
    assert(targets[0]>0 && targets[1]>0);
    send("[j, 0, 0, 0, 0]");
    assert(targets[0]==0 && targets[1]==0);
    /* Sensor failure must still stop despite command-hold mode. */
    drive("[j,0,100,0,0]");
    imu.valid=0; step();
    assert(targets[0]==0 && targets[1]==0);
    #endif
    puts("Remote control tests passed");
    return 0;
}
