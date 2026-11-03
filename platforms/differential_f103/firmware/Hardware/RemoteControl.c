#include "RemoteControl.h"
#include "MotorControl.h"
#include "HWT101.h"
#include "HeadingControl.h"
#include "Delay.h"
#include <string.h>

static char frame[64];
static uint8_t length, receiving, armed, active, braking, quiet;
static int16_t throttle, steering;
static int8_t direction[2];
static uint32_t packet_ms, byte_ms, tick_ms, quiet_ms, faults, seq, angle_ms;
static float heading, speed, integral, correction;
static uint8_t was_turning;
static uint8_t packet_seen;
static uint8_t first_command;
volatile RemoteControl_Debug remote_debug;

void RemoteControl_RxError(void)
{
    remote_debug.uart_errors++;
    RemoteControl_Stop();
    receiving=length=0U;
}

static float Abs(float v) { return v < 0 ? -v : v; }
static float Limit(float v, float limit)
{ return v > limit ? limit : (v < -limit ? -limit : v); }
static float Axis(int16_t v)
{
    if (v > REMOTE_DEADZONE) return (float)(v-REMOTE_DEADZONE)/(100-REMOTE_DEADZONE);
    if (v < -REMOTE_DEADZONE) return (float)(v+REMOTE_DEADZONE)/(100-REMOTE_DEADZONE);
    return 0.0f;
}
static void Halt(void)
{
    MotorControl_Stop();
    active = braking = quiet = was_turning = 0U;
    speed = integral = correction = 0.0f;
    /* Retain direction until measured wheel speed is zero before restart. */
}
void RemoteControl_Stop(void)
{
    Halt();
    armed = 0U;
    throttle = steering = 0;
    packet_seen = 0U;
}
void RemoteControl_Init(void)
{
    RemoteControl_Stop();
    first_command = 1U;
    memset((void *)&remote_debug, 0, sizeof(remote_debug));
    receiving = length = 0U;
    direction[0] = direction[1] = 0;
    tick_ms = packet_ms = byte_ms = Delay_GetTick();
    faults = MotorControl_FaultSequence();
}
static char *SkipSpace(char *p)
{
    while (*p==' ' || *p=='\t') ++p;
    return p;
}
static uint8_t Parse(void)
{
    char *p;
    int16_t value[4];
    int sign, n;
    uint8_t i;
    p=SkipSpace(frame);
    if (strncmp(p, "joystick", 8U) == 0) p+=8;
    else if (*p=='j') ++p;
    else return 0U;
    p=SkipSpace(p);
    if (*p!=',') return 0U;
    ++p;
    for (i=0U; i<4U; ++i)
    {
        sign=1; n=0;
        p=SkipSpace(p);
        if (*p=='-') { sign=-1; p++; }
        else if (*p=='+') p++;
        if (*p<'0' || *p>'9') return 0U;
        while (*p>='0' && *p<='9')
        {
            n=n*10+(*p++-'0');
            if (n>100) return 0U;
        }
        value[i]=(int16_t)(sign*n);
        p=SkipSpace(p);
        if (i<3U) { if (*p++!=',') return 0U; }
        else if (*p!='\0') return 0U;
    }
    throttle=value[1]; steering=value[2];
    return 1U;
}
void RemoteControl_Feed(uint8_t byte)
{
    RemoteControl_FeedAt(byte,Delay_GetTick());
}
void RemoteControl_FeedAt(uint8_t byte, uint32_t now)
{
    remote_debug.rx_bytes++;
    if (receiving && (uint32_t)(now-byte_ms)>100U) receiving=0U;
    byte_ms=now;
    if (byte=='[') { receiving=1U; length=0U; return; }
    if (!receiving) return;
    if (byte==']')
    {
        receiving=0U; frame[length]='\0';
        if (Parse())
        {
            packet_ms=now; packet_seen=1U;
            remote_debug.valid_packets++;
            remote_debug.throttle=throttle;
            remote_debug.steering=steering;
        }
        else remote_debug.rejected_packets++;
        return;
    }
    if ((byte<32U && byte!='\t') || byte>126U || length>=sizeof(frame)-1U)
    { receiving=0U; return; }
    frame[length++]=(char)byte;
}
void RemoteControl_Process(void)
{
    HWT101_Data imu;
    MotorControl_State wheel[2];
    uint32_t now, elapsed;
    float forward, turn, desired, step, dt, error, candidate, out, left, right, peak;
    int16_t command[2];
    int8_t sign[2];
    uint8_t i, imu_valid;
    HWT101_Poll();
    imu_valid=HWT101_Read(&imu);
    /* Read time AFTER Poll/Read: a newly arrived frame can have a timestamp
     * later than the time at function entry. Unsigned subtraction would then
     * wrap and falsely trigger the stale-data stop. */
    now=Delay_GetTick();
    remote_debug.packet_age_ms=now-packet_ms;
    remote_debug.imu_age_ms=now-imu.timestamp_ms;
    if (
#if REMOTE_TIMEOUT_MS > 0U
        (uint32_t)(now-packet_ms)>=REMOTE_TIMEOUT_MS ||
#endif
        !imu_valid || !imu.continuous_valid || HWT101_IsBusy() ||
        (uint32_t)(now-imu.timestamp_ms)>=HEADING_DATA_TIMEOUT_MS ||
        MotorControl_FaultSequence()!=faults)
    {
        if (MotorControl_FaultSequence()!=faults) remote_debug.state=5U;
        else if (!imu_valid || HWT101_IsBusy() ||
            (uint32_t)(now-imu.timestamp_ms)>=HEADING_DATA_TIMEOUT_MS) remote_debug.state=3U;
        else if (!imu.continuous_valid) remote_debug.state=4U;
        else remote_debug.state=1U;
        RemoteControl_Stop();
        faults=MotorControl_FaultSequence();
        return;
    }
    forward=Axis(throttle); turn=Axis(steering);
    if (!armed)
    {
        /* App does not send a neutral packet on connection. Accept the first
         * valid command at boot; after faults require an explicit neutral. */
        if (packet_seen && (first_command || (forward==0.0f && turn==0.0f)))
        {
            armed=1U;
            first_command=0U;
        }
        else { remote_debug.state=2U; return; }
    }
    if (forward==0.0f && turn==0.0f) { Halt(); remote_debug.state=0U; return; }
    elapsed=now-tick_ms;
    if (elapsed<20U) return;
    tick_ms=now;
    dt=(float)(elapsed>40U ? 20U : elapsed)*0.001f;
    for (i=0U;i<2U;++i) MotorControl_GetState(i+1U,&wheel[i]);
    if (!active)
    {
        active=braking=1U; quiet=0U;
        heading=imu.continuous_deg;
    }
    if (braking)
    {
        remote_debug.state=6U;
        MotorControl_Stop();
        if (Abs((float)wheel[0].encoder_delta)<=2.0f && Abs((float)wheel[1].encoder_delta)<=2.0f)
        {
            if (!quiet) { quiet=1U; quiet_ms=now; }
            if ((uint32_t)(now-quiet_ms)<100U) return;
        }
        else { quiet=0U; return; }
        braking=0U; direction[0]=direction[1]=0;
        heading=imu.continuous_deg; speed=integral=correction=0.0f;
        seq=imu.sequence-1U; angle_ms=imu.timestamp_ms;
    }
    desired=forward*REMOTE_MAX_SPEED_M_S;
    step=REMOTE_ACCEL_M_S2*dt;
    speed+=Limit(desired-speed,step);
    if (turn!=0.0f)
    {
        heading-=turn*REMOTE_MAX_TURN_DPS*dt; /* Right = clockwise = negative yaw. */
        heading=imu.continuous_deg+Limit(heading-imu.continuous_deg,REMOTE_MAX_LEAD_DEG);
        was_turning=1U;
    }
    else if (was_turning)
    {
        /* Capture after rotation settles, rather than chase an old lead angle. */
        heading=imu.continuous_deg; integral=0.0f;
        if (imu.rate_valid && Abs(imu.yaw_rate_dps)<=HEADING_SETTLED_DPS) was_turning=0U;
    }
    if (imu.sequence!=seq)
    {
        dt=(float)(uint32_t)(imu.timestamp_ms-angle_ms)*0.001f;
        if (dt>0.3f) dt=0.0f;
        seq=imu.sequence; angle_ms=imu.timestamp_ms;
        error=heading-imu.continuous_deg;
        candidate=Limit(integral+HEADING_KI*error*dt,HEADING_MAX_COUNTS);
        out=HEADING_KP*error+candidate;
        if (imu.rate_valid) out-=HEADING_KD*imu.yaw_rate_dps;
        if (!((out>HEADING_MAX_COUNTS && error>0) || (out<-HEADING_MAX_COUNTS && error<0))) integral=candidate;
        out=HEADING_KP*error+integral;
        if (imu.rate_valid) out-=HEADING_KD*imu.yaw_rate_dps;
        correction=Limit(out,HEADING_MAX_COUNTS);
    }
    left=speed/0.004f-correction; right=speed/0.004f+correction;
    peak=Abs(left)>Abs(right) ? Abs(left) : Abs(right);
    if (peak>REMOTE_WHEEL_LIMIT_COUNTS)
    { left*=REMOTE_WHEEL_LIMIT_COUNTS/peak; right*=REMOTE_WHEEL_LIMIT_COUNTS/peak; }
    command[0]=(int16_t)left; command[1]=(int16_t)right;
    for (i=0U;i<2U;++i)
    {
        sign[i]=command[i]>0 ? 1 : (command[i]<0 ? -1 : 0);
        if (sign[i] && direction[i] && sign[i]!=direction[i])
        { MotorControl_Stop(); braking=1U; quiet=0U; return; }
    }
    for (i=0U;i<2U;++i)
    {
        MotorControl_SetTarget(i+1U,command[i]);
        if (sign[i]) direction[i]=sign[i];
    }
    remote_debug.state=7U;
}
