#include "ChassisControl.h"
#include "MotorEncoder.h"
#include "Bluetooth.h"
#include "HWT101.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static ChassisVelocity goal,actual;
static ChassisPose pose;
static float odom_yaw;
static float kp[4],ki[4],kd[4],integ[4],last[4],last_target[4],target[4],speed[4],pwm[4];
static uint16_t previous[4];
static const int8_t signs[4]={MOTOR_ENCODER_SIGN_A,MOTOR_ENCODER_SIGN_B,MOTOR_ENCODER_SIGN_C,MOTOR_ENCODER_SIGN_D};
static uint32_t tick,report,boot_since;
static uint8_t calibrated,command_active;
static uint32_t command_tick;
static char command[96];static unsigned length;
static uint8_t heading,locked,sensor_ready;static float yaw_ref;
static uint16_t telemetry_sequence;
static void Put16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void Put32(uint8_t *p,uint32_t v){unsigned i;for(i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
static void PutFloat(uint8_t *p,float v){uint32_t bits;memcpy(&bits,&v,4);Put32(p,bits);}
static uint16_t PacketCRC(const uint8_t *p,unsigned size){
 uint16_t crc=0xFFFFU;unsigned i;
 while(size--){crc^=*p++;for(i=0;i<8;i++)crc=(uint16_t)((crc>>1)^((crc&1U)?0xA001U:0U));}
 return crc;
}
static void SendPose(uint32_t now){
 uint8_t packet[24];
 packet[0]=0xAA;packet[1]=0x55;packet[2]=1;packet[3]=24;
 /* Increment per scheduled attempt, including busy drops, so gaps are visible. */
 Put16(packet+4,telemetry_sequence++);Put32(packet+6,now);
 if(pose.valid){
  PutFloat(packet+10,pose.x_m);PutFloat(packet+14,pose.y_m);
  PutFloat(packet+18,pose.yaw_rad/0.017453293f);
 }else{
  Put32(packet+10,0x7FC00000UL);Put32(packet+14,0x7FC00000UL);Put32(packet+18,0x7FC00000UL);
 }
 Put16(packet+22,PacketCRC(packet,22));
 /* SendAsync copies all 24 bytes into its persistent TX buffer. */
 (void)Bluetooth_SendAsync((const char *)packet,sizeof(packet));
}
static float Clip(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
uint8_t Chassis_SetVelocity(float x,float y,float z){
 /* Reject non-finite/unreasonable transport inputs without altering last command. */
 if(!calibrated)return 0;
 if(!(x>=-10&&x<=10&&y>=-10&&y<=10&&z>=-20&&z<=20))return 0;
 goal.x=x;goal.y=y;goal.z=z;
 command_tick=HAL_GetTick();command_active=1;return 1;
}
void Chassis_GetMeasuredVelocity(ChassisVelocity *v){*v=actual;}
void Chassis_GetPose(ChassisPose *value){if(value)*value=pose;}
/* Encoder displacement is rotated into the boot-zero frame by IMU yaw.
 * Invalidate permanently after a sensor gap: missing travel cannot be recovered. */
static void UpdateOdometry(float dt,uint32_t now){
 HWT101_Data sample;float angle,mid,c,s;
 if(!pose.valid)return;
 if(!sensor_ready||!HWT101_Read(&sample)||!sample.continuous_valid||
    now-sample.timestamp_ms>=300U){pose.valid=0;return;}
 angle=sample.heading_deg;
 mid=(odom_yaw+0.5f*HWT101_AngleError(angle,odom_yaw))*0.017453293f;
 c=cosf(mid);s=sinf(mid);
 pose.x_m+=(actual.x*c-actual.y*s)*dt;
 pose.y_m+=(actual.x*s+actual.y*c)*dt;
 pose.yaw_rad=angle*0.017453293f;odom_yaw=angle;
}
void Chassis_EnableHeadingAssist(uint8_t en){heading=en?1:0;locked=0;}
uint8_t WheelPID_SetGains(unsigned w,float p,float i,float d){
 if(w>=4||!(p>=0&&p<=10&&i>=0&&i<=20&&d>=0&&d<=1))return 0;
 kp[w]=p;ki[w]=i;kd[w]=d;integ[w]=0;return 1;
}
static void Parse(void){
 char *s=command+3,*end;float v[3];unsigned i;
 if(strncmp(command,"[v,",3))return;
 for(i=0;i<3;i++){v[i]=(float)strtod(s,&end);if(end==s||*end!=(i==2?']':','))return;s=end+1;}
 if(!*s)(void)Chassis_SetVelocity(v[0],v[1],v[2]);
}
HAL_StatusTypeDef Chassis_Init(void){
 unsigned i;Motor_StopAll();memset(&goal,0,sizeof(goal));memset(&actual,0,sizeof(actual));
 memset(&pose,0,sizeof(pose));odom_yaw=0;
 telemetry_sequence=0;calibrated=0;command_active=0;command_tick=0;length=locked=0;heading=CHASSIS_HEADING_DEFAULT;
 if(MotorEncoder_Init()!=HAL_OK)return HAL_ERROR;
 if(Bluetooth_Init()!=HAL_OK)return HAL_ERROR;
 sensor_ready=HWT101_Init()==HAL_OK;
 for(i=0;i<4;i++){kp[i]=CHASSIS_KP;ki[i]=CHASSIS_KI;kd[i]=CHASSIS_KD;integ[i]=last[i]=last_target[i]=target[i]=speed[i]=pwm[i]=0;previous[i]=MotorEncoder_ReadRaw((MotorId)i);}
 boot_since=tick=report=HAL_GetTick();return HAL_OK;
}
void Chassis_Update(void){
 unsigned i,budget=128;uint8_t b;int rc;uint32_t now,elapsed;
 WheelVelocity desired,measured;ChassisVelocity request;HWT101_Data imu;
 float dt,e,candidate,u,d,correction,lo,hi;uint16_t raw;int32_t delta;
 if(sensor_ready)HWT101_Poll();
 while(budget--){
  rc=Bluetooth_ReadByte(&b);if(!rc)break;if(rc<0){length=0;break;}
  if(b=='[')length=0;else if(!length)continue;
  if(length>=sizeof(command)-1){length=0;continue;}
  command[length++]=(char)b;if(b==']'){command[length]=0;Parse();length=0;}
 }
 now=HAL_GetTick();
 if(!calibrated){
  Motor_StopAll();length=0;
  if(now-boot_since<5000U)return;
  /* Motors remain stationary: reset counters and all controller state once. */
  if(MotorEncoder_Init()!=HAL_OK){Error_Handler();return;}
  memset(&goal,0,sizeof(goal));memset(&actual,0,sizeof(actual));
  for(i=0;i<4;i++){
   previous[i]=MotorEncoder_ReadRaw((MotorId)i);
   integ[i]=last[i]=last_target[i]=target[i]=speed[i]=pwm[i]=0;
  }
  locked=0;yaw_ref=0;
  memset(&pose,0,sizeof(pose));odom_yaw=0;
  if(sensor_ready&&HWT101_Read(&imu)&&now-imu.timestamp_ms<300U)
   pose.valid=HWT101_Zero();
  /* Discard commands received during startup; require a fresh command. */
  budget=256;while(budget-- && Bluetooth_ReadByte(&b)>0){}
  tick=report=HAL_GetTick();calibrated=1;return;
 }
 /* Only complete, valid velocity commands keep motion alive. */
 if(command_active&&(uint32_t)(now-command_tick)>=CHASSIS_COMMAND_TIMEOUT_MS){
  command_active=0;memset(&goal,0,sizeof(goal));locked=0;
  for(i=0;i<4;i++)integ[i]=last_target[i]=target[i]=pwm[i]=0;
  Motor_StopAll();
 }
 elapsed=now-tick;
 if(elapsed>=10){
  tick=now;dt=(float)elapsed/1000;request=goal;
  if(command_active&&heading&&sensor_ready&&HWT101_Read(&imu)&&imu.continuous_valid&&imu.rate_valid&&now-imu.timestamp_ms<300){
   if(fabsf(goal.z)>0.0001f){locked=0;correction=CHASSIS_RATE_KP*(goal.z-imu.yaw_rate_dps*0.017453293f);}
   else{if(!locked){yaw_ref=imu.continuous_deg;locked=1;}correction=CHASSIS_HEADING_KP*HWT101_AngleError(yaw_ref,imu.continuous_deg)*0.017453293f;}
   request.z+=Clip(correction,-CHASSIS_HEADING_MAX,CHASSIS_HEADING_MAX);
  }else locked=0;
  Mecanum_Inverse(&request,&desired);Mecanum_Limit(&desired,CHASSIS_WHEEL_MAX_MPS);
  for(i=0;i<4;i++){
   raw=MotorEncoder_ReadRaw((MotorId)i);delta=MotorEncoder_Delta(previous[i],raw)*signs[i];previous[i]=raw;
   speed[i]=(float)delta/dt;measured.v[i]=Mecanum_CountsToMps(speed[i]);target[i]=Mecanum_MpsToCounts(desired.v[i]);
   /* Match A tuning: no active reverse torque for a same-direction goal.
    * Mirror the bounds for reverse travel and reset I on command reversal. */
   if(target[i]*last_target[i]<0)integ[i]=0;
   last_target[i]=target[i];
   lo=target[i]<0?-1000.0f:0.0f;hi=target[i]>0?1000.0f:0.0f;
   if(fabsf(target[i])<0.01f){integ[i]=0;pwm[i]=0;}
   else{
    e=target[i]-speed[i];d=-kd[i]*(speed[i]-last[i])/dt;
    candidate=integ[i]+ki[i]*e*dt;u=kp[i]*e+candidate+d;
    if(!((u>hi&&e>0)||(u<lo&&e<0)))integ[i]=Clip(candidate,-1000,1000);
    pwm[i]=Clip(kp[i]*e+integ[i]+d,lo,hi);
   }
   last[i]=speed[i];(void)Motor_Set((MotorId)i,(int32_t)pwm[i]);
  }
  Mecanum_Forward(&measured,&actual);
  UpdateOdometry(dt,now);
 }
 if(now-report>=CHASSIS_TELEMETRY_PERIOD_MS){
  SendPose(now);
  report=now;
 }
}
