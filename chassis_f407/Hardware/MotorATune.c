#include "MotorATune.h"
#include "MotorEncoder.h"
#include "Bluetooth.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <float.h>
static float kp=MOTOR_A_PID_KP,ki=MOTOR_A_PID_KI,kd=MOTOR_A_PID_KD,goal,measured,integral,previous_speed;
static int32_t output;
static uint32_t tick,report,last_motion;
static uint16_t previous;
static unsigned fault,length;
static char command[80];
static void Halt(void){goal=integral=0;output=0;Motor_StopAll();}
void MotorATune_Stop(void){Halt();fault=0;}
uint8_t MotorATune_SetPID(float p,float i,float d){
 if(!(p>=0&&p<=10&&i>=0&&i<=20&&d>=0&&d<=1))return 0;
 kp=p;ki=i;kd=d;integral=0;return 1;
}
uint8_t MotorATune_SetTarget(float v){
 /* No application speed cap; reject negative, NaN and infinity. */
 if(!(v>=0&&v<=FLT_MAX)||fault)return 0;
 if(v==0){Halt();return 1;}
 if(goal==0)last_motion=HAL_GetTick();goal=v;return 1;
}
static uint8_t Numbers(char *s,float *v,unsigned n){
 unsigned a;char *end;
 for(a=0;a<n;a++){
  v[a]=(float)strtod(s,&end);
  if(end==s||*end!=(a+1==n?']':','))return 0;s=end+1;
 }
 return *s==0;
}
static void Command(void){
 float v[3];uint8_t ok=0;
 if(!strcmp(command,"[stop]")){MotorATune_Stop();return;}
 if(!strncmp(command,"[v,",3)&&Numbers(command+3,v,1))ok=MotorATune_SetTarget(v[0]);
 if(!ok){fault=5;Halt();}
}
HAL_StatusTypeDef MotorATune_Init(void){
 Motor_StopAll();goal=measured=integral=previous_speed=0;output=0;fault=length=0;
 if(MotorEncoder_Init()!=HAL_OK)return HAL_ERROR;
 if(Bluetooth_Init()!=HAL_OK)return HAL_ERROR;
 previous=MotorEncoder_ReadRaw(MOTOR_LF);tick=report=last_motion=HAL_GetTick();return HAL_OK;
}
void MotorATune_Update(void){
 uint32_t now=HAL_GetTick(),elapsed;unsigned budget=128;uint8_t byte;int received;
 uint16_t raw;int32_t delta;float dt,e,candidate,u,d;
 char line[112];int n;
 while(budget--){
  received=Bluetooth_ReadByte(&byte);if(!received)break;
  if(received<0){fault=1;Halt();length=0;break;}
  if(byte=='[')length=0;else if(!length)continue;
  if(length>=sizeof(command)-1){fault=5;Halt();length=0;continue;}
  command[length++]=(char)byte;
  if(byte==']'){command[length]=0;Command();length=0;}
 }
 /* Commands may refresh last_motion after the entry timestamp. */
 now=HAL_GetTick();
 elapsed=now-tick;
 if(elapsed>=10){
  raw=MotorEncoder_ReadRaw(MOTOR_LF);delta=MotorEncoder_Delta(previous,raw)*MOTOR_ENCODER_SIGN_A;
  previous=raw;tick=now;dt=(float)elapsed/1000.0f;measured=(float)delta/dt;
  if(delta>0)last_motion=now;
  if(goal>0&&elapsed>30){fault=2;Halt();}
  if(goal>0&&delta< -2){fault=3;Halt();}
  if(goal>0&&now-last_motion>=1000){fault=4;Halt();}
  if(goal>0&&!fault){
   e=goal-measured;d=-kd*(measured-previous_speed)/dt;
   candidate=integral+ki*e*dt;
   u=kp*e+candidate+d;
   if(!((u>MOTOR_A_PWM_MAX&&e>0)||(u<0&&e<0)))integral=candidate;
   if(integral>MOTOR_A_PWM_MAX)integral=MOTOR_A_PWM_MAX;if(integral< -MOTOR_A_PWM_MAX)integral=-MOTOR_A_PWM_MAX;
   u=kp*e+integral+d;if(u>MOTOR_A_PWM_MAX)u=MOTOR_A_PWM_MAX;if(u<0)u=0;
   output=(int32_t)u;
   if(Motor_Set(MOTOR_LF,output)!=HAL_OK){fault=6;Halt();}
  }
  previous_speed=measured;
 }
 if(now-report>=50){
  /* Fixed-point decimal PWM avoids printf float-library dependency. */
  n=snprintf(line,sizeof(line),"%.0f,%ld,%ld.%ld\r\n",
   (double)goal,(long)measured,(long)(output/10),(long)(output%10));
  if(n<=0||n>=(int)sizeof(line)||Bluetooth_SendText(line,(uint16_t)n)!=HAL_OK){fault=1;Halt();}
  report=now;
 }
}
