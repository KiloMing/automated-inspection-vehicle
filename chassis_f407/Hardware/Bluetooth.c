#include "Bluetooth.h"
#include "motor.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
static UART_HandleTypeDef uart;
static uint8_t rx;
static volatile uint8_t queue[256];
static volatile uint32_t head,tail,error;
static char frame[80];
static unsigned length;
static int32_t target[4];
static int8_t previous_sign[4];
static uint8_t braking;
static uint32_t brake_since;
volatile BluetoothStatus g_bluetooth;
static void Stop(void){memset(target,0,sizeof(target));Motor_StopAll();}
static HAL_StatusTypeDef Init(uint8_t receive){
 GPIO_InitTypeDef g={0};
 __HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_USART3_CLK_ENABLE();
 g.Pin=GPIO_PIN_10|GPIO_PIN_11;g.Mode=GPIO_MODE_AF_PP;g.Pull=GPIO_PULLUP;
 g.Speed=GPIO_SPEED_FREQ_HIGH;g.Alternate=GPIO_AF7_USART3;HAL_GPIO_Init(GPIOC,&g);
 uart.Instance=USART3;uart.Init.BaudRate=115200;uart.Init.WordLength=UART_WORDLENGTH_8B;
 uart.Init.StopBits=UART_STOPBITS_1;uart.Init.Parity=UART_PARITY_NONE;
 uart.Init.Mode=receive?UART_MODE_TX_RX:UART_MODE_TX;uart.Init.HwFlowCtl=UART_HWCONTROL_NONE;
 uart.Init.OverSampling=UART_OVERSAMPLING_16;
 if(HAL_UART_Init(&uart)!=HAL_OK)return HAL_ERROR;
 head=tail=error=0;length=0;braking=0;memset(target,0,sizeof(target));memset(previous_sign,0,sizeof(previous_sign));
 if(!receive)return HAL_OK;
 HAL_NVIC_SetPriority(USART3_IRQn,3,0);HAL_NVIC_EnableIRQ(USART3_IRQn);
 return HAL_UART_Receive_IT(&uart,&rx,1);
}
HAL_StatusTypeDef Bluetooth_Init(void){return Init(1);}
HAL_StatusTypeDef Bluetooth_InitTxOnly(void){return Init(0);}
HAL_StatusTypeDef Bluetooth_SendText(const char *s,uint16_t n){return HAL_UART_Transmit(&uart,(uint8_t*)s,n,30);}
void USART3_IRQHandler(void){HAL_UART_IRQHandler(&uart);}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h){
 uint32_t next;if(h!=&uart)return;next=(head+1)&255;
 if(next==tail)error=1;else{queue[head]=rx;head=next;}
 if(HAL_UART_Receive_IT(&uart,&rx,1)!=HAL_OK)error=1;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h){if(h==&uart)error=1;}
/* Decimal joystick fields in [-100,100]. Reject trailing data and NaN. */
static uint8_t Parse(void){
 float v[4];char *s=frame+3,*end;unsigned i;int32_t x,y,turn,max=1200;
 if(strncmp(frame,"[j,",3))return 0;
 for(i=0;i<4;i++){
  v[i]=(float)strtod(s,&end);
  if(end==s || !(v[i]>=-100.0f && v[i]<=100.0f) || *end!=(i==3?']':','))return 0;
  s=end+1;
 }
 if(*s)return 0;
 for(i=0;i<4;i++)if(fabsf(v[i])<5.0f)v[i]=0;
 x=(int32_t)(v[1]*12);y=(int32_t)(-v[0]*12);turn=(int32_t)(-v[2]*6);
 target[0]=x-y-turn;target[1]=x+y-turn;target[2]=x+y+turn;target[3]=x-y+turn;
 for(i=0;i<4;i++){int32_t a=target[i]<0?-target[i]:target[i];if(a>max)max=a;}
 for(i=0;i<4;i++)target[i]=(int32_t)((int64_t)target[i]*1200/max);
 return 1;
}
void Bluetooth_Update(void){
 unsigned budget=128,i;uint32_t now=HAL_GetTick();uint8_t reverse=0;
 if(error){
  uint32_t mask=__get_PRIMASK();__disable_irq();tail=head;error=0;__set_PRIMASK(mask);
  Stop();length=0;g_bluetooth.rx_errors++;
  HAL_UART_AbortReceive(&uart);HAL_UART_Receive_IT(&uart,&rx,1);
 }
 while(tail!=head && budget--){
  char c=(char)queue[tail];tail=(tail+1)&255;
  if(c=='[')length=0;
  else if(!length)continue;
  if(length>=sizeof(frame)-1){length=0;Stop();continue;}
  frame[length++]=c;
  if(c==']'){
   frame[length]=0;
   if(Parse())g_bluetooth.valid_packets++;else{g_bluetooth.invalid_packets++;Stop();}
   length=0;
  }
 }
 for(i=0;i<4;i++)if(target[i] && previous_sign[i] && (target[i]>0?1:-1)!=previous_sign[i])reverse=1;
 if(braking){
  Motor_StopAll();if(now-brake_since<500)return;
  braking=0;memset(previous_sign,0,sizeof(previous_sign));reverse=0;
 }
 if(reverse){Motor_StopAll();braking=1;brake_since=now;return;}
 if(!target[0]&&!target[1]&&!target[2]&&!target[3]){Motor_StopAll();return;}
 /* Joystick full scale maps to 500/1000 duty (50%), open loop. */
 for(i=0;i<4;i++){
  if(Motor_Set((MotorId)i,(int32_t)((int64_t)target[i]*500/1200))!=HAL_OK){Stop();return;}
 }
 for(i=0;i<4;i++)if(target[i])previous_sign[i]=target[i]>0?1:-1;
}

int Bluetooth_ReadByte(uint8_t *byte){
 if(error){
  uint32_t mask=__get_PRIMASK();__disable_irq();tail=head;error=0;__set_PRIMASK(mask);
  HAL_UART_AbortReceive(&uart);HAL_UART_Receive_IT(&uart,&rx,1);return -1;
 }
 if(tail==head)return 0;
 *byte=queue[tail];tail=(tail+1)&255;return 1;
}

/* Persistent IRQ TX buffer; do not overwrite while UART is transmitting. */
static uint8_t telemetry[256];
static volatile uint8_t telemetry_busy;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h){if(h==&uart)telemetry_busy=0;}
HAL_StatusTypeDef Bluetooth_SendAsync(const char *data,uint16_t size){
 HAL_StatusTypeDef status;
 if(telemetry_busy)return HAL_BUSY;
 if(size>sizeof(telemetry))return HAL_ERROR;
 memcpy(telemetry,data,size);telemetry_busy=1;
 status=HAL_UART_Transmit_IT(&uart,telemetry,size);
 if(status!=HAL_OK)telemetry_busy=0;
 return status;
}
