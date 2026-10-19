#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "RemoteSerial.h"
#include "RemoteControl.h"
USART_TypeDef test_usart;
static uint32_t now, errors, received_ms;
static unsigned count;
static char received[512];
uint32_t Delay_GetTick(void) { return now; }
void RemoteControl_FeedAt(uint8_t b,uint32_t ms)
{ received[count++]=(char)b; received_ms=ms; }
void RemoteControl_RxError(void) { errors++; }
static void inject(uint8_t b,uint32_t status)
{ test_usart.SR=status; test_usart.DR=b; RemoteSerial_RxIRQ(); }
int main(void)
{
    unsigned i;
    const char *p="[j, 0, 100, 0, 0][j,0,0,0,0]";
    RemoteSerial_Init(); now=10;
    for(i=0;i<strlen(p);i++) inject((uint8_t)p[i],USART_SR_RXNE);
    assert(count==0); /* ISR buffers without parsing or commanding motors. */
    now=20; RemoteSerial_Poll();
    assert(count==strlen(p) && memcmp(received,p,count)==0 && received_ms==10);
    count=0;
    for(i=0;i<200;i++) inject('a',USART_SR_RXNE);
    RemoteSerial_Poll(); assert(count==64);
    RemoteSerial_Poll(); RemoteSerial_Poll(); RemoteSerial_Poll(); assert(count==200);
    count=0;
    for(i=0;i<256;i++) inject('b',USART_SR_RXNE);
    RemoteSerial_Poll(); assert(errors==1 && count==0);
    inject('c',USART_SR_RXNE); RemoteSerial_Poll(); assert(count==1);
    inject('d',USART_SR_RXNE|USART_SR_ORE); RemoteSerial_Poll(); assert(errors==2);
    count=0; inject('e',USART_SR_RXNE); now+=REMOTE_SERIAL_MAX_AGE_MS;
    RemoteSerial_Poll(); assert(count==0 && errors==3);
    now=0xfffffff0U; inject('f',USART_SR_RXNE); now=5;
    RemoteSerial_Poll(); assert(count==1 && received_ms==0xfffffff0U);
    puts("Remote serial queue tests passed");
    return 0;
}
