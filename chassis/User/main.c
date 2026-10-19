#include "stm32f10x.h"
#include "Delay.h"
#include "UART.h"
#include "MotorControl.h"
#include "HWT101.h"
#include "RemoteControl.h"
#include "RemoteSerial.h"

int main(void)
{
    Delay_TickInit();
    UART_Init(115200U);
    MotorControl_Init();
    MotorControl_SetStartupPWM(REMOTE_STARTUP_PWM);
    HWT101_Init();
    RemoteControl_Init();
    RemoteSerial_Init();
    while (1)
    {
        RemoteSerial_Poll();
        MotorControl_Process();
        RemoteControl_Process();
    }
}
