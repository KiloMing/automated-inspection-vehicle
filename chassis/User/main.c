#include "stm32f10x.h"
#include "Delay.h"
#include "UART.h"
#include "Encoder.h"
#include "Motor.h"
#include "HWT101.h"
#include <stdio.h>

#define SPEED_SAMPLE_MS 100U
#define REPORT_MS 200U

/* Integer output avoids pulling floating-point printf into the firmware. */
static long Centidegrees(float value)
{
    return (long)(value * 100.0f + ((value >= 0.0f) ? 0.5f : -0.5f));
}

int main(void)
{
    uint32_t last_sample;
    uint32_t last_report;
    uint32_t now;
    uint32_t elapsed;
    uint32_t sample_dt = 0U;
    int32_t count1;
    int32_t count2;
    int32_t previous1;
    int32_t previous2;
    int32_t delta1 = 0;
    int32_t delta2 = 0;
    long angle_cdeg;
    unsigned long angle_magnitude;
    HWT101_Data imu;
    char message[160];
    uint8_t command;

    Delay_TickInit();
    UART_Init(9600U);
    Encoder_Init();
    //Motor_Init();
    HWT101_Init();
    UART_SendString("IMU:115200. Debug:9600. r=encoder,z=soft zero,c=module zero,b=module reboot. ANGLE is module yaw in degrees.\r\n");
    previous1 = Encoder_GetCount(1U);
    previous2 = Encoder_GetCount(2U);
    last_sample = Delay_GetTick();
    last_report = last_sample;
    Motor_SetPWM(MOTOR_1, 500);
    Motor_SetPWM(MOTOR_2, 500);

    while (1)
    {
        HWT101_Poll();
        if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
        {
            command = (uint8_t)USART_ReceiveData(USART1);
            if ((command == 'r') || (command == 'R'))
            {
                Encoder_Reset();
                previous1 = 0;
                previous2 = 0;
                delta1 = 0;
                delta2 = 0;
                sample_dt = 0U;
                last_sample = Delay_GetTick();
            }
            else if ((command == 'c') || (command == 'C'))
            {
                UART_SendString(HWT101_ModuleZero() ? "Module zero requested\r\n" : "IMU busy or TX failed\r\n");
            }
            else if ((command == 'b') || (command == 'B'))
            {
                UART_SendString(HWT101_Reboot() ? "Module reboot requested\r\n" : "IMU busy or TX failed\r\n");
            }
            else if ((command == 'z') || (command == 'Z'))
            {
                UART_SendString(HWT101_Zero() ? "Yaw zero OK\r\n" : "Yaw zero: no fresh data\r\n");
            }
        }
        count1 = Encoder_GetCount(1U);
        count2 = Encoder_GetCount(2U);
        now = Delay_GetTick();
        elapsed = (uint32_t)(now - last_sample);
        if (elapsed >= SPEED_SAMPLE_MS)
        {
            delta1 = count1 - previous1;
            delta2 = count2 - previous2;
            previous1 = count1;
            previous2 = count2;
            last_sample = now;
            sample_dt = elapsed;
        }
        if ((uint32_t)(now - last_report) >= REPORT_MS)
        {
            last_report = now;
            if (HWT101_IsBusy())
            {
                sprintf(message, "IMU=RESETTING\r\n");
            }
            else if (HWT101_Read(&imu))
            {
                angle_cdeg = Centidegrees(imu.yaw_deg);
                angle_magnitude = (unsigned long)((angle_cdeg < 0) ? -angle_cdeg : angle_cdeg);
                sprintf(message, "ANGLE=%s%lu.%02lu deg,S1=%ld,S2=%ld,DT=%lu\r\n",
                        (angle_cdeg < 0) ? "-" : "", angle_magnitude / 100UL,
                        angle_magnitude % 100UL, (long)delta1, (long)delta2,
                        (unsigned long)sample_dt);
            }
            else
            {
                sprintf(message, "S1=%ld,S2=%ld,DT=%lu,IMU=NO_DATA_OR_TIMEOUT\r\n",
                        (long)delta1, (long)delta2, (unsigned long)sample_dt);
            }
            UART_SendString(message);
        }
    }
}
