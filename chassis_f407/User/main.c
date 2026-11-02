#include "ChassisControl.h"
/* Four-wheel physical-velocity chassis control. */
#include <stdio.h>
#include "main.h"
#include "motor.h"
#include "MotorControl.h"
#include "HWT101.h"
#include "Bluetooth.h"
int main(void) {
 if(HAL_Init()!=HAL_OK)Error_Handler();
 SystemClock_Config();
 if(Motor_Init()!=HAL_OK)Error_Handler();Motor_StopAll();
 if(Chassis_Init()!=HAL_OK)Error_Handler();
 while(1){Chassis_Update();__WFI();}
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  /* Board HSE=8 MHz; failure leaves the motor GPIO uninitialized. */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 8;
  osc.PLL.PLLN = 336;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) { Error_Handler(); }
  clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV4;
  clk.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) { Error_Handler(); }
  SystemCoreClockUpdate();
}

void Error_Handler(void)
{

  
  
  
  Motor_StopAll();
  __disable_irq();
  while (1) { /* No GPIO signaling on an unverified board. */ }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
  Error_Handler();
}
#endif
