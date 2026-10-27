/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "board_hw_config.h"
#include "spi_gpo.h" 
#include "spi_gpo_defines.h"
#include "board_gpio_defines.h"
#include "board_gpio_config.h"
#include "robot_gpio_config.h"
#include "maxon.h" 
#include "stepper.h" 
#include "rc_servo.h" 
#include "byte2word.h" 

#include "dynamixel_master.h"
#include "dynamixel_ax.h" 
#include "usartx_communication.h" 


#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "os_tasks.h" 
#include "hl_rtu.h"
#include "serial.h"



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	SYSCFG_PWR_Init();
	SystemClockConfig();
	
	/*	IT IS IMPORTANT FOR FreeRTOS	*/
	NVIC_SetPriorityGrouping(0);
	
	HW_ClockConfig();
	Board_GPIO_Config();
	Robot_GPIO_Config();

	SPI1_Board_HW_Init();
	SPI_GPO_ResetOutputPin(SPI_GPO_ALL);

	

	USART1_TerminalInit();
	USART3_DynamixelInit();
	UART4_RS485_Init();
  UART5_SerialInit();
	USART6_RS485_Init();

	
	
	TIM1_Steppers_PWM_Init();
	TIM4_PWM_Group1234_Init();
	TIM5_PWM_Group5678_Init();
	TIM8_WS2812B_Init();
	TIM14_USARTx_TimeoutInit();

	#if defined (ROBOT_BIG)

		

	#endif

	#if defined (ROBOT_GUIDO)

		

	#endif

	#if defined (ROBOT_BIG)

		

	#endif

  
  /*  Binding USART ports with protocols  */
  USARTx.BindProtocol(USART3, DynamixelMaster.GetProtocol());
  USARTx.BindProtocol(USART6, DynamixelMaster.GetProtocol());
  USARTx.BindProtocol(UART4, DynamixelMaster.GetProtocol());
  USARTx.BindProtocol(UART5, Serial.GetProtocol());
  USARTx.BindProtocol(USART1, HL_RTU.GetProtocol());

  /*  Creating tasks  */
	ButtonsPollTask.CreateTask(3);
  ButtonsCallbackTask.CreateTask(2);
	OdometryTask.CreateTask(1);
	TerminalTask.CreateTask(1);
  GameTask.CreateTask(1);

  GripperTask.CreateTask(1);
  // Experimental_Task_1.CreateTask(1);


  /*  Run scheduler  */
  vTaskStartScheduler();
	
  while (1);
}
	



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
