/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         functions.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
/*
#include "os_tasks.h"

#include "dynamixel_ax.h" 
#include "bus_device.h"
#include "spi_gpo.h" 
#include "spi_gpo_defines.h"
#include "robot_behavior_routine.h"
#include "h_bridge.h"
#include "stm32f4xx_ll_gpio.h"
*/
#include "os_tasks.h" 
#include "dynamixel_master.h"
#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.h"
#include "robot_gpio_defines.h"
#include "byte2word.h" 
#include "maxon.h" 
#include "dynamixel_ax.h" 
#include "stepper.h" 
#include "rc_servo.h" 
#include "stm32f4xx_ll_tim.h"
#include "terminal.h"
#include "h_bridge.h"
#include "serial.h"

#include "spi_gpo.h" 
#include "spi_gpo_defines.h"
#include "kinematics.h"
#include "odometry.h"
#include "ws2812b.h" 
#include "robot_behavior_routine.h"

uint8_t gripper_state;

#define TASK_STACK_SIZE             512


static TaskHandle_t task_handle;
static StackType_t stack_buffer[TASK_STACK_SIZE];
static StaticTask_t task_buffer;

/*
static void ValveEnable(uint8_t valve_id)
{
	switch(valve_id){
		case 1:
			LL_GPIO_SetOutputPin(GPIOD, GPIO1_PD12);
			LL_GPIO_ResetOutputPin(GPIOD, GPIO2_PD13);
			break;
		case 2:
			LL_GPIO_SetOutputPin(GPIOD, GPIO3_PD15);
			LL_GPIO_ResetOutputPin(GPIOD, GPIO4_PD14);
			break;
		case 3:
			LL_GPIO_SetOutputPin(GPIOD, GPIO7_PD1);
			LL_GPIO_ResetOutputPin(GPIOD, GPIO8_PD0);
			break;
		case 4:
			LL_GPIO_SetOutputPin(GPIOB, GPIO9_PB9);
			LL_GPIO_ResetOutputPin(GPIOB, GPIO10_PB8);
			break;
		default:
			break;
	}
}

static void ValveDisable(uint8_t valve_id)
{
	switch(valve_id){
		case 1:
			LL_GPIO_ResetOutputPin(GPIOD, GPIO1_PD12);
			LL_GPIO_ResetOutputPin(GPIOD, GPIO2_PD13);
			break;
		case 2:
			LL_GPIO_ResetOutputPin(GPIOD, GPIO3_PD15);
			LL_GPIO_ResetOutputPin(GPIOD, GPIO4_PD14);
			break;
		case 3:
			LL_GPIO_ResetOutputPin(GPIOD, GPIO7_PD1);
			LL_GPIO_ResetOutputPin(GPIOD, GPIO8_PD0);
			break;
		case 4:
			LL_GPIO_ResetOutputPin(GPIOB, GPIO9_PB9);
			LL_GPIO_ResetOutputPin(GPIOB, GPIO10_PB8);
			break;
		default:
			break;
	}
}

*/
static void ValveEnable(uint8_t valve_id)
{
	switch(valve_id){
		case 1:
			SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_14);
			break;
		case 2:
			SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_16);
			break;
		case 3:
			SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_13);
			break;
		case 4:
			SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_15);
			break;
		default:
			break;
	}
}

static void ValveDisable(uint8_t valve_id)
{
	switch(valve_id){
		case 1:
			SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_14);
			break;
		case 2:
			SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_16);
			break;
		case 3:
			SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_13);
			break;
		case 4:
			SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_15);
			break;
		default:
			break;
	}
}

/*
static void ValveDisable(uint8_t valve_id)
{
	switch(valve_id){
		case 1:
			LL_GPIO_SetOutputPin(GPIOB, GPIO15_PB5);
			LL_GPIO_ResetOutputPin(GPIOB, GPIO16_PB4);
			break;
		case 2:
			LL_GPIO_SetOutputPin(GPIOB, GPIO17_PB6);
			LL_GPIO_ResetOutputPin(GPIOB, GPIO18_PB7);
			break;
		case 3:
			LL_GPIO_SetOutputPin(GPIOE, GPIO19_PE1);
			LL_GPIO_ResetOutputPin(GPIOE, GPIO20_PE0);
			break;
		case 4:
			LL_GPIO_SetOutputPin(GPIOE, GPIO21_PE2);
			LL_GPIO_ResetOutputPin(GPIOE, GPIO22_PE3);
			break;
		default:
			break;
	}
}

static void ValveEnable(uint8_t valve_id)
{
	switch(valve_id){
		case 1:
			LL_GPIO_ResetOutputPin(GPIOB, GPIO15_PB5);
			LL_GPIO_SetOutputPin(GPIOB, GPIO16_PB4);
			break;
		case 2:
			LL_GPIO_ResetOutputPin(GPIOB, GPIO17_PB6);
			LL_GPIO_SetOutputPin(GPIOB, GPIO18_PB7);
			break;
		case 3:
			LL_GPIO_ResetOutputPin(GPIOE, GPIO19_PE1);
			LL_GPIO_SetOutputPin(GPIOE, GPIO20_PE0);
			break;
		case 4:
			LL_GPIO_ResetOutputPin(GPIOE, GPIO21_PE2);
			LL_GPIO_SetOutputPin(GPIOE, GPIO22_PE3);
			break;
		default:
			break;
	}
}
*/

static void Task(void *arg)
{
  vTaskDelay(50);
	uint8_t dynamixel_id_1 = 98;
	uint8_t dynamixel_id_2 = 112;
	uint16_t dynamixel_speed = 200; //0-1024
	uint8_t start_angle = 60; //0-300
	uint8_t end_angle = 150; //0-300
	/*
	ValveDisable(1);
	ValveDisable(2);
	ValveDisable(3);
	ValveDisable(4);
	*/
	//uint8_t dynamixel_id = 0xFE;
	
	// Attempt to turn on power outputs +12V (failure)
	/* 
	SPI_GPO_ResetOutputPin(SPI_GPO_ALL);
	vTaskDelay(5000);
	SPI_GPO_SetOutputPin(SPI_GPO_ALL);
  vTaskDelay(5000);
	*/
	
	// Set custom id to single dynamixel (DISCONNECT ANY OTHER DYNAMIXELS!)
	/*
	while(dynamixel_id > 0)
	{
		dynamixel_id --;
		if(Dynamixel_AX.Ping(USART3, dynamixel_id) == 0)
		{
			Dynamixel_AX.Set_ID(USART3, dynamixel_id, 15);
			break;
		}
	}
	Dynamixel_AX.SetAngle(USART3, 15, 0);
	*/

	
	Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_1, dynamixel_speed);
	Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_2, dynamixel_speed);
	Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, 70);
	Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, 70);
	vTaskDelay(100);
	//Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, -10);
	//Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, -10);
	
	
	while(1)
	{
		
		/*
		Dynamixel_AX.Enable_LED(USART3, dynamixel_id_1);
		Dynamixel_AX.SetAngle(USART3, dynamixel_id_1,0);
		vTaskDelay(1000);
		Dynamixel_AX.Disable_LED(USART3, dynamixel_id_1);
		Dynamixel_AX.SetAngle(USART3, dynamixel_id_1,180);
		vTaskDelay(1000);
		
		Dynamixel_AX.Enable_LED(USART3, dynamixel_id_2);
		Dynamixel_AX.SetAngle(USART3, dynamixel_id_2,0);
		vTaskDelay(1000);
		Dynamixel_AX.Disable_LED(USART3, dynamixel_id_2);
		Dynamixel_AX.SetAngle(USART3, dynamixel_id_2,180);
		vTaskDelay(1000);
		*/

		vTaskDelay(100);
		
		/*
		ValveEnable(1);
		vTaskDelay(1000);
		ValveDisable(1);
		vTaskDelay(1000);
		ValveEnable(2);
		vTaskDelay(1000);
		ValveDisable(2);
		vTaskDelay(1000);
		ValveEnable(3);
		vTaskDelay(1000);
		ValveDisable(3);
		vTaskDelay(1000);
		ValveEnable(4);
		vTaskDelay(1000);
		ValveDisable(4);
		vTaskDelay(1000);
		
		ValveEnable(1);
		ValveEnable(2);
		ValveEnable(3);
		ValveEnable(4);
		vTaskDelay(5000);
		ValveDisable(1);
		ValveDisable(2);
		ValveDisable(3);
		ValveDisable(4);
		*/
		
		if(robot_ctrl.linear_switch_1 == LINEAR_SWITCH_PRESSED)
		{
			SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_5);
			SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_6);
			
			// 1st step
			ValveEnable(1);
			ValveEnable(2);
			ValveEnable(3);
			ValveEnable(4);
			
			if(robot_ctrl.linear_switch_3 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, start_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, start_angle);
				//vTaskDelay(1000);
			}
			vTaskDelay(1000);
			
			if(robot_ctrl.linear_switch_2 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, end_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, end_angle);
				//ValveDisable(1);
			}
			vTaskDelay(1000);
			
			// 2nd step
			//ValveEnable(1);
			//ValveEnable(2);
			//ValveEnable(3);
			//ValveEnable(4);
			
			ValveEnable(1);
			ValveEnable(2);
			ValveEnable(3);
			ValveDisable(4);
				
			if(robot_ctrl.linear_switch_3 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, start_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, start_angle);
				//vTaskDelay(1000);
			}
			vTaskDelay(1000);
			
			if(robot_ctrl.linear_switch_2 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, end_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, end_angle);
				//ValveDisable(2);
			}
			vTaskDelay(1000);
			
			// 3rd step
			//ValveEnable(1);
			//ValveEnable(2);
			//ValveEnable(3);
			//ValveEnable(4);
			
			ValveEnable(1);
			ValveEnable(2);
			ValveDisable(3);
			ValveDisable(4);
			
				
			if(robot_ctrl.linear_switch_3 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, start_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, start_angle);
				//vTaskDelay(1000);
			}
			vTaskDelay(1000);
			
			if(robot_ctrl.linear_switch_2 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, end_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, end_angle);
				ValveDisable(3);
			}
			vTaskDelay(1000);
			
			// 4th step
			//ValveEnable(1);
			//ValveEnable(2);
			//ValveEnable(3);
			//ValveEnable(4);
			
			ValveEnable(1);
			ValveDisable(2);
			ValveDisable(3);
			ValveDisable(4);
				
			if(robot_ctrl.linear_switch_3 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, start_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, start_angle);
				//vTaskDelay(1000);
				
			}
			vTaskDelay(1000);
			
			if(robot_ctrl.linear_switch_2 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, end_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, end_angle);
				ValveDisable(4);
			}
			vTaskDelay(1000);
			
			// final step (blocks dropping)
			ValveDisable(1);
			ValveDisable(2);
			ValveDisable(3);
			ValveDisable(4);
			
			if(robot_ctrl.linear_switch_3 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, start_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, start_angle);
				//vTaskDelay(1000);
			}
			vTaskDelay(1000);
			SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_5);
			SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_6);
			
			
			
			
			
			/*
			// 2nd step
			ValveEnable(1);
			ValveEnable(2);
			//ValveEnable(3);
			//ValveEnable(4);
				
			Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, start_angle);
			Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, start_angle);
			vTaskDelay(2000);
			Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, end_angle);
			Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, end_angle);
			vTaskDelay(2000);
			
			// 3rd step
			ValveEnable(1);
			ValveEnable(2);
			ValveEnable(3);
			//ValveEnable(4);
				
			Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, start_angle);
			Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, start_angle);
			vTaskDelay(2000);
			Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, end_angle);
			Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, end_angle);
			vTaskDelay(2000);
			
			// 4th step
			ValveEnable(1);
			ValveEnable(2);
			ValveEnable(3);
			ValveEnable(4);
				
			Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, start_angle);
			Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, start_angle);
			vTaskDelay(2000);
			Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, end_angle);
			Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, end_angle);
			vTaskDelay(2000);
			
			// final step (blocks dropping)
			Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, start_angle);
			Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, start_angle);
			vTaskDelay(2000);
			ValveDisable(1);
			ValveDisable(2);
			ValveDisable(3);
			ValveDisable(4);
			*/
		
		}
		
	}
	
}

static TaskHandle_t GetTaskHandle(void)
{
    return task_handle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    task_handle = xTaskCreateStatic(Task, "GripperTask", TASK_STACK_SIZE,
                              NULL, uxPriority, stack_buffer, &task_buffer);
}


const struct os_task_base_s GripperTask = { .CreateTask = CreateTask,
                                          .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
