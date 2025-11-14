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

#include "bus_device.h"


uint8_t gripper_state;

#define TASK_STACK_SIZE             512


static TaskHandle_t task_handle;
static StackType_t stack_buffer[TASK_STACK_SIZE];
static StaticTask_t task_buffer;

#define MAXON
//#define VALVES_4
//#define SERVO

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


	#ifdef VALVES_4
		Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_1, dynamixel_speed);
		Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_2, dynamixel_speed);
		Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, 70);
		Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, 70);
		vTaskDelay(100);
		//Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, -10);
		//Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, -10);
	#endif


	#ifdef SERVO
		RC_Servo.Enable(RC_SERVO_1);
	#endif
	
	#ifdef MAXON
		//Maxon.Enable(bus_devices[MAXON_MOTOR_1].usart, bus_devices[MAXON_MOTOR_1].id);
		MaxonsDisable();
    	Kinematics.MatrixInit();
    	MaxonsEnable();
		vTaskDelay(500);
	#endif

	while(1)
	{
		vTaskDelay(100);
		
		#ifdef VALVES_4
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
			
		
		}
		#endif
		
		#ifdef SERVO
			RC_Servo.SetServoAngle(RC_SERVO_1, 700, 1900, 180, 0);
			vTaskDelay(200);
			RC_Servo.SetServoAngle(RC_SERVO_1, 700, 1900, 180, 135);
		#endif


		#ifdef MAXON
			/*
			Maxon.Set_TPM(bus_devices[MAXON_MOTOR_1].usart, bus_devices[MAXON_MOTOR_1].id, 10000000);
			vTaskDelay(2000);

			Maxon.Set_TPM(bus_devices[MAXON_MOTOR_1].usart, bus_devices[MAXON_MOTOR_1].id, -10000000);
			vTaskDelay(2000);
			*/
			struct speed_s speed;

			speed.dphi = 1.57f;
			speed.dx = 0;
			speed.dy = 0;
			Odometry.SetSpeed(&speed);
			vTaskDelay(1000);

			speed.dphi = -1.57f;
			Odometry.SetSpeed(&speed);
			vTaskDelay(1000);

			speed.dphi = 0;
			speed.dx = 0.1;
			speed.dy = 0;
			Odometry.SetSpeed(&speed);
			vTaskDelay(1000);
			
			speed.dphi = 0;
			speed.dx = -0.1;
			speed.dy = 0;
			Odometry.SetSpeed(&speed);
			vTaskDelay(1000);

			speed.dphi = 0;
			speed.dx = 0;
			speed.dy = 0.1;
			Odometry.SetSpeed(&speed);
			vTaskDelay(1000);
			
			speed.dphi = 0;
			speed.dx = 0;
			speed.dy = -0.1;
			Odometry.SetSpeed(&speed);
			vTaskDelay(1000);
		#endif

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
