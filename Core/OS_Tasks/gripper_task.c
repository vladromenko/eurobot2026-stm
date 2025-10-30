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

#include "os_tasks.h"
#include "gripper_consts.h"
#include "dynamixel_ax.h" 
#include "bus_device.h"

int8_t gripper_state = -1;

#define TASK_STACK_SIZE             512


static TaskHandle_t task_handle;
static StackType_t stack_buffer[TASK_STACK_SIZE];
static StaticTask_t task_buffer;
int16_t speed = 0;

static void Task(void *arg)
{
  vTaskDelay(50);

  Dynamixel_AX.TorqueEnable(bus_devices[SERVO_GRAB].usart, bus_devices[SERVO_GRAB].id);
	Dynamixel_AX.TorqueEnable(bus_devices[SERVO_LIFT11].usart, bus_devices[SERVO_LIFT11].id);
	Dynamixel_AX.TorqueEnable(bus_devices[SERVO_LIFT12].usart, bus_devices[SERVO_LIFT12].id);
	Dynamixel_AX.TorqueEnable(bus_devices[SERVO_LIFT21].usart, bus_devices[SERVO_LIFT21].id);
	Dynamixel_AX.TorqueEnable(bus_devices[SERVO_LIFT22].usart, bus_devices[SERVO_LIFT22].id);
	
	speed = 0;
	while(speed != 100){
		Dynamixel_AX.SetMovingSpeed(bus_devices[SERVO_LIFT11].usart, bus_devices[SERVO_LIFT11].id, 100);
		Dynamixel_AX.GetMovingSpeed(bus_devices[SERVO_LIFT11].usart, bus_devices[SERVO_LIFT11].id, &speed);
	}
	
	speed = 0;
	while(speed != 100){
		Dynamixel_AX.SetMovingSpeed(bus_devices[SERVO_LIFT12].usart, bus_devices[SERVO_LIFT12].id, 100);
		Dynamixel_AX.GetMovingSpeed(bus_devices[SERVO_LIFT12].usart, bus_devices[SERVO_LIFT12].id, &speed);
	}
	
	speed = 0;
	while(speed != 100){
		Dynamixel_AX.SetMovingSpeed(bus_devices[SERVO_LIFT21].usart, bus_devices[SERVO_LIFT21].id, 100);
		Dynamixel_AX.GetMovingSpeed(bus_devices[SERVO_LIFT21].usart, bus_devices[SERVO_LIFT21].id, &speed);
	}
	
	speed = 0;
	while(speed != 100){
		Dynamixel_AX.SetMovingSpeed(bus_devices[SERVO_LIFT22].usart, bus_devices[SERVO_LIFT22].id, 100);
		Dynamixel_AX.GetMovingSpeed(bus_devices[SERVO_LIFT22].usart, bus_devices[SERVO_LIFT22].id, &speed);
	}
	
	while(1)
	{
    //vTaskSuspend(task_handle);

    switch (gripper_state)
    {
      case 0: //open grab
        Dynamixel_AX.SetAngle(bus_devices[SERVO_GRAB].usart, bus_devices[SERVO_GRAB].id, 60);
				gripper_state = -1;
        break;
      case 1: //close grab
        Dynamixel_AX.SetAngle(bus_devices[SERVO_GRAB].usart, bus_devices[SERVO_GRAB].id, 125);
				gripper_state = -1;
				break;
			case 2: //down
				Dynamixel_AX.SetAngle(bus_devices[SERVO_LIFT11].usart, bus_devices[SERVO_LIFT11].id, 167);
				Dynamixel_AX.SetAngle(bus_devices[SERVO_LIFT12].usart, bus_devices[SERVO_LIFT12].id, 186);
				gripper_state = -1;
				break;
			case 3:
        Dynamixel_AX.SetAngle(bus_devices[SERVO_LIFT11].usart, bus_devices[SERVO_LIFT11].id, 77);
				Dynamixel_AX.SetAngle(bus_devices[SERVO_LIFT12].usart, bus_devices[SERVO_LIFT12].id, 93);
				Dynamixel_AX.SetAngle(bus_devices[SERVO_LIFT21].usart, bus_devices[SERVO_LIFT21].id, 203);
				Dynamixel_AX.SetAngle(bus_devices[SERVO_LIFT22].usart, bus_devices[SERVO_LIFT22].id, 202);
				gripper_state = -1;
        break;
      case 4: //up
				Dynamixel_AX.SetAngle(bus_devices[SERVO_LIFT11].usart, bus_devices[SERVO_LIFT11].id, 30);
				Dynamixel_AX.SetAngle(bus_devices[SERVO_LIFT12].usart, bus_devices[SERVO_LIFT12].id, 48);
				gripper_state = -1;
				break;
			case 5:
        Dynamixel_AX.SetAngle(bus_devices[SERVO_LIFT11].usart, bus_devices[SERVO_LIFT11].id, 145);
				Dynamixel_AX.SetAngle(bus_devices[SERVO_LIFT12].usart, bus_devices[SERVO_LIFT12].id, 163);
				Dynamixel_AX.SetAngle(bus_devices[SERVO_LIFT21].usart, bus_devices[SERVO_LIFT21].id, 83);
				Dynamixel_AX.SetAngle(bus_devices[SERVO_LIFT22].usart, bus_devices[SERVO_LIFT22].id, 82);
				gripper_state = -1;
        break;
			case 6:
        Dynamixel_AX.SetAngle(bus_devices[SERVO_LIFT11].usart, bus_devices[SERVO_LIFT11].id, 195);
				Dynamixel_AX.SetAngle(bus_devices[SERVO_LIFT12].usart, bus_devices[SERVO_LIFT12].id, 213);
				gripper_state = -1;
        break;
			default:
				break;
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
