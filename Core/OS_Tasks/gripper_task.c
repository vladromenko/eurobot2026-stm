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

#include "dynamixel_ax.h" 
#include "bus_device.h"

uint8_t gripper_state;

#define TASK_STACK_SIZE             512


static TaskHandle_t task_handle;
static StackType_t stack_buffer[TASK_STACK_SIZE];
static StaticTask_t task_buffer;


static void Task(void *arg)
{
  vTaskDelay(50);

  Dynamixel_AX.TorqueEnable(bus_devices[GRIPPER_TOP].usart, bus_devices[GRIPPER_TOP].id);
	Dynamixel_AX.TorqueEnable(bus_devices[GRIPPER_BOTTOM].usart, bus_devices[GRIPPER_BOTTOM].id);

	while(1)
	{
    vTaskSuspend(task_handle);

    switch (gripper_state)
    {
      case 0:
        Dynamixel_AX.SetAngle(bus_devices[GRIPPER_TOP].usart, bus_devices[GRIPPER_TOP].id, 0);
        Dynamixel_AX.SetAngle(bus_devices[GRIPPER_BOTTOM].usart, bus_devices[GRIPPER_BOTTOM].id, 0);
        break;
      case 1:
        Dynamixel_AX.SetAngle(bus_devices[GRIPPER_TOP].usart, bus_devices[GRIPPER_TOP].id, 100);
        Dynamixel_AX.SetAngle(bus_devices[GRIPPER_BOTTOM].usart, bus_devices[GRIPPER_BOTTOM].id, 100);
        vTaskDelay(1000);
        Dynamixel_AX.SetAngle(bus_devices[GRIPPER_TOP].usart, bus_devices[GRIPPER_TOP].id, 50);
        Dynamixel_AX.SetAngle(bus_devices[GRIPPER_BOTTOM].usart, bus_devices[GRIPPER_BOTTOM].id, 50);
        vTaskDelay(1000);
        Dynamixel_AX.SetAngle(bus_devices[GRIPPER_TOP].usart, bus_devices[GRIPPER_TOP].id, 0);
        Dynamixel_AX.SetAngle(bus_devices[GRIPPER_BOTTOM].usart, bus_devices[GRIPPER_BOTTOM].id, 0);
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
