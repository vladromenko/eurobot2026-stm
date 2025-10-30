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
#include "dynamixel_master.h"
#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.h"
#include "robot_gpio_defines.h"
#include "byte2word.h" 
#include "maxon.h" 
#include "dynamixel_ax.h" 

#include "kinematics.h"
#include "odometry.h"
#include "bus_device.h"


#define ODOMETRY_UPDATE_RATE_HZ     100

#define TASK_STACK_SIZE             512

static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;



static void Task(void *arg)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000/ODOMETRY_UPDATE_RATE_HZ;

  vTaskDelay(100);

  Kinematics.MatrixInit();
  Odometry.Init();
  
	while(1)
	{
    xLastWakeTime = xTaskGetTickCount();
    Odometry.UpdatePath();
    xTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "OdometryTask", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


const struct os_task_base_s OdometryTask = { .CreateTask = CreateTask,
                                          .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
