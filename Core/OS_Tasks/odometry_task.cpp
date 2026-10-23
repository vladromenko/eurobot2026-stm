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

#include "os_tasks.hpp" 
#include "dynamixel_master.hpp"
#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.hpp"
#include "robot_gpio_defines.hpp"
#include "byte2word.hpp" 
#include "maxon.hpp" 
#include "dynamixel_ax.hpp" 

#include "kinematics.hpp"
#include "odometry.hpp"
#include "bus_device.hpp"


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

  Odometry.Init();
  xLastWakeTime = xTaskGetTickCount();
  
	while(1)
	{
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
