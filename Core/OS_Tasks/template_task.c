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


#define TASK_STACK_SIZE             512


static TaskHandle_t task_handle;
static StackType_t stack_buffer[TASK_STACK_SIZE];
static StaticTask_t task_buffer;


static void Task(void *arg)
{
  
	while(1)
	{
    
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return task_handle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    task_handle = xTaskCreateStatic(Task, "TemplateTask", TASK_STACK_SIZE,
                              NULL, uxPriority, stack_buffer, &task_buffer);
}


const struct os_task_base_s TemplateTask = { .CreateTask = CreateTask,
                                          .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
