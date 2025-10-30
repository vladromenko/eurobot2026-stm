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
#include "button.h"
#include "stm32f4xx_ll_gpio.h"
#include "buttons_enum.h"
#include "robot_behavior_routine.h" 


#define TASK_STACK_SIZE             64


static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;


static void Task(void *arg)
{
  struct button_callback_queue_item_s button_callback_item;

  while(button_callbacks_queue_handle==NULL)
    taskYIELD();

	while(1)
	{
        while(xQueueReceive(button_callbacks_queue_handle, (void*)(&button_callback_item), portMAX_DELAY) == pdFAIL);
        ButtonsCallback(button_callback_item.button_id, button_callback_item.callback_type, button_callback_item.button_action, button_callback_item.button_state);
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "ButtonsCallbackTask", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


const struct os_task_base_s ButtonsCallbackTask = {.CreateTask = CreateTask,
                                          .GetTaskHandle = GetTaskHandle};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
