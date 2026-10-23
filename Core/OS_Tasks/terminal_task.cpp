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
#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.hpp"
#include "terminal_cmd_handlers.hpp" 
#include "terminal.hpp"
#include "debug_terminal.hpp"
#include "robot_behavior_routine.hpp"
#include "pca9685.hpp"




#define TASK_STACK_SIZE             512

static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;


static void Task(void *arg)
{
    struct terminal_request_s terminal_request;

    vTaskDelay(100);
    
	while(1)
	{
		
        while(Terminal.GetRequest(&terminal_request))
        {
            TerminalCmd.Handle(&terminal_request);
        }

				

        taskYIELD();
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "TerminalTask", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


const struct os_task_base_s TerminalTask = {   .CreateTask = CreateTask,
                                            .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
