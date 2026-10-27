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
#include "board_gpio_defines.h"
#include "robot_gpio_defines.h"
#include "buttons_enum.h"
#include "robot_behavior_routine.h" 
#include "hw_instance.h"


#define TASK_STACK_SIZE                   64
#define BUTTON_CALLBACKS_QUEUE_LENGTH     BUTTONS_NUMBER
#define BUTTON_CALLBACKS_QUEUE_ITEM_SIZE  sizeof(struct button_callback_queue_item_s)


static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;

static uint8_t ucQueueStorageBuffer[BUTTON_CALLBACKS_QUEUE_LENGTH * BUTTON_CALLBACKS_QUEUE_ITEM_SIZE];
static StaticQueue_t xStaticQueue;

static void ButtonCallbackRedirect(enum buttons_e button_id, enum button_callback_type_s callback_type, enum button_action_e button_action, enum button_state_e button_state)
{
	struct button_callback_queue_item_s button_callback_item;

	button_callback_item.button_id = button_id;
	button_callback_item.callback_type = callback_type;
	button_callback_item.button_action = button_action;
	button_callback_item.button_state = button_state;
	
	xQueueSend(button_callbacks_queue_handle, &button_callback_item, 0);
}


static void Task(void *arg)
{
  const struct hw_instance_s* hw = GetHwInstance();
  TickType_t xLastWakeTime;

  hw->button->AddButton(STARTING_CORD_BTN, GPIOC, STARTING_CORD_BTN_GPIO14_PC15, BUTTON_POLARITY_HI, ButtonCallbackRedirect);
  hw->button->AddButton(PLAYGROUND_SIDE_BTN, GPIOE, PLAYGROUND_SIDE_BTN_GPIO13_PE6, BUTTON_POLARITY_HI, ButtonCallbackRedirect);
  hw->button->AddButton(STRATEGY_BTN, GPIOE, STRATEGY_BTN_GPIO12_PE5, BUTTON_POLARITY_HI, ButtonCallbackRedirect);
  hw->button->AddButton(MAXON_ENABLE_BTN, GPIOE, MAXON_ENABLE_BTN_GPIO11_PE2, BUTTON_POLARITY_HI, ButtonCallbackRedirect);

  button_callbacks_queue_handle = xQueueCreateStatic(BUTTON_CALLBACKS_QUEUE_LENGTH, BUTTON_CALLBACKS_QUEUE_ITEM_SIZE, ucQueueStorageBuffer, &xStaticQueue);

	while(1)
	{
    xLastWakeTime = xTaskGetTickCount();
    hw->button->PollButtons();
    xTaskDelayUntil(&xLastWakeTime,1);
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "ButtonsPollTask", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


const struct os_task_base_s ButtonsPollTask = {.CreateTask = CreateTask,
                                            .GetTaskHandle = GetTaskHandle};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
