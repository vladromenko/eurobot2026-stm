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
#include "button.hpp"
#include "buttons_enum.hpp"
#include "robot_behavior_routine.hpp" 
#include "terminal_cmd_handlers.hpp"
#include "spi_gpo.hpp" 
#include "spi_gpo_defines.hpp"
#include "kinematics.hpp"
#include "odometry.hpp"
#include "rc_servo.hpp"
#include "stepper.hpp"
#include "odometry.hpp"
#include "bus_device.hpp"
#include "stm32f4xx_ll_iwdg.h"     

#define TASK_STACK_SIZE             512


static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;

extern uint8_t COMM_ENABLE; // from terminal_cmd_handlers.c
extern uint8_t gripper_cmd;
extern uint8_t gripper_id;
extern uint8_t grips_cmd;

extern uint8_t dir;
extern uint8_t move_cmd_speed;
extern uint8_t move_cmd_time;

static void Task(void *arg)
{
  TickType_t xGameStartTime;

  robot_ctrl.starting_cord = STARTING_CORD_ATTACHED;
  robot_ctrl.robot_mode = GAME_MODE; // [GAME_MODE, DEBUG_MODE]
	robot_ctrl.reset_state = RESETED;

  vTaskDelay(50);
  // PrepareRobot();
	MaxonsStop();
  MaxonsEnable();
  
	while(1)
	{
		vTaskDelay(50);
		LL_IWDG_ReloadCounter(IWDG);
    /*  PREPARE ROBOT FOR THE GAME, SET IT TO THE INITIAL CONDITION  */
    
    /*  WAITING FOR THE STARTING CORD EXTRACTION  */
    while(robot_ctrl.starting_cord == STARTING_CORD_ATTACHED)
    {
      taskYIELD();
    }
    /*  INITIALISE GAMING 100 SECONDS TIMEOUT  */
    xGameStartTime = xTaskGetTickCount();

    if (gripper_cmd == 0xFF) {
      //SPI_GPO.SetOutputPin(SPI_POWER_OUTPUT_17 | SPI_POWER_OUTPUT_18);
      /*gripper_id = 1;
      grips_cmd = 0;
      gripper_cmd = 1;
      vTaskDelay(200); */
      gripper_id = 2;
      grips_cmd = 0;
      gripper_cmd = 1;
      vTaskDelay(150);
      /*
      gripper_id = 1;
      dir = 0;
      move_cmd_speed = 25;
      move_cmd_time = 4;
      gripper_cmd = 6;
      vTaskDelay(2100);

      gripper_id = 2;
      dir = 1;
      move_cmd_speed = 25;
      move_cmd_time = 2;
      gripper_cmd = 6;
      vTaskDelay(1100);
      */
    }

    /*  START THE GAME AND WAIT UNTIL 100 SECONDS ARE RUNNNING OUT OR THE STARTING CORD IS ATTACHED AGAIN  */
    while(robot_ctrl.starting_cord == STARTING_CORD_RELEASED)
    {
      if(((xTaskGetTickCount() - xGameStartTime) >= 99000) && (robot_ctrl.robot_mode == GAME_MODE)){
        /*  STOP THE GAME AND TURN OF ALL ACTUATORS ACCORDING TO THE EUROBOT RULES */
        MaxonsStop();
        StopAllActuators();
        COMM_ENABLE = 0; // disable communication with HL
        break;
      }
        taskYIELD();
    }

    /*  DO NOTHING UNTIL THE STARTING CORD WILL BE ATTACHED AGAIN */
    while(robot_ctrl.starting_cord == STARTING_CORD_RELEASED)
      taskYIELD();
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "GameTask", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


const struct os_task_base_s GameTask = {.CreateTask = CreateTask,
                                      .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
