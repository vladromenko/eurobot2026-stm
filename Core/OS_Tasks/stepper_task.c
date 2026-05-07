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
#include "stepper.h" 
#include "robot_behavior_routine.h"

int32_t stepper1_goal_position = 0;
int16_t stepper2_goal_position = 0;
int16_t stepper3_goal_position = 0;

uint8_t stepper1_calibrate = 0;
uint8_t stepper2_calibrate = 0;
uint8_t stepper3_calibrate = 0;


#define TASK_STACK_SIZE             512

#define MM_TO_STEPS 	25


static TaskHandle_t task_handle;
static StackType_t stack_buffer[TASK_STACK_SIZE];
static StaticTask_t task_buffer;


static void Task(void *arg)
{
    vTaskDelay(50);
    
    Stepper.Init(STEPPER_MOTOR_1);
    Stepper.SetZeroPosisition(STEPPER_MOTOR_1);
    
    // Флаг состояния, чтобы отправлять команды по SPI только один раз
    uint8_t stepper1_enabled = 0; 
    
    while(1)
    {
        vTaskDelay(50);

        if (stepper1_calibrate == 1) {
            
            // Во время калибровки мотор должен быть включен
            if (!stepper1_enabled) {
                Stepper.Enable(STEPPER_MOTOR_1);
                stepper1_enabled = 1;
            }

            //if ((robot_ctrl.linear_switch_1 != LINEAR_SWITCH_PRESSED)||(robot_ctrl.linear_switch_2 != LINEAR_SWITCH_PRESSED)) {
            if (robot_ctrl.linear_switch_2 != LINEAR_SWITCH_PRESSED) {
                if (Stepper.GetPresentPosisition(STEPPER_MOTOR_1) != -100000) {
                    Stepper.StepToPosisition(STEPPER_MOTOR_1, -100000, 32, 16, 500); 
                }
            } else {
                int32_t current_pos = Stepper.GetPresentPosisition(STEPPER_MOTOR_1);
                Stepper.StepToPosisition(STEPPER_MOTOR_1, current_pos-50, 32, 10, 500);
                vTaskDelay(100);
                Stepper.StopMotor(STEPPER_MOTOR_1);         
                Stepper.SetZeroPosisition(STEPPER_MOTOR_1); 
                
                //stepper1_goal_position = -2200;
                stepper1_goal_position = 1000; 
                Stepper.StepToPosisition(STEPPER_MOTOR_1, stepper1_goal_position, 32, 10, 300);
                
                stepper1_calibrate = 0; 
            }
        }
        else {
            // Обычный режим работы
            if(Stepper.GetPresentPosisition(STEPPER_MOTOR_1) != stepper1_goal_position) {
                // Мы в пути -> Включаем драйвер (если еще не включен)
                if (!stepper1_enabled) {
                    Stepper.Enable(STEPPER_MOTOR_1);
                    stepper1_enabled = 1;
                }
            } else {
                // Стоим на цели -> Выключаем драйвер (если еще включен)
                if (stepper1_enabled) {
                    Stepper.Disable(STEPPER_MOTOR_1);
                    stepper1_enabled = 0;
                }
            }
        }
    }
}

static TaskHandle_t GetTaskHandle(void)
{
    return task_handle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    task_handle = xTaskCreateStatic(Task, "StepperTask", TASK_STACK_SIZE,
                              NULL, uxPriority, stack_buffer, &task_buffer);
}


const struct os_task_base_s StepperTask = { .CreateTask = CreateTask,
                                          .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
