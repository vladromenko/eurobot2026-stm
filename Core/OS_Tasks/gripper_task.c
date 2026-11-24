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
#include "hw_instance.h"
#include "stepper.h"
#include "l6474_drvr.h"

/* Set by the terminal commands, gripper_cmd is cleared by the task when done */
uint8_t gripper_state;                      // BOARD_TOGGLE_GRIPPER: 0 - close, 1 - cycle
volatile uint8_t gripper_cmd = GRIPPER_CMD_NONE;
volatile uint8_t gripper_cmd_status = 0;    // 1 - reached, 0 - timeout, bus error or the other stepper is moving
uint8_t gripper_height;                     // mm from the power-on position
uint8_t cursor_pos;                         // 1 - horizontal, otherwise vertical
int32_t rack_distance_mm;                   // from the start position of the pinion

#define TASK_STACK_SIZE             512

/* Calibrated on lymunt-2026. Stepper 1 runs in full step (StepperMotor_1_Init) */
#define VERTICAL_MM_TO_STEPS        25
#define VERTICAL_NORMAL_CURRENT_MA  1100.0f
#define VERTICAL_DOWN_CURRENT_MA    320.0f
#define VERTICAL_EDGE_DIV           64
#define VERTICAL_WORKING_DIV        10
#define VERTICAL_RAMP_MS            200
#define VERTICAL_TIMEOUT_MS         4000    // the terminal handler waits a bit longer

/* Dynamixel angles in deg (0..300), speeds 0..1023 */
#define CURSOR_SPEED                600
#define CURSOR_HOR_ANGLE            140
#define CURSOR_VERT_ANGLE           80

#define PINION_SPEED                800
#define PINION_START_ANGLE          148
#define PINION_MM_TO_DEG            3.979f

#define ANGLE_TIMEOUT_MS            1500
#define ANGLE_TOLERANCE             10


static TaskHandle_t task_handle;
static StackType_t stack_buffer[TASK_STACK_SIZE];
static StaticTask_t task_buffer;


static uint8_t WaitDynamixelAngle(const struct hw_instance_s* hw, enum bus_devices_e dev, uint16_t target_angle)
{
    TickType_t start_time = xTaskGetTickCount();
    uint16_t angle;

    while (xTaskGetTickCount() - start_time < pdMS_TO_TICKS(ANGLE_TIMEOUT_MS)) {
        if (hw->dynamixel_ax->GetAngle(hw->bus_devices[dev].usart, hw->bus_devices[dev].id, &angle) == 0) {
            int32_t diff = (int32_t)angle - (int32_t)target_angle;
            if (diff < ANGLE_TOLERANCE && diff > -ANGLE_TOLERANCE) {
                return 1;
            }
        }
        vTaskDelay(20);
    }
    return 0;
}

static void StartDynamixel(const struct hw_instance_s* hw, enum bus_devices_e dev, int16_t speed)
{
    hw->dynamixel_ax->SetMovingSpeed(hw->bus_devices[dev].usart, hw->bus_devices[dev].id, speed);
    hw->dynamixel_ax->TorqueEnable(hw->bus_devices[dev].usart, hw->bus_devices[dev].id);
}

static uint8_t SetDynamixelAngle(const struct hw_instance_s* hw, enum bus_devices_e dev, int16_t speed, uint16_t angle)
{
    StartDynamixel(hw, dev, speed);
    hw->dynamixel_ax->SetAngle(hw->bus_devices[dev].usart, hw->bus_devices[dev].id, angle);
    vTaskDelay(50);
    return WaitDynamixelAngle(hw, dev, angle);
}

/* Lowering uses a lower current, the driver is disabled after the move */
static uint8_t MoveVertical(const struct hw_instance_s* hw, uint8_t height_mm)
{
    int32_t target_position = (int32_t)height_mm * VERTICAL_MM_TO_STEPS;
    int32_t current_position = hw->stepper->GetPresentPosisition(STEPPER_MOTOR_1);
    TickType_t start_time;
    uint8_t reached;

    if (target_position == current_position) {
        return 1;
    }

    hw->l6474h->SetCurrentLimit_mA(STEPPER_MOTOR_1, (target_position < current_position)
                                   ? VERTICAL_DOWN_CURRENT_MA : VERTICAL_NORMAL_CURRENT_MA);
    // Every L6474 reset latches UVLO: without GetStatus the ENABLE command is ignored
    hw->l6474h->GetStatus(STEPPER_MOTOR_1);
    hw->stepper->Enable(STEPPER_MOTOR_1);
    vTaskDelay(10);
    hw->stepper->StepToPosisition(STEPPER_MOTOR_1, target_position, VERTICAL_EDGE_DIV,
                                  VERTICAL_WORKING_DIV, VERTICAL_RAMP_MS);

    start_time = xTaskGetTickCount();
    while (hw->stepper->GetPresentPosisition(STEPPER_MOTOR_1) != target_position &&
           xTaskGetTickCount() - start_time < pdMS_TO_TICKS(VERTICAL_TIMEOUT_MS)) {
        vTaskDelay(20);
    }
    reached = (hw->stepper->GetPresentPosisition(STEPPER_MOTOR_1) == target_position);

    hw->stepper->StopMotor(STEPPER_MOTOR_1);
    hw->stepper->Disable(STEPPER_MOTOR_1);
    hw->l6474h->SetCurrentLimit_mA(STEPPER_MOTOR_1, VERTICAL_NORMAL_CURRENT_MA);
    return reached;
}

static uint8_t SlideGripper(const struct hw_instance_s* hw, int32_t distance_mm)
{
    int32_t angle = PINION_START_ANGLE + (int32_t)((float)distance_mm * PINION_MM_TO_DEG);

    if (angle < 0) angle = 0;
    if (angle > 300) angle = 300;
    return SetDynamixelAngle(hw, GRIPPER_PINION, PINION_SPEED, (uint16_t)angle);
}

/* Start positions: cursor vertical, pinion at the start of the rack, the vertical zero is the power-on position */
static void InitPositions(const struct hw_instance_s* hw)
{
    hw->stepper->Init(STEPPER_MOTOR_1);
    hw->l6474h->GetStatus(STEPPER_MOTOR_1);
    hw->stepper->SetZeroPosisition(STEPPER_MOTOR_1);
    hw->stepper->Disable(STEPPER_MOTOR_1);

    StartDynamixel(hw, GRIPPER_CURSOR, CURSOR_SPEED);
    hw->dynamixel_ax->SetAngle(hw->bus_devices[GRIPPER_CURSOR].usart, hw->bus_devices[GRIPPER_CURSOR].id, CURSOR_VERT_ANGLE);
    vTaskDelay(100);

    StartDynamixel(hw, GRIPPER_PINION, PINION_SPEED);
    hw->dynamixel_ax->SetAngle(hw->bus_devices[GRIPPER_PINION].usart, hw->bus_devices[GRIPPER_PINION].id, PINION_START_ANGLE);
    vTaskDelay(100);
}

static void ToggleGripper(const struct hw_instance_s* hw)
{
    switch (gripper_state)
    {
      case 0:
        hw->dynamixel_ax->SetAngle(hw->bus_devices[GRIPPER_TOP].usart, hw->bus_devices[GRIPPER_TOP].id, 0);
        hw->dynamixel_ax->SetAngle(hw->bus_devices[GRIPPER_BOTTOM].usart, hw->bus_devices[GRIPPER_BOTTOM].id, 0);
        break;
      case 1:
        hw->dynamixel_ax->SetAngle(hw->bus_devices[GRIPPER_TOP].usart, hw->bus_devices[GRIPPER_TOP].id, 100);
        hw->dynamixel_ax->SetAngle(hw->bus_devices[GRIPPER_BOTTOM].usart, hw->bus_devices[GRIPPER_BOTTOM].id, 100);
        vTaskDelay(1000);
        hw->dynamixel_ax->SetAngle(hw->bus_devices[GRIPPER_TOP].usart, hw->bus_devices[GRIPPER_TOP].id, 50);
        hw->dynamixel_ax->SetAngle(hw->bus_devices[GRIPPER_BOTTOM].usart, hw->bus_devices[GRIPPER_BOTTOM].id, 50);
        vTaskDelay(1000);
        hw->dynamixel_ax->SetAngle(hw->bus_devices[GRIPPER_TOP].usart, hw->bus_devices[GRIPPER_TOP].id, 0);
        hw->dynamixel_ax->SetAngle(hw->bus_devices[GRIPPER_BOTTOM].usart, hw->bus_devices[GRIPPER_BOTTOM].id, 0);
        break;
    }
}

static void Task(void *arg)
{
  const struct hw_instance_s* hw = GetHwInstance();
  vTaskDelay(50);

  hw->dynamixel_ax->TorqueEnable(hw->bus_devices[GRIPPER_TOP].usart, hw->bus_devices[GRIPPER_TOP].id);
	hw->dynamixel_ax->TorqueEnable(hw->bus_devices[GRIPPER_BOTTOM].usart, hw->bus_devices[GRIPPER_BOTTOM].id);
  InitPositions(hw);

	while(1)
	{
    vTaskDelay(10);

    switch (gripper_cmd)
    {
      case GRIPPER_CMD_TOGGLE:
        ToggleGripper(hw);
        break;
      case GRIPPER_CMD_VERTICAL:
        // Busy while the grail lift (Stepper 2) is moving
        gripper_cmd_status = 0;
        if (hw->stepper->TryLock(STEPPER_MOTOR_1)) {
            gripper_cmd_status = MoveVertical(hw, gripper_height);
            hw->stepper->Unlock(STEPPER_MOTOR_1);
        }
        break;
      case GRIPPER_CMD_CURSOR:
        gripper_cmd_status = SetDynamixelAngle(hw, GRIPPER_CURSOR, CURSOR_SPEED,
                                               (cursor_pos == 1) ? CURSOR_HOR_ANGLE : CURSOR_VERT_ANGLE);
        break;
      case GRIPPER_CMD_SLIDE:
        gripper_cmd_status = SlideGripper(hw, rack_distance_mm);
        break;
      default:
        continue;
    }
    gripper_cmd = GRIPPER_CMD_NONE;
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
