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

#include "odometry.h" 
#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.h"
#include "byte2word.h" 
#include "maxon.h" 

#include "kinematics.h"
#include "bus_device.h"
#include "hw_instance.h"




#define WHEEL_ROTATION_TIMEOUT_MS   20000
#define WHEEL_ROTATION_SETTLE_MS    300

enum wheel_rotation_step_e{
  ROTATION_STEP_NONE,
  ROTATION_STEP_MOVE,
  ROTATION_STEP_SETTLE
};

struct coord_s path;

/*  Single wheel rotation for calibration: started by the terminal, stopped by the odometry task  */
static volatile enum wheel_rotation_step_e rotation_step = ROTATION_STEP_NONE;
static volatile enum wheel_rotation_state_e rotation_state = WHEEL_ROTATION_IDLE;
static volatile int32_t rotation_ticks;
static enum bus_devices_e rotation_device;
static int32_t rotation_target_ticks;
static int32_t rotation_start_ticks;
static TickType_t rotation_time;

static void CancelWheelRotation(void)
{
  if(rotation_step != ROTATION_STEP_NONE){
    rotation_step = ROTATION_STEP_NONE;
    rotation_state = WHEEL_ROTATION_ABORTED;
  }
}

static void Init(void)
{
  const struct hw_instance_s* hw = GetHwInstance();
  hw->maxon->SetTotalTicks(hw->bus_devices[MAXON_MOTOR_1].usart, hw->bus_devices[MAXON_MOTOR_1].id, 0);
  hw->maxon->SetTotalTicks(hw->bus_devices[MAXON_MOTOR_2].usart, hw->bus_devices[MAXON_MOTOR_2].id, 0);
  hw->maxon->SetTotalTicks(hw->bus_devices[MAXON_MOTOR_3].usart, hw->bus_devices[MAXON_MOTOR_3].id, 0);

  path.x = 0;
  path.y = 0;
  path.phi = 0;
}

static void SetCoord(struct coord_s* coord)
{
  path.x = coord->x;
  path.y = coord->y;
  path.phi = coord->phi;
}

static void GetCoord(struct coord_s* coord)
{
  coord->x = path.x;
  coord->y = path.y;
  coord->phi = path.phi;
}

static void SetSpeed(struct speed_s* speed)
{
  const struct hw_instance_s* hw = GetHwInstance();
  int32_t tpm[3];

  CancelWheelRotation();

  Kinematics.SpeedTo_TPM(speed,tpm);

  hw->maxon->Set_TPM(hw->bus_devices[MAXON_MOTOR_1].usart, hw->bus_devices[MAXON_MOTOR_1].id, tpm[0]);
  hw->maxon->Set_TPM(hw->bus_devices[MAXON_MOTOR_2].usart, hw->bus_devices[MAXON_MOTOR_2].id, tpm[1]);
  hw->maxon->Set_TPM(hw->bus_devices[MAXON_MOTOR_3].usart, hw->bus_devices[MAXON_MOTOR_3].id, tpm[2]);
}

static void GetSpeed(struct speed_s* speed)
{
  const struct hw_instance_s* hw = GetHwInstance();
  int32_t tpm[3];

  hw->maxon->Get_TPM(hw->bus_devices[MAXON_MOTOR_1].usart, hw->bus_devices[MAXON_MOTOR_1].id, tpm);
  hw->maxon->Get_TPM(hw->bus_devices[MAXON_MOTOR_2].usart, hw->bus_devices[MAXON_MOTOR_2].id, tpm+1);
  hw->maxon->Get_TPM(hw->bus_devices[MAXON_MOTOR_3].usart, hw->bus_devices[MAXON_MOTOR_3].id, tpm+2);

  Kinematics.TPM_ToSpeed(tpm,speed);
}

static uint8_t GetWheelTicks(int32_t* ticks)
{
  const struct hw_instance_s* hw = GetHwInstance();
  uint8_t error = 0;

  error |= hw->maxon->GetTotalTicks(hw->bus_devices[MAXON_MOTOR_1].usart, hw->bus_devices[MAXON_MOTOR_1].id, ticks);
  error |= hw->maxon->GetTotalTicks(hw->bus_devices[MAXON_MOTOR_2].usart, hw->bus_devices[MAXON_MOTOR_2].id, ticks+1);
  error |= hw->maxon->GetTotalTicks(hw->bus_devices[MAXON_MOTOR_3].usart, hw->bus_devices[MAXON_MOTOR_3].id, ticks+2);

  return error;
}

// Spins one wheel by |ticks| encoder ticks with the given signed speed, other wheels are stopped
static uint8_t StartWheelRotation(uint8_t wheel_number, int32_t ticks, int32_t tpm)
{
  const struct hw_instance_s* hw = GetHwInstance();
  int32_t start_ticks;

  if((wheel_number < 1) || (wheel_number > 3) || (ticks <= 0) || (tpm == 0))
    return 0;
  if(rotation_step != ROTATION_STEP_NONE)
    return 0;

  rotation_device = (enum bus_devices_e)(MAXON_MOTOR_1 + wheel_number - 1);
  if(hw->maxon->GetTotalTicks(hw->bus_devices[rotation_device].usart, hw->bus_devices[rotation_device].id, &start_ticks))
    return 0;

  rotation_start_ticks = start_ticks;
  rotation_target_ticks = ticks;
  rotation_ticks = 0;
  rotation_time = xTaskGetTickCount();
  rotation_state = WHEEL_ROTATION_RUNNING;

  for(enum bus_devices_e device = MAXON_MOTOR_1; device <= MAXON_MOTOR_3; device++)
    hw->maxon->Set_TPM(hw->bus_devices[device].usart, hw->bus_devices[device].id, (device == rotation_device) ? tpm : 0);

  rotation_step = ROTATION_STEP_MOVE;
  return 1;
}

static void GetWheelRotation(enum wheel_rotation_state_e* state, int32_t* ticks)
{
  *state = rotation_state;
  *ticks = rotation_ticks;
}

static void ProcessWheelRotation(const int32_t* ticks, uint8_t error)
{
  const struct hw_instance_s* hw = GetHwInstance();
  TickType_t now = xTaskGetTickCount();
  enum wheel_rotation_step_e step = rotation_step;

  if(step == ROTATION_STEP_NONE)
    return;

  if(!error)
    rotation_ticks = ticks[rotation_device - MAXON_MOTOR_1] - rotation_start_ticks;

  if(step == ROTATION_STEP_MOVE){
    int32_t abs_ticks = (rotation_ticks < 0) ? -rotation_ticks : rotation_ticks;
    uint8_t is_timeout = (now - rotation_time) >= pdMS_TO_TICKS(WHEEL_ROTATION_TIMEOUT_MS);

    if((abs_ticks >= rotation_target_ticks) || is_timeout){
      hw->maxon->Set_TPM(hw->bus_devices[rotation_device].usart, hw->bus_devices[rotation_device].id, 0);
      rotation_time = now;
      if(is_timeout && (abs_ticks < rotation_target_ticks)){
        rotation_state = WHEEL_ROTATION_TIMEOUT;
        rotation_step = ROTATION_STEP_NONE;
      }
      else
        rotation_step = ROTATION_STEP_SETTLE;
    }
  }
  // Keep counting while the wheel coasts, so the reported ticks include the overshoot
  else if(step == ROTATION_STEP_SETTLE){
    if((now - rotation_time) >= pdMS_TO_TICKS(WHEEL_ROTATION_SETTLE_MS)){
      rotation_state = WHEEL_ROTATION_DONE;
      rotation_step = ROTATION_STEP_NONE;
    }
  }
}

static void UpdatePath(void)
{
  const struct hw_instance_s* hw = GetHwInstance();
  static int32_t ticks_old[3] = {0};

  int32_t ticks_new[3];
  int32_t d_ticks[3];
  uint8_t error = 0;

  error |= hw->maxon->GetTotalTicks(hw->bus_devices[MAXON_MOTOR_1].usart, hw->bus_devices[MAXON_MOTOR_1].id, ticks_new);
  error |= hw->maxon->GetTotalTicks(hw->bus_devices[MAXON_MOTOR_2].usart, hw->bus_devices[MAXON_MOTOR_2].id, ticks_new+1);
  error |= hw->maxon->GetTotalTicks(hw->bus_devices[MAXON_MOTOR_3].usart, hw->bus_devices[MAXON_MOTOR_3].id, ticks_new+2);

  ProcessWheelRotation(ticks_new, error);

  if(error)
    return;

  d_ticks[0] = ticks_new[0] - ticks_old[0];
  d_ticks[1] = ticks_new[1] - ticks_old[1];
  d_ticks[2] = ticks_new[2] - ticks_old[2];

  Kinematics.UpdatePath(d_ticks,&path);

  memcpy(ticks_old,ticks_new,12);
}


const struct odometry_s Odometry = {.Init = Init,
                                    .SetCoord = SetCoord,
                                    .GetCoord = GetCoord,
                                    .SetSpeed = SetSpeed,
                                    .GetSpeed = GetSpeed,
                                    .UpdatePath = UpdatePath,
                                    .GetWheelTicks = GetWheelTicks,
                                    .StartWheelRotation = StartWheelRotation,
                                    .GetWheelRotation = GetWheelRotation};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
