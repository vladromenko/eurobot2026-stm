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




struct coord_s path;

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
                                    .UpdatePath = UpdatePath};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
