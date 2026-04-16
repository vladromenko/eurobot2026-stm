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




struct coord_s path;

static void Init(void)
{
  Maxon.SetTotalTicks(bus_devices[MAXON_MOTOR_1].usart, bus_devices[MAXON_MOTOR_1].id, 0);
  Maxon.SetTotalTicks(bus_devices[MAXON_MOTOR_2].usart, bus_devices[MAXON_MOTOR_2].id, 0);
  Maxon.SetTotalTicks(bus_devices[MAXON_MOTOR_3].usart, bus_devices[MAXON_MOTOR_3].id, 0);

  path.x = 0; //0.25
  path.y = 0; //2.65
  path.phi = 0; //-2.0944
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
  int32_t tpm[3];

  Kinematics.SpeedTo_TPM(speed,tpm);

  Maxon.Set_TPM(bus_devices[MAXON_MOTOR_3].usart, bus_devices[MAXON_MOTOR_3].id, tpm[2]);
  Maxon.Set_TPM(bus_devices[MAXON_MOTOR_2].usart, bus_devices[MAXON_MOTOR_2].id, tpm[1]);
  Maxon.Set_TPM(bus_devices[MAXON_MOTOR_1].usart, bus_devices[MAXON_MOTOR_1].id, tpm[0]);
  //Maxon.Set_TPM(bus_devices[MAXON_MOTOR_2].usart, bus_devices[MAXON_MOTOR_2].id, tpm[1]);
  //Maxon.Set_TPM(bus_devices[MAXON_MOTOR_3].usart, bus_devices[MAXON_MOTOR_3].id, tpm[2]);
}

static void GetSpeed(struct speed_s* speed)
{
  int32_t tpm[3];

  Maxon.Get_TPM(bus_devices[MAXON_MOTOR_1].usart, bus_devices[MAXON_MOTOR_1].id, tpm);
  Maxon.Get_TPM(bus_devices[MAXON_MOTOR_2].usart, bus_devices[MAXON_MOTOR_2].id, tpm+1);
  Maxon.Get_TPM(bus_devices[MAXON_MOTOR_3].usart, bus_devices[MAXON_MOTOR_3].id, tpm+2);
	tpm[0] = tpm[0] * 0.86f;
	tpm[1] = tpm[1] * 0.86f;
	tpm[2] = tpm[2] * 0.86f;
  Kinematics.TPM_ToSpeed(tpm,speed);
}

// this update path will be called with frequency of 100 Hz
static void UpdatePath(void)
{
  static int32_t ticks_old[3] = {0};

  int32_t ticks_new[3];
  int32_t d_ticks[3];
  uint8_t error = 0;

  error |= Maxon.GetTotalTicks(bus_devices[MAXON_MOTOR_1].usart, bus_devices[MAXON_MOTOR_1].id, ticks_new);
  error |= Maxon.GetTotalTicks(bus_devices[MAXON_MOTOR_2].usart, bus_devices[MAXON_MOTOR_2].id, ticks_new+1);
  error |= Maxon.GetTotalTicks(bus_devices[MAXON_MOTOR_3].usart, bus_devices[MAXON_MOTOR_3].id, ticks_new+2);

  if(error)
    return;

  d_ticks[0] = ticks_new[0] - ticks_old[0];
  d_ticks[1] = ticks_new[1] - ticks_old[1];
  d_ticks[2] = ticks_new[2] - ticks_old[2];

  // d_ticks is in ticks per 10ms
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
