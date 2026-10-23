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
#include "odometry.hpp" 
#include "bus_device.hpp"


constinit OdometryModule Odometry{Maxon, Kinematics};


void OdometryModule::SyncTicksReference()
{
  uint8_t error = 0;

  error |= maxon.GetTotalTicks(bus_devices[MAXON_MOTOR_1].id, ticks_old);
  error |= maxon.GetTotalTicks(bus_devices[MAXON_MOTOR_2].id, ticks_old + 1);
  error |= maxon.GetTotalTicks(bus_devices[MAXON_MOTOR_3].id, ticks_old + 2);

  if(error){
    ticks_old[0] = 0;
    ticks_old[1] = 0;
    ticks_old[2] = 0;
  }
}

void OdometryModule::Init()
{
  maxon.SetTotalTicks(bus_devices[MAXON_MOTOR_1].id, 0);
  maxon.SetTotalTicks(bus_devices[MAXON_MOTOR_2].id, 0);
  maxon.SetTotalTicks(bus_devices[MAXON_MOTOR_3].id, 0);

  path.x = 0; //0.25
  path.y = 0; //2.65
  path.phi = 0; //-2.0944

  ticks_old[0] = 0;
  ticks_old[1] = 0;
  ticks_old[2] = 0;
}

void OdometryModule::SetCoord(const coord_s* coord)
{
  path = *coord;

  // Start integrating the new pose from current encoder totals.
  SyncTicksReference();
}

void OdometryModule::GetCoord(coord_s* coord)
{
  *coord = path;
}

void OdometryModule::SetSpeed(const speed_s* speed)
{
  int32_t tpm[3];

  kinematics.SpeedTo_TPM(speed, tpm);

  maxon.Set_TPM(bus_devices[MAXON_MOTOR_1].id, tpm[0]);
  maxon.Set_TPM(bus_devices[MAXON_MOTOR_2].id, tpm[1]);
  maxon.Set_TPM(bus_devices[MAXON_MOTOR_3].id, tpm[2]);
}

void OdometryModule::GetSpeed(speed_s* speed)
{
  /*  Zero speed for a wheel that did not answer  */
  int32_t tpm[3] = {};

  maxon.Get_TPM(bus_devices[MAXON_MOTOR_1].id, tpm);
  maxon.Get_TPM(bus_devices[MAXON_MOTOR_2].id, tpm+1);
  maxon.Get_TPM(bus_devices[MAXON_MOTOR_3].id, tpm+2);
  kinematics.TPM_ToSpeed(tpm, speed);
}

// this update path will be called with frequency of 100 Hz
void OdometryModule::UpdatePath()
{
  int32_t ticks_new[3];
  int32_t d_ticks[3];
  uint8_t error = 0;

  error |= maxon.GetTotalTicks(bus_devices[MAXON_MOTOR_1].id, ticks_new);
  error |= maxon.GetTotalTicks(bus_devices[MAXON_MOTOR_2].id, ticks_new+1);
  error |= maxon.GetTotalTicks(bus_devices[MAXON_MOTOR_3].id, ticks_new+2);

  if(error)
    return;

  for(uint8_t ik = 0; ik < 3; ik++){
    d_ticks[ik] = ticks_new[ik] - ticks_old[ik];
    ticks_old[ik] = ticks_new[ik];
  }

  // d_ticks is in ticks per 10ms
  kinematics.UpdatePath(d_ticks, &path);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
