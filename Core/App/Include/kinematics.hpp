/**
  ******************************************************************************
  * @file    	functions.h
  * @brief   HAL configuration file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include "std_def.hpp"

struct speed_s{
  float dx;
  float dy;
  float dphi;
};

struct coord_s{
  float x;
  float y;
  float phi;
};

/**
 * Kinematics of the 3-wheel omni base.
 * Body speed: m/s, m/s, rad/s. Wheel speed: ticks per minute (TPM).
 * All coefficients are compile-time constants, no init call is needed.
 */
class KinematicsModule{
public:
  static constexpr uint8_t WHEELS_NUMBER = 3;

  /*  Body speed -> wheel speeds  */
  void SpeedTo_TPM(const speed_s* desired_body_speed, int32_t* tpm);
  /*  Wheel speeds -> body speed  */
  void TPM_ToSpeed(const int32_t* tpm, speed_s* real_body_speed);
  /*  Integrate wheel ticks done since the last call into the robot pose  */
  void UpdatePath(const int32_t* d_ticks, coord_s* path);
  /*  Wheel encoder ticks -> wheel path in meters  */
  void WheelTicksTo_m(const int32_t* ticks, float* wheel_path_m);
};

extern KinematicsModule Kinematics;

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
