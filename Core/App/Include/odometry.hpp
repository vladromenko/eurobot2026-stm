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
#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "std_def.hpp"
#include "kinematics.hpp"
#include "maxon.hpp"

/*  Robot pose from the wheel encoders and robot speed through the wheels  */
class OdometryModule{
public:
  constexpr OdometryModule(MaxonDriver& maxon, KinematicsModule& kinematics)
    : maxon(maxon), kinematics(kinematics) {}

  /*  Zero the encoders and the pose  */
  void Init();
  void SetCoord(const coord_s* coord);
  void GetCoord(coord_s* coord);
  void SetSpeed(const speed_s* speed);
  void GetSpeed(speed_s* speed);
  /*  Must be called at 100 Hz  */
  void UpdatePath();

private:
  void SyncTicksReference();

  MaxonDriver& maxon;
  KinematicsModule& kinematics;
  coord_s path = {};
  int32_t ticks_old[3] = {};
};

extern OdometryModule Odometry;

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
