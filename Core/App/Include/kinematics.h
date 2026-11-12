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

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.h"
	 
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

/*  Nominal robot geometry, compiled in kinematics.c  */
struct odometry_geometry_s{
  float wheel_radius_m;
  float dist_center_to_wheel_m;
  float gear_ratio;
  float encoder_ticks_per_turn;
};

/*  Real motion per nominal unit: wheel path per wheel, then body dx, dy, dphi  */
struct odometry_calibration_s{
  float wheel_coef[3];
  float x_coef;
  float y_coef;
  float phi_coef;
};

void SpeedTo_TPM_2(struct speed_s* speed, int32_t* tpm);

struct kinematics_s{
    void (*MatrixInit)(void);
    void (*SpeedTo_TPM)(struct speed_s* speed, int32_t* tpm);
    void (*TPM_ToSpeed)(int32_t* tpm, struct speed_s* speed);
    void (*UpdatePath)(int32_t* d_ticks, struct coord_s* path);
    void (*GetGeometry)(struct odometry_geometry_s* geometry);
    uint8_t (*SetCalibration)(const struct odometry_calibration_s* new_calibration);
    void (*GetCalibration)(struct odometry_calibration_s* current_calibration);
};

extern const struct kinematics_s Kinematics;


#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
