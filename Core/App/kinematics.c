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
#include "kinematics.h"
#include "arm_math.h"
#include "robot_behavior_routine.h" 
#include "stm32f4xx_ll_tim.h"
#include "board_gpio_defines.h"
#include "FreeRTOS.h"
#include "task.h"



#define DIST_CENTER_TO_WHEEL_m 			0.1075f // [m]
#define WHEEL_RADIUS_m					0.03f   // [m]
#define	GEAR_RATIO						21.0f
#define ENCODER_TICKS_PER_TURN			4096	// [ticks]

// Odometry calibration, measured with tools/odom_calibrate.py
#define WHEEL_1_ODOMETRY_COEF			1.0f
#define WHEEL_2_ODOMETRY_COEF			1.0f
#define WHEEL_3_ODOMETRY_COEF			1.0f
#define ODOMETRY_X_COEF					1.0f
#define ODOMETRY_Y_COEF					1.0f
#define ODOMETRY_PHI_COEF				1.0f

#define CALIBRATION_COEF_MIN			0.5f
#define CALIBRATION_COEF_MAX			1.5f




#define cos30 	0.8660254f
#define cos60 	0.5f

float direct_matrix[9] = { 	cos30, cos60, 1,
							-cos30, cos60, 1,
							0,		-1,		1	};

float inverse_matrix[9] = {	1/(2.0f*cos30), -1/(2.0f*cos30), 0,
							0.3333333f, 0.3333333f,	-0.6666666f,
							0.3333333f,	0.3333333f,	 0.3333333f	};

														

arm_matrix_instance_f32 arm_direct_matrix;
arm_matrix_instance_f32 arm_inverse_matrix;



float ms_to_tpm, ticks_to_m;

static struct odometry_calibration_s calibration = {.wheel_coef = {WHEEL_1_ODOMETRY_COEF, WHEEL_2_ODOMETRY_COEF, WHEEL_3_ODOMETRY_COEF},
													.x_coef = ODOMETRY_X_COEF,
													.y_coef = ODOMETRY_Y_COEF,
													.phi_coef = ODOMETRY_PHI_COEF};

													
static void MatrixInit(void)
{
	arm_mat_init_f32(&arm_direct_matrix, 3, 3, direct_matrix);
	arm_mat_init_f32(&arm_inverse_matrix, 3, 3, inverse_matrix);
	
	ms_to_tpm = (30.0f*GEAR_RATIO)/(PI*WHEEL_RADIUS_m)*ENCODER_TICKS_PER_TURN;
	ticks_to_m = (2*PI*WHEEL_RADIUS_m)/GEAR_RATIO/ENCODER_TICKS_PER_TURN;
	

	float direct_gain[9] = {1,0,0,
							0,1,0,
							0,0,DIST_CENTER_TO_WHEEL_m};
	
	arm_matrix_instance_f32 direct_gain_matrix;
	arm_mat_init_f32(&direct_gain_matrix, 3, 3, direct_gain);
												
																									
	arm_mat_mult_f32(&arm_direct_matrix, &direct_gain_matrix, &arm_direct_matrix);
													
	
													
	float inverse_gain[9] = {1,0,0,
							0,1,0,
							0,0,1/DIST_CENTER_TO_WHEEL_m};
	
	arm_matrix_instance_f32 inverse_gain_matrix;
	arm_mat_init_f32(&inverse_gain_matrix, 3, 3, inverse_gain);
																									
	arm_mat_mult_f32(&inverse_gain_matrix, &arm_inverse_matrix, &arm_inverse_matrix);
}

static void SpeedTo_TPM(struct speed_s* speed, int32_t* tpm)
{
	arm_matrix_instance_f32 speed_matrix;
	arm_matrix_instance_f32 ms_matrix;
	struct speed_s nominal_speed;
	float ms[3];

	// Coefs scale nominal motion into real motion, so commands are divided by them
	nominal_speed.dx = speed->dx/calibration.x_coef;
	nominal_speed.dy = speed->dy/calibration.y_coef;
	nominal_speed.dphi = speed->dphi/calibration.phi_coef;
	
	arm_mat_init_f32(&speed_matrix, 3, 1, (float*)&nominal_speed);
	arm_mat_init_f32(&ms_matrix, 3, 1, ms);
											
	arm_mat_mult_f32(&arm_direct_matrix, &speed_matrix, &ms_matrix);

	tpm[0] = ms[0]/calibration.wheel_coef[0]*ms_to_tpm;
	tpm[1] = ms[1]/calibration.wheel_coef[1]*ms_to_tpm;
	tpm[2] = ms[2]/calibration.wheel_coef[2]*ms_to_tpm;
}

static void TPM_ToSpeed(int32_t* tpm, struct speed_s* speed)
{
	arm_matrix_instance_f32 speed_matrix;
	arm_matrix_instance_f32 ms_matrix;
	float ms[3];

	ms[0] = tpm[0]/ms_to_tpm*calibration.wheel_coef[0];
	ms[1] = tpm[1]/ms_to_tpm*calibration.wheel_coef[1];
	ms[2] = tpm[2]/ms_to_tpm*calibration.wheel_coef[2];
	
	arm_mat_init_f32(&speed_matrix, 3, 1, (float*)speed);
	arm_mat_init_f32(&ms_matrix, 3, 1, ms);
											
	arm_mat_mult_f32(&arm_inverse_matrix, &ms_matrix, &speed_matrix);

	speed->dx *= calibration.x_coef;
	speed->dy *= calibration.y_coef;
	speed->dphi *= calibration.phi_coef;
}

static void UpdatePath(int32_t* d_ticks, struct coord_s* path)
{
	float d_m[3];
	struct coord_s d_coord;

	d_m[0] = d_ticks[0]*ticks_to_m*calibration.wheel_coef[0];
	d_m[1] = d_ticks[1]*ticks_to_m*calibration.wheel_coef[1];
	d_m[2] = d_ticks[2]*ticks_to_m*calibration.wheel_coef[2];

	arm_matrix_instance_f32 d_ms_matrix;
	arm_matrix_instance_f32 d_coord_matrix;
	
	arm_mat_init_f32(&d_ms_matrix, 3, 1, d_m);
	arm_mat_init_f32(&d_coord_matrix, 3, 1, (float*)&d_coord);
											
	arm_mat_mult_f32(&arm_inverse_matrix, &d_ms_matrix, &d_coord_matrix);

	d_coord.x *= calibration.x_coef;
	d_coord.y *= calibration.y_coef;
	d_coord.phi *= calibration.phi_coef;

	path->phi += d_coord.phi;

	while(path->phi>2*PI)
		path->phi -= 2*PI;
	while(path->phi<-2*PI)
		path->phi += 2*PI;

	path->x += d_coord.x*arm_cos_f32(path->phi) - d_coord.y*arm_sin_f32(path->phi);
	path->y += d_coord.x*arm_sin_f32(path->phi) + d_coord.y*arm_cos_f32(path->phi);
}

void SpeedTo_TPM_2(struct speed_s* speed, int32_t* tpm)
{
	arm_matrix_instance_f32 speed_matrix;
	arm_matrix_instance_f32 ms_matrix;
	float ms[3];
	
	arm_mat_init_f32(&speed_matrix, 3, 1, (float*)speed);
	arm_mat_init_f32(&ms_matrix, 3, 1, ms);
											
	arm_mat_mult_f32(&arm_direct_matrix, &speed_matrix, &ms_matrix);
LL_GPIO_SetOutputPin(GPIOA, PWM7_PA2);
	tpm[0] = ms[0]*ms_to_tpm;
	tpm[1] = ms[1]*ms_to_tpm;
	tpm[2] = ms[2]*ms_to_tpm;

	
}

static void GetGeometry(struct odometry_geometry_s* geometry)
{
	geometry->wheel_radius_m = WHEEL_RADIUS_m;
	geometry->dist_center_to_wheel_m = DIST_CENTER_TO_WHEEL_m;
	geometry->gear_ratio = GEAR_RATIO;
	geometry->encoder_ticks_per_turn = ENCODER_TICKS_PER_TURN;
}

static uint8_t IsCoefValid(float coef)
{
	// Written so that NaN is rejected too
	return (coef >= CALIBRATION_COEF_MIN) && (coef <= CALIBRATION_COEF_MAX);
}

// Coefs live in RAM until reboot; bake the final ones into the defines above
static uint8_t SetCalibration(const struct odometry_calibration_s* new_calibration)
{
	if(!IsCoefValid(new_calibration->wheel_coef[0]) || !IsCoefValid(new_calibration->wheel_coef[1]) ||
	   !IsCoefValid(new_calibration->wheel_coef[2]) || !IsCoefValid(new_calibration->x_coef) ||
	   !IsCoefValid(new_calibration->y_coef) || !IsCoefValid(new_calibration->phi_coef))
		return 0;

	// Odometry task reads the coefs every 10 ms, so update them in one piece
	taskENTER_CRITICAL();
	calibration = *new_calibration;
	taskEXIT_CRITICAL();

	return 1;
}

static void GetCalibration(struct odometry_calibration_s* current_calibration)
{
	taskENTER_CRITICAL();
	*current_calibration = calibration;
	taskEXIT_CRITICAL();
}

const struct kinematics_s Kinematics = {.MatrixInit = MatrixInit,
										.SpeedTo_TPM = SpeedTo_TPM,
										.TPM_ToSpeed = TPM_ToSpeed,
										.UpdatePath = UpdatePath,
										.GetGeometry = GetGeometry,
										.SetCalibration = SetCalibration,
										.GetCalibration = GetCalibration};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
