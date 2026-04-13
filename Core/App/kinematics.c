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


/*// previous year's robot params
#define DIST_CENTER_TO_WHEEL_m			0.1216f // [m]
#define WHEEL_RADIUS_m					0.0245f // [m]
#define	GEAR_RATIO						21.0f
#define ENCODER_TICKS_PER_TURN			4096	// [ticks]
#define CALIBRATION_COEF				0.87f 	// used to equate the real path and path from odometry
*/

// new robot params
#define DIST_CENTER_TO_WHEEL_m 			0.1315f  // [m] 0.155
#define WHEEL_RADIUS_m					0.0245f	// [m]
#define	GEAR_RATIO						28.0f
#define ENCODER_TICKS_PER_TURN			4096	// [ticks]
#define CALIBRATION_COEF				1.158f   // used to equate the real path and path from odometry 1.16f
//#define ANGULAR_CALIBRATION_COEF		1.06f	// used to equate the real angle and angle from odometry


#define cos30 	0.8660254f
#define cos60 	0.5f

///* // from prev year team
// result = linear_wheel_speed(m/s, m/s, m/s)  = inverse_matrix * body_speed(m/s, ms/s, rad/s)
// result = linear_wheel_speed(m/s, m/s, m/s) = inverse_matrix * body_speed(Vx, Vy, W)
float inverse_matrix[9] = {
    -cos30,  cos60, -DIST_CENTER_TO_WHEEL_m,  // Колесо 1
     0,     -1.0f,  -DIST_CENTER_TO_WHEEL_m,  // Колесо 2
     cos30,  cos60, -DIST_CENTER_TO_WHEEL_m   // Колесо 3
};

// result = body_speed(m/s, ms/s, rad/s) = direct_matrix * wheel_speed (m/s, m/s, m/s)
// result = body_speed(Vx, Vy, W) = direct_matrix * wheel_speed (v1, v2, v3)
float direct_matrix[9] = {
    // Строка для Vx (теперь вперед)
    -1.0f / (2.0f * cos30),  0.0f,                   1.0f / (2.0f * cos30),
    
    // Строка для Vy (теперь налево)
     1.0f / 3.0f,           -2.0f / 3.0f,            1.0f / 3.0f,
     
    // Строка для W (угловая скорость - без изменений)
    -1.0f / 3.0f / DIST_CENTER_TO_WHEEL_m, 
    -1.0f / 3.0f / DIST_CENTER_TO_WHEEL_m, 
    -1.0f / 3.0f / DIST_CENTER_TO_WHEEL_m
};

//*/

/*//corrected
// result = linear_wheel_speed(m/s, m/s, m/s)  = inverse_matrix * body_speed(m/s, ms/s, rad/s)
float inverse_matrix[9] = {
    cos30,   cos60,   DIST_CENTER_TO_WHEEL_m,
    0,       -1,      DIST_CENTER_TO_WHEEL_m, 
    -cos30,  cos60,   DIST_CENTER_TO_WHEEL_m
};

// Direct kinematics: body_speed = direct_matrix * wheel_speed
float direct_matrix[9] = {
    1/(2*cos30),   0,               -1/(2*cos30), 
    (2*cos60)/3.0f,       -2/3.0f,          (2*cos60)/3.0f,       
    1/(3*DIST_CENTER_TO_WHEEL_m), 1/(3*DIST_CENTER_TO_WHEEL_m), 1/(3*DIST_CENTER_TO_WHEEL_m)
};
*/

/*
// from API
float direct_matrix[9] = { 	cos30, cos60, DIST_CENTER_TO_WHEEL_m,
							-cos30, cos60, DIST_CENTER_TO_WHEEL_m,
							0,		-1,		DIST_CENTER_TO_WHEEL_m	};

float inverse_matrix[9] = {	1/(2.0f*cos30), -1/(2.0f*cos30), 0,
							0.3333333f, 0.3333333f,	-0.6666666f,
							0.3333333f/DIST_CENTER_TO_WHEEL_m,	0.3333333f/DIST_CENTER_TO_WHEEL_m,	 0.3333333f/DIST_CENTER_TO_WHEEL_m	};
*/

arm_matrix_instance_f32 arm_direct_matrix;
arm_matrix_instance_f32 arm_inverse_matrix;



float ms_to_tpm, ticks_to_m, tpm_to_ms;

													
static void MatrixInit(void)
{
	arm_mat_init_f32(&arm_direct_matrix, 3, 3, direct_matrix);
	arm_mat_init_f32(&arm_inverse_matrix, 3, 3, inverse_matrix);
	
	// wheel's ticks_per_minute = wheel_linear_speed(m/s) * ms_to_tpm
	ms_to_tpm = (60.0f*GEAR_RATIO*ENCODER_TICKS_PER_TURN)/(2*PI*WHEEL_RADIUS_m);

	// linear_path = wheel_ticks_per_minute * ticks_to_m 
	//ticks_to_m =  2*PI*WHEEL_RADIUS_m / (GEAR_RATIO*ENCODER_TICKS_PER_TURN);
	ticks_to_m =  CALIBRATION_COEF * 2*PI*WHEEL_RADIUS_m / (GEAR_RATIO*ENCODER_TICKS_PER_TURN);

	// wheel_linear_speed(m/s) = wheel_ticks_per_minute * ticks_to_m
	tpm_to_ms = 2*PI*WHEEL_RADIUS_m / (60.0f*GEAR_RATIO*ENCODER_TICKS_PER_TURN);
													
}
/*from speed to robot's body (m/s), calculate speed of 3 wheels(ticks per minutes)*/
static void SpeedTo_TPM(struct speed_s* desired_body_speed, int32_t* tpm)
{
	arm_matrix_instance_f32 desired_body_speed_matrix;
	arm_matrix_instance_f32 rad_per_second;
	float ms[3];
	
	arm_mat_init_f32(&desired_body_speed_matrix, 3, 1, (float*)desired_body_speed);
	arm_mat_init_f32(&rad_per_second, 3, 1, ms);
											
	arm_mat_mult_f32(&arm_inverse_matrix, &desired_body_speed_matrix, &rad_per_second);

	// wheel's ticks_per_minute = wheel_linear_speed(m/s) * ms_to_tpm
	tpm[0] = ms[0]*ms_to_tpm;
	tpm[1] = ms[1]*ms_to_tpm;
	tpm[2] = ms[2]*ms_to_tpm;
}

/* from linear speed of 3 wheels(ticks per minutes) to robot's body velocity (m/s, m/s, rad/s)*/
static void TPM_ToSpeed(int32_t* tpm, struct speed_s* real_body_speed)
{
	arm_matrix_instance_f32 real_body_speed_matrix;
	arm_matrix_instance_f32 linear_wheel_speed_matrix;
	float ms[3];
	// find linear speed of 3 wheels (m/s) = ticks_per_minute * tpm_to_ms
	ms[0] = tpm[0]*tpm_to_ms;
	ms[1] = tpm[1]*tpm_to_ms;
	ms[2] = tpm[2]*tpm_to_ms;
	
	arm_mat_init_f32(&real_body_speed_matrix, 3, 1, (float*)real_body_speed);
	arm_mat_init_f32(&linear_wheel_speed_matrix, 3, 1, ms);
											
	arm_mat_mult_f32(&arm_direct_matrix, &linear_wheel_speed_matrix, &real_body_speed_matrix);
}

// d_ticks is in ticks per 10ms
static void UpdatePath(int32_t* d_ticks, struct coord_s* path)
{
	float d_m[3];
	struct coord_s d_coord;

	d_m[0] = d_ticks[0]*ticks_to_m;
	d_m[1] = d_ticks[1]*ticks_to_m;
	d_m[2] = d_ticks[2]*ticks_to_m;

	arm_matrix_instance_f32 d_ms_matrix;
	arm_matrix_instance_f32 d_coord_matrix;
	
	arm_mat_init_f32(&d_ms_matrix, 3, 1, d_m);
	arm_mat_init_f32(&d_coord_matrix, 3, 1, (float*)&d_coord);
											
	//arm_mat_mult_f32(&arm_inverse_matrix, &d_ms_matrix, &d_coord_matrix);
	arm_mat_mult_f32(&arm_direct_matrix, &d_ms_matrix, &d_coord_matrix);

	//path->phi += d_coord.phi * ANGULAR_CALIBRATION_COEF;
	path->phi += d_coord.phi;

	while(path->phi>2*PI)
		path->phi -= 2*PI;
	while(path->phi<-2*PI)
		path->phi += 2*PI;

	path->x += d_coord.x*arm_cos_f32(path->phi) - d_coord.y*arm_sin_f32(path->phi);
	path->y += d_coord.x*arm_sin_f32(path->phi) + d_coord.y*arm_cos_f32(path->phi);
}

const struct kinematics_s Kinematics = {.MatrixInit = MatrixInit,
										.SpeedTo_TPM = SpeedTo_TPM,
										.TPM_ToSpeed = TPM_ToSpeed,
										.UpdatePath = UpdatePath};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
