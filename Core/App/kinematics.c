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

// new robot params
#define DIST_CENTER_TO_WHEEL_m             0.1217f  // [m]
#define WHEEL_RADIUS_m                     0.0295f  // [m]
#define GEAR_RATIO                         28.0f
#define ENCODER_TICKS_PER_TURN             4096     // [ticks]

// Final low-speed odometry calibration.
#define CMD_CALIBRATION_COEF               1.0f
#define ODOMETRY_CALIBRATION_COEF          0.988f
#define ODOMETRY_X_COEF                    0.988f
#define ODOMETRY_Y_COEF                    1.0f
#define ANGULAR_CMD_CALIBRATION_COEF       1.0f
#define ANGULAR_ODOMETRY_CALIBRATION_COEF  0.988f
#define WHEEL_1_ODOMETRY_COEF              1.003f
#define WHEEL_2_ODOMETRY_COEF              0.984f
#define WHEEL_3_ODOMETRY_COEF              0.997f

#define cos30  0.8660254f
#define cos60  0.5f

float inverse_matrix[9] = {
     cos30, -cos60, -DIST_CENTER_TO_WHEEL_m,
     0.0f,   1.0f,  -DIST_CENTER_TO_WHEEL_m,
    -cos30, -cos60, -DIST_CENTER_TO_WHEEL_m
};

float direct_matrix[9] = {
     1.0f / (2.0f * cos30),  0.0f,                  -1.0f / (2.0f * cos30),
    -1.0f / 3.0f,            2.0f / 3.0f,          -1.0f / 3.0f,
    -1.0f / 3.0f / DIST_CENTER_TO_WHEEL_m,
    -1.0f / 3.0f / DIST_CENTER_TO_WHEEL_m,
    -1.0f / 3.0f / DIST_CENTER_TO_WHEEL_m
};

arm_matrix_instance_f32 arm_direct_matrix;
arm_matrix_instance_f32 arm_inverse_matrix;

float ms_to_tpm, ticks_to_m, tpm_to_ms;
float wheel_ticks_to_m[3], wheel_tpm_to_ms[3];

static void MatrixInit(void)
{
    arm_mat_init_f32(&arm_direct_matrix, 3, 3, direct_matrix);
    arm_mat_init_f32(&arm_inverse_matrix, 3, 3, inverse_matrix);

    // wheel ticks_per_minute = wheel_linear_speed(m/s) * ms_to_tpm
    ms_to_tpm = (60.0f * GEAR_RATIO * ENCODER_TICKS_PER_TURN) /
                (CMD_CALIBRATION_COEF * 2 * PI * WHEEL_RADIUS_m);

    ticks_to_m = ODOMETRY_CALIBRATION_COEF * 2 * PI * WHEEL_RADIUS_m /
                 (GEAR_RATIO * ENCODER_TICKS_PER_TURN);
    wheel_ticks_to_m[0] = ticks_to_m * WHEEL_1_ODOMETRY_COEF;
    wheel_ticks_to_m[1] = ticks_to_m * WHEEL_2_ODOMETRY_COEF;
    wheel_ticks_to_m[2] = ticks_to_m * WHEEL_3_ODOMETRY_COEF;

    tpm_to_ms = ODOMETRY_CALIBRATION_COEF * 2 * PI * WHEEL_RADIUS_m /
                (60.0f * GEAR_RATIO * ENCODER_TICKS_PER_TURN);
    wheel_tpm_to_ms[0] = wheel_ticks_to_m[0] / 60.0f;
    wheel_tpm_to_ms[1] = wheel_ticks_to_m[1] / 60.0f;
    wheel_tpm_to_ms[2] = wheel_ticks_to_m[2] / 60.0f;
}

/* from body speed (m/s, m/s, rad/s) to wheel speeds (ticks per minute) */
static void SpeedTo_TPM(struct speed_s* desired_body_speed, int32_t* tpm)
{
    arm_matrix_instance_f32 desired_body_speed_matrix;
    arm_matrix_instance_f32 rad_per_second;
    struct speed_s calibrated_body_speed;
    float ms[3];

    calibrated_body_speed = *desired_body_speed;
    calibrated_body_speed.dphi *= ANGULAR_CMD_CALIBRATION_COEF;

    arm_mat_init_f32(&desired_body_speed_matrix, 3, 1, (float*)&calibrated_body_speed);
    arm_mat_init_f32(&rad_per_second, 3, 1, ms);

    arm_mat_mult_f32(&arm_inverse_matrix, &desired_body_speed_matrix, &rad_per_second);

    tpm[0] = ms[0] * ms_to_tpm;
    tpm[1] = ms[1] * ms_to_tpm;
    tpm[2] = ms[2] * ms_to_tpm;
}

/* from wheel speeds (ticks per minute) to body speed (m/s, m/s, rad/s) */
static void TPM_ToSpeed(int32_t* tpm, struct speed_s* real_body_speed)
{
    arm_matrix_instance_f32 real_body_speed_matrix;
    arm_matrix_instance_f32 linear_wheel_speed_matrix;
    float ms[3];

    ms[0] = tpm[0] * wheel_tpm_to_ms[0];
    ms[1] = tpm[1] * wheel_tpm_to_ms[1];
    ms[2] = tpm[2] * wheel_tpm_to_ms[2];

    arm_mat_init_f32(&real_body_speed_matrix, 3, 1, (float*)real_body_speed);
    arm_mat_init_f32(&linear_wheel_speed_matrix, 3, 1, ms);

    arm_mat_mult_f32(&arm_direct_matrix, &linear_wheel_speed_matrix, &real_body_speed_matrix);
    real_body_speed->dx *= ODOMETRY_X_COEF;
    real_body_speed->dy *= ODOMETRY_Y_COEF;
    real_body_speed->dphi *= ANGULAR_ODOMETRY_CALIBRATION_COEF;
}

// d_ticks is in ticks per 10 ms
static void UpdatePath(int32_t* d_ticks, struct coord_s* path)
{
    float d_m[3];
    struct coord_s d_coord;
    float dphi;
    float phi_mid;
    arm_matrix_instance_f32 d_ms_matrix;
    arm_matrix_instance_f32 d_coord_matrix;

    d_m[0] = d_ticks[0] * wheel_ticks_to_m[0];
    d_m[1] = d_ticks[1] * wheel_ticks_to_m[1];
    d_m[2] = d_ticks[2] * wheel_ticks_to_m[2];

    arm_mat_init_f32(&d_ms_matrix, 3, 1, d_m);
    arm_mat_init_f32(&d_coord_matrix, 3, 1, (float*)&d_coord);

    arm_mat_mult_f32(&arm_direct_matrix, &d_ms_matrix, &d_coord_matrix);
    d_coord.x *= ODOMETRY_X_COEF;
    d_coord.y *= ODOMETRY_Y_COEF;

    dphi = d_coord.phi * ANGULAR_ODOMETRY_CALIBRATION_COEF;
    phi_mid = path->phi + 0.5f * dphi;

    path->phi += dphi;

    while (path->phi > PI)
        path->phi -= 2 * PI;
    while (path->phi < -PI)
        path->phi += 2 * PI;

    path->x += d_coord.x * arm_cos_f32(phi_mid) - d_coord.y * arm_sin_f32(phi_mid);
    path->y += d_coord.x * arm_sin_f32(phi_mid) + d_coord.y * arm_cos_f32(phi_mid);
}

const struct kinematics_s Kinematics = {
    .MatrixInit = MatrixInit,
    .SpeedTo_TPM = SpeedTo_TPM,
    .TPM_ToSpeed = TPM_ToSpeed,
    .UpdatePath = UpdatePath
};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
