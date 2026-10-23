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
#include "pwm.hpp"
#include "rc_servo.hpp"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.hpp"

namespace {

constexpr uint32_t ANALOG_SERVO_FREQUENCY_HZ = 50;
constexpr uint32_t SERVO_SIGNAL_PERIOD_US = 20000;

constexpr pwm_group_e pwm_group[] = {PWM_GROUP_1234,PWM_GROUP_1234,PWM_GROUP_1234,PWM_GROUP_1234,
                                        PWM_GROUP_5678,PWM_GROUP_5678,PWM_GROUP_5678,PWM_GROUP_5678};

}

void RC_ServoDriver::SetServoAngle(rc_servo_e servo, uint32_t pulse_min_us, uint32_t pulse_max_us, uint32_t angle_range, uint32_t angle)
{
  uint32_t pulse_us = pulse_min_us + (angle*(pulse_max_us-pulse_min_us))/angle_range;
  uint32_t arr_value = PWM.GetGroupTimAutoreload(pwm_group[servo]);
  uint32_t cc_value = (pulse_us*(arr_value + 1))/SERVO_SIGNAL_PERIOD_US;

  PWM.SetOutput_CC((pwm_output_e)servo, cc_value);
}

void RC_ServoDriver::Enable(rc_servo_e servo)
{
  PWM.SetGroupFrequency(pwm_group[servo], ANALOG_SERVO_FREQUENCY_HZ);
  PWM.Enable((pwm_output_e)servo);
}

void RC_ServoDriver::Disable(rc_servo_e servo)
{
  PWM.Disable((pwm_output_e)servo);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
