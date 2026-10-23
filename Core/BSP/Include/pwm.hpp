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
#ifndef __PWM_H
#define __PWM_H

#include "std_def.hpp"


enum pwm_output_e{
  PWM_OUTPUT_1,
  PWM_OUTPUT_2,
  PWM_OUTPUT_3,
  PWM_OUTPUT_4,
  PWM_OUTPUT_5,
  PWM_OUTPUT_6,
  PWM_OUTPUT_7,
  PWM_OUTPUT_8
};

enum pwm_group_e{
  PWM_GROUP_1234,
  PWM_GROUP_5678
};

/*  PWM1..4 on TIM4, PWM5..8 on TIM5  */
class PWM_Driver{
public:
  uint32_t GetGroupTimAutoreload(pwm_group_e pwm_group);
  void SetOutput_CC(pwm_output_e pwm_output, uint32_t cc_value);
  void SetGroupFrequency(pwm_group_e pwm_group, uint32_t frequency_hz);
  void SetOutputDutyCycle(pwm_output_e pwm_output, uint32_t duty_cycle);
  void Enable(pwm_output_e pwm_output);
  void Disable(pwm_output_e pwm_output);
};

/*  The object lives in HwInstance (hw_instance.hpp); this is a shortcut to it  */
extern PWM_Driver& PWM;

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
