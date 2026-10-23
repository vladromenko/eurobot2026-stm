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

#ifdef __cplusplus
 extern "C" {
#endif
	 
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

struct pwm_s{
  uint32_t (*GetGroupTimAutoreload)(enum pwm_group_e pwm_group);
  void (*SetOutput_CC)(enum pwm_output_e pwm_output, uint32_t cc_value);
  void (*SetGroupFrequency)(enum pwm_group_e pwm_group, uint32_t frequency_hz);
  void (*SetOutputDutyCycle)(enum pwm_output_e pwm_output, uint32_t duty_cycle);
  void (*Enable)(enum pwm_output_e pwm_output);
  void (*Disable)(enum pwm_output_e pwm_output);
};

extern const struct pwm_s PWM;

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
