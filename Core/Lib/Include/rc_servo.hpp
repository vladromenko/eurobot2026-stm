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
#ifndef __RC_SERVO_CONTROL_H
#define __RC_SERVO_CONTROL_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.hpp"


enum rc_servo_e{
  RC_SERVO_1,
  RC_SERVO_2,
  RC_SERVO_3,
  RC_SERVO_4,
  RC_SERVO_5,
  RC_SERVO_6,
  RC_SERVO_7,
  RC_SERVO_8
};

struct rc_servo_s{
  void (*SetServoAngle)(enum rc_servo_e servo, uint32_t pulse_min_us, uint32_t pulse_max_us, uint32_t angle_range, uint32_t angle);
  void (*Enable)(enum rc_servo_e servo);
  void (*Disable)(enum rc_servo_e servo);
};

extern struct rc_servo_s RC_Servo;

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
