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
#ifndef __BUS_DEVICE_H
#define __BUS_DEVICE_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.hpp"
#include "stm32f4xx_ll_usart.h"

enum bus_devices_e{
  MAXON_MOTOR_1,
  MAXON_MOTOR_2,
  MAXON_MOTOR_3,
  BMS_BOARD,
  SERVO_GRAB,
  SERVO_LIFT11,
	SERVO_LIFT12,
	SERVO_LIFT21,
	SERVO_LIFT22,
  BUS_DEVICES_NUMBER
};

struct bus_device_s{
  USART_TypeDef* usart;
  uint8_t id;
};


extern const struct bus_device_s bus_devices[BUS_DEVICES_NUMBER];


#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
