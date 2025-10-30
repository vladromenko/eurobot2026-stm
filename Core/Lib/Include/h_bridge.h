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
#ifndef __H_BRIDGE_H
#define __H_BRIDGE_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.h"


enum h_bridge_e{
  H_BRIDGE1,
  H_BRIDGE2
};

struct h_bridge_s{
  void (*SetMotorSpeed)(enum h_bridge_e h_bridge, int32_t speed);
  void (*Enable)(enum h_bridge_e h_bridge);
  void (*Disable)(enum h_bridge_e h_bridge);
};

extern const struct h_bridge_s H_Bridge;

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
