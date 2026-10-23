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
#ifndef __BUTTONS_ENUM_H
#define __BUTTONS_ENUM_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.hpp"


enum buttons_e{
  STARTING_CORD_BTN,
  PLAYGROUND_SIDE_BTN,
  STRATEGY_BTN,
  MAXON_ENABLE_BTN,
  LINEAR_SWITCH_1_BTN,
  LINEAR_SWITCH_2_BTN,
	LINEAR_SWITCH_3_BTN,
	LINEAR_SWITCH_4_BTN,
  LINEAR_SWITCH_5_BTN,
	LINEAR_SWITCH_6_BTN,
	LINEAR_SWITCH_7_BTN,
	LINEAR_SWITCH_8_BTN,
	LINEAR_SWITCH_9_BTN,
  BUTTONS_NUMBER
};


#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
