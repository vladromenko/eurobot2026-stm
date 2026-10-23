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

#include "std_def.hpp"

enum h_bridge_e{
  H_BRIDGE1,
  H_BRIDGE2
};

/*  DC motor H-bridges on PWM outputs 1..4; speed -100..100 %  */
class H_BridgeDriver{
public:
    void SetMotorSpeed(h_bridge_e h_bridge, int32_t speed);
    void Enable(h_bridge_e h_bridge);
    void Disable(h_bridge_e h_bridge);
};

/*  The object lives in HwInstance (hw_instance.hpp); this is a shortcut to it  */
extern H_BridgeDriver& H_Bridge;

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
