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
#include "h_bridge.hpp"
#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.hpp"


namespace {

/*  H_BRIDGE1: PWM 1/2, H_BRIDGE2: PWM 3/4  */
constexpr pwm_output_e H_BRIDGE_P1[] = {PWM_OUTPUT_1, PWM_OUTPUT_3};
constexpr pwm_output_e H_BRIDGE_P2[] = {PWM_OUTPUT_2, PWM_OUTPUT_4};

}

void H_BridgeDriver::SetMotorSpeed(h_bridge_e h_bridge, int32_t speed)
{
  uint32_t duty_cycle_p1, duty_cycle_p2;

  if((speed>100)||(speed<-100))
    return;

  if(speed>=0){
    duty_cycle_p1 = speed;
    duty_cycle_p2 = 100;
  }else{
    duty_cycle_p1 = 100;
    duty_cycle_p2 = -speed;
  }

  PWM.SetOutputDutyCycle(H_BRIDGE_P1[h_bridge], duty_cycle_p1);
  PWM.SetOutputDutyCycle(H_BRIDGE_P2[h_bridge], duty_cycle_p2);
}

void H_BridgeDriver::Enable(h_bridge_e h_bridge)
{
  PWM.Enable(H_BRIDGE_P1[h_bridge]);
  PWM.Enable(H_BRIDGE_P2[h_bridge]);
}

void H_BridgeDriver::Disable(h_bridge_e h_bridge)
{
  PWM.Disable(H_BRIDGE_P1[h_bridge]);
  PWM.Disable(H_BRIDGE_P2[h_bridge]);
}



constinit H_BridgeDriver H_Bridge;

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
