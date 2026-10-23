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



#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.hpp"
#include "robot_gpio_defines.hpp"
#include "byte2word.hpp" 
#include "maxon.hpp" 
#include "dynamixel_ax.hpp" 
#include "hl_rtu.hpp" 
#include "terminal.hpp" 




static void SendResponse(struct terminal_response_s* terminal_response)
{
  HL_RTU.SendResponse((struct hl_rtu_response_s*)terminal_response);
}

static uint8_t GetRequest(struct terminal_request_s* terminal_request)
{
  return HL_RTU.GetRequest((struct hl_rtu_request_s*)terminal_request);
}


const struct terminal_s Terminal = {.SendResponse = SendResponse,
                                    .GetRequest = GetRequest};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
