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
#include "terminal.hpp" 
#include "hl_rtu.hpp" 

/*  Terminal packets are sent as HL_RTU PDUs as is  */
static_assert(sizeof(terminal_request_s) == sizeof(hl_rtu_request_s));
static_assert(sizeof(terminal_response_s) == sizeof(hl_rtu_response_s));

constinit TerminalModule Terminal;


void TerminalModule::SendResponse(terminal_response_s* terminal_response)
{
  HL_RTU.SendResponse(reinterpret_cast<hl_rtu_response_s*>(terminal_response));
}

uint8_t TerminalModule::GetRequest(terminal_request_s* terminal_request)
{
  return HL_RTU.GetRequest(reinterpret_cast<hl_rtu_request_s*>(terminal_request));
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
