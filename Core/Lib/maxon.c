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
#include "maxon.h"
#include "dynamixel_master.h"
#include "byte2word.h"

#include "board_gpio_defines.h"
#include "stm32f4xx_ll_gpio.h"

#include "FreeRTOS.h"
#include "task.h"


static uint8_t Ping(USART_TypeDef* usart, uint8_t maxon_id)
{
  struct dynamixel_master_response_s response;
  DynamixelMaster.Ping(usart,maxon_id, &response);
  while(response.is_data_ready==0) taskYIELD();
  return response.bus_error;
}
    
static void Set_ID(USART_TypeDef* usart, uint8_t current_id, uint8_t new_id)
{
  DynamixelMaster.Write(usart, current_id, MAXON_BOARD_ID, &new_id, 1, NULL); 
}
    
static void SetRange_TPM(USART_TypeDef* usart, uint8_t maxon_id, uint32_t tpm_bottom, uint32_t tpm_top)
{
  uint8_t data[8];
  Word32_ToByte(&tpm_bottom,data);
  Word32_ToByte(&tpm_top,data+4);
  DynamixelMaster.Write(usart,maxon_id,MAXON_RangeTPM,data,8,NULL);
}

static uint8_t GetRange_TPM(USART_TypeDef* usart, uint8_t maxon_id, uint32_t* tpm_bottom, uint32_t* tpm_top)
{
  struct dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart,maxon_id,MAXON_RangeTPM,8,&response);

  while(response.is_data_ready==0) taskYIELD();

  if(response.bus_error || response.device_error)
    return response.bus_error | response.device_error;

  ByteTo_Word32(response.data,tpm_bottom);
  ByteTo_Word32(response.data+4,tpm_top);

  return 0;
}

static void Set_TPM(USART_TypeDef* usart, uint8_t maxon_id, int32_t tpm)
{
  uint8_t data[4];
  Word32_ToByte(&tpm,data);
  DynamixelMaster.Write(usart,maxon_id,MAXON_TPM,data,4,NULL); 
}

static uint8_t Get_TPM(USART_TypeDef* usart, uint8_t maxon_id, int32_t* tpm)
{
  struct dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart,maxon_id,MAXON_TPM,4,&response);
  while(response.is_data_ready==0) taskYIELD();

  if(response.bus_error || response.device_error)
    return response.bus_error | response.device_error;

  ByteTo_Word32(response.data,tpm);

  return 0;
}

static void SetTotalTicks(USART_TypeDef* usart, uint8_t maxon_id, int32_t ticks)
{
  uint8_t data[4];
  Word32_ToByte(&ticks,data);
  DynamixelMaster.Write(usart,maxon_id,MAXON_TotalTicks,data,4,NULL); 
}

static uint8_t GetTotalTicks(USART_TypeDef* usart, uint8_t maxon_id, int32_t* ticks)
{
  struct dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart,maxon_id,MAXON_TotalTicks,4,&response);
  while(response.is_data_ready==0) taskYIELD();

  if(response.bus_error || response.device_error)
    return response.bus_error | response.device_error;

  ByteTo_Word32(response.data,ticks);

  return 0;
}

static void Enable(USART_TypeDef* usart, uint8_t maxon_id)
{
  uint8_t data = 1;
  DynamixelMaster.Write(usart,maxon_id,MAXON_Enable,&data,1,NULL);
}

static void Disable(USART_TypeDef* usart, uint8_t maxon_id)
{
  uint8_t data = 0;
  DynamixelMaster.Write(usart,maxon_id,MAXON_Enable,&data,1,NULL);
}

const struct maxon_s Maxon = {.Ping = Ping,
                              .Set_ID = Set_ID,
                              .SetRange_TPM = SetRange_TPM,
                              .GetRange_TPM = GetRange_TPM,
                              .Set_TPM = Set_TPM,
                              .Get_TPM = Get_TPM,
                              .SetTotalTicks = SetTotalTicks,
                              .GetTotalTicks = GetTotalTicks,
                              .Enable = Enable,
                              .Disable = Disable};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
