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

#include "dynamixel_mx.hpp" 
#include "dynamixel_master.hpp"
#include "byte2word.hpp"

#include "board_gpio_defines.hpp"
#include "stm32f4xx_ll_gpio.h"

#include "FreeRTOS.h"
#include "task.h"

static uint8_t Ping(USART_TypeDef* usart, uint8_t servo_id)
{
  struct dynamixel_master_response_s response;
  DynamixelMaster.Ping(usart,servo_id,&response);
  while(response.is_data_ready==0) taskYIELD();
  return response.bus_error;
}

static void Set_ID(USART_TypeDef* usart, uint8_t current_id, uint8_t new_id)
{
  DynamixelMaster.Write(usart, current_id, MX_ID, &new_id, 1, NULL); 
}

static void TorqueEnable(USART_TypeDef* usart, uint8_t servo_id)
{
  uint8_t data = 1;
  DynamixelMaster.Write(usart,servo_id,MX_TorqueEnable,&data,1,NULL); 
}

static void TorqueDisable(USART_TypeDef* usart, uint8_t servo_id)
{
  uint8_t data = 0;
  DynamixelMaster.Write(usart,servo_id,MX_TorqueEnable,&data,1,NULL); 
}

static void SetAngle(USART_TypeDef* usart, uint8_t servo_id, uint16_t angle)
{
  uint16_t temp = ((uint32_t)angle*4095)/360;
  DynamixelMaster.Write(usart,servo_id,MX_GoalPosition,(uint8_t*)&temp,2,NULL); 
}

static uint8_t GetAngle(USART_TypeDef* usart, uint8_t servo_id, uint16_t* angle)
{
  struct dynamixel_master_response_s response;
  uint32_t temp;
  
  DynamixelMaster.Read(usart,servo_id,MX_PresentPosition,2,&response);
  while(response.is_data_ready==0) taskYIELD();
    
  ByteTo_Word16(response.data,&temp);
  *angle = (temp*360)/4095;

  return response.bus_error;
}

static void SetWheelMode(USART_TypeDef* usart, uint8_t servo_id)
{
  // uint8_t data[4] = {0x00,0x00,0x00,0x00};
  // DynamixelMaster.Write(usart,servo_id,MX_CW_AngleLimit,data,4,NULL);
}

static void SetJointMode(USART_TypeDef* usart, uint8_t servo_id)
{
  // uint8_t data[4] = {0x00,0x00,0xFF,0x03};
  // DynamixelMaster.Write(usart,servo_id,MX_CW_AngleLimit,data,4,NULL); 
}

static void SetMovingSpeed(USART_TypeDef* usart, uint8_t servo_id, int16_t speed)
{
  // uint8_t data[2];

  // if(speed<0)
  //   speed = 1024 - speed;

  // Word16_ToByte(&speed,data);
  // DynamixelMaster.Write(usart,servo_id,MX_MovingSpeed,data,2,NULL); 
}

static uint8_t GetPresentLoad(USART_TypeDef* usart, uint8_t servo_id, int16_t* load)
{
  // struct dynamixel_master_response_s response;
  // int16_t temp;
  
  // DynamixelMaster.Read(usart,servo_id,MX_PresentLoad,2,&response);
  // while(response.is_data_ready==0);
    
	
  // ByteTo_Word16(response.data,&temp);

  // if(temp>1023)
  //   temp = 1024 - temp; 

  // *load = temp;

  // return response.bus_error;
  return 0;
}

static void Enable_LED(USART_TypeDef* usart, uint8_t servo_id)
{
  uint8_t data = 1;
  DynamixelMaster.Write(usart,servo_id,MX_LED,&data,1,NULL); 
}

static void Disable_LED(USART_TypeDef* usart, uint8_t servo_id)
{
  uint8_t data = 0;
  DynamixelMaster.Write(usart,servo_id,MX_LED,&data,1,NULL);
}

const struct dynamixel_mx_s Dynamixel_MX = {.Ping = Ping,
                                            .Set_ID = Set_ID,
                                            .TorqueEnable = TorqueEnable,
                                            .TorqueDisable = TorqueDisable,
                                            .SetAngle = SetAngle,
                                            .GetAngle = GetAngle,
                                            .SetWheelMode = SetWheelMode,
                                            .SetJointMode = SetJointMode,
                                            .SetMovingSpeed = SetMovingSpeed,
                                            .GetPresentLoad = GetPresentLoad,
                                            .Enable_LED = Enable_LED,
                                            .Disable_LED = Disable_LED};



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
