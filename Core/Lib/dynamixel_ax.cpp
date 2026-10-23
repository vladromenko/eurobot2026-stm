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

#include "dynamixel_ax.hpp" 
#include "dynamixel_master.hpp"
#include "byte2word.hpp"

#include "board_gpio_defines.hpp"
#include "stm32f4xx_ll_gpio.h"

#include "FreeRTOS.h"
#include "task.h"

uint8_t Dynamixel_AX_Driver::Ping(uint8_t servo_id)
{
  dynamixel_master_response_s response;
  bus.Ping(servo_id,&response);
  while(response.is_data_ready==0) taskYIELD();
  return response.bus_error;
}

void Dynamixel_AX_Driver::Set_ID(uint8_t current_id, uint8_t new_id)
{
  bus.Write(current_id, AX_ID, &new_id, 1, nullptr); 
}

void Dynamixel_AX_Driver::TorqueEnable(uint8_t servo_id)
{
  uint8_t data = 1;
  bus.Write(servo_id,AX_TorqueEnable,&data,1,nullptr); 
}

void Dynamixel_AX_Driver::TorqueDisable(uint8_t servo_id)
{
  uint8_t data = 0;
  bus.Write(servo_id,AX_TorqueEnable,&data,1,nullptr); 
}

void Dynamixel_AX_Driver::SetAngle(uint8_t servo_id, uint16_t angle)
{
  uint16_t temp = ((uint32_t)angle*1023)/300;
  bus.Write(servo_id,AX_GoalPosition,(uint8_t*)&temp,2,nullptr); 
}

uint8_t Dynamixel_AX_Driver::GetAngle(uint8_t servo_id, uint16_t* angle)
{
  dynamixel_master_response_s response = {};
  uint32_t temp = 0;
  
  bus.Read(servo_id,AX_PresentPosition,2,&response);
  while(response.is_data_ready==0) taskYIELD();
    
  ByteTo_Word16(response.data,&temp);
  *angle = (uint16_t)((temp*300)/1023);

  return response.bus_error;
}

void Dynamixel_AX_Driver::SetWheelMode(uint8_t servo_id)
{
  uint8_t data[4] = {0x00,0x00,0x00,0x00};
  bus.Write(servo_id,AX_CW_AngleLimit,data,4,nullptr);
}

void Dynamixel_AX_Driver::SetJointMode(uint8_t servo_id)
{
  uint8_t data[4] = {0x00,0x00,0xFF,0x03};
  bus.Write(servo_id,AX_CW_AngleLimit,data,4,nullptr); 
}

void Dynamixel_AX_Driver::SetMovingSpeed(uint8_t servo_id, int16_t speed)
{
  if(speed<0) 
    speed = 1024 - speed;
  bus.Write(servo_id,AX_MovingSpeed,(uint8_t*)&speed,2,nullptr); 
}

uint8_t Dynamixel_AX_Driver::GetMovingSpeed(uint8_t servo_id, int16_t* speed)
{
	dynamixel_master_response_s response;
  int16_t temp;
  
  bus.Read(servo_id,AX_MovingSpeed,2,&response);
  while(response.is_data_ready==0) taskYIELD();
    
  ByteTo_Word16(response.data,&temp);

  if(temp>1023)
    temp = 1024 - temp;

  *speed = temp;

  return response.bus_error;
}

uint8_t Dynamixel_AX_Driver::GetPresentLoad(uint8_t servo_id, int16_t* load)
{
  dynamixel_master_response_s response;
  int16_t temp;
  
  bus.Read(servo_id,AX_PresentLoad,2,&response);
  while(response.is_data_ready==0) taskYIELD();
    
  ByteTo_Word16(response.data,&temp);

  if(temp>1023)
    temp = 1024 - temp;

  *load = temp;

  return response.bus_error;
}

void Dynamixel_AX_Driver::Enable_LED(uint8_t servo_id)
{
  uint8_t data = 1;
  bus.Write(servo_id,AX_LED,&data,1,nullptr); 
}

void Dynamixel_AX_Driver::Disable_LED(uint8_t servo_id)
{
  uint8_t data = 0;
  bus.Write(servo_id,AX_LED,&data,1,nullptr);
}

constinit Dynamixel_AX_Driver Dynamixel_AX{DynamixelBus3};



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
