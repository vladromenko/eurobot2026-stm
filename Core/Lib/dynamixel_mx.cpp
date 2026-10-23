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

uint8_t Dynamixel_MX_Driver::Ping(uint8_t servo_id)
{
  dynamixel_master_response_s response;
  bus.Ping(servo_id,&response);
  while(response.is_data_ready==0) taskYIELD();
  return response.bus_error;
}

void Dynamixel_MX_Driver::Set_ID(uint8_t current_id, uint8_t new_id)
{
  bus.Write(current_id, MX_ID, &new_id, 1, nullptr);
}

void Dynamixel_MX_Driver::TorqueEnable(uint8_t servo_id)
{
  uint8_t data = 1;
  bus.Write(servo_id,MX_TorqueEnable,&data,1,nullptr);
}

void Dynamixel_MX_Driver::TorqueDisable(uint8_t servo_id)
{
  uint8_t data = 0;
  bus.Write(servo_id,MX_TorqueEnable,&data,1,nullptr);
}

void Dynamixel_MX_Driver::SetAngle(uint8_t servo_id, uint16_t angle)
{
  uint16_t temp = ((uint32_t)angle*4095)/360;
  bus.Write(servo_id,MX_GoalPosition,(uint8_t*)&temp,2,nullptr);
}

uint8_t Dynamixel_MX_Driver::GetAngle(uint8_t servo_id, uint16_t* angle)
{
  dynamixel_master_response_s response;
  uint32_t temp = 0;

  bus.Read(servo_id,MX_PresentPosition,2,&response);
  while(response.is_data_ready==0) taskYIELD();

  ByteTo_Word16(response.data,&temp);
  *angle = (temp*360)/4095;

  return response.bus_error;
}

void Dynamixel_MX_Driver::SetWheelMode(uint8_t /*servo_id*/)
{
  // uint8_t data[4] = {0x00,0x00,0x00,0x00};
  // bus.Write(servo_id,MX_CW_AngleLimit,data,4,nullptr);
}

void Dynamixel_MX_Driver::SetJointMode(uint8_t /*servo_id*/)
{
  // uint8_t data[4] = {0x00,0x00,0xFF,0x03};
  // bus.Write(servo_id,MX_CW_AngleLimit,data,4,nullptr);
}

void Dynamixel_MX_Driver::SetMovingSpeed(uint8_t /*servo_id*/, int16_t /*speed*/)
{
  // uint8_t data[2];

  // if(speed<0)
  //   speed = 1024 - speed;

  // Word16_ToByte(&speed,data);
  // bus.Write(servo_id,MX_MovingSpeed,data,2,nullptr);
}

uint8_t Dynamixel_MX_Driver::GetPresentLoad(uint8_t /*servo_id*/, int16_t* /*load*/)
{
  // dynamixel_master_response_s response;
  // int16_t temp;

  // bus.Read(servo_id,MX_PresentLoad,2,&response);
  // while(response.is_data_ready==0);

  // ByteTo_Word16(response.data,&temp);

  // if(temp>1023)
  //   temp = 1024 - temp;

  // *load = temp;

  // return response.bus_error;
  return 0;
}

void Dynamixel_MX_Driver::Enable_LED(uint8_t servo_id)
{
  uint8_t data = 1;
  bus.Write(servo_id,MX_LED,&data,1,nullptr);
}

void Dynamixel_MX_Driver::Disable_LED(uint8_t servo_id)
{
  uint8_t data = 0;
  bus.Write(servo_id,MX_LED,&data,1,nullptr);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
