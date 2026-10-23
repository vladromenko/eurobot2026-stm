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
#include "bms_board.hpp"
#include "dynamixel_master.hpp"
#include "byte2word.hpp"

#include "board_gpio_defines.hpp"
#include "stm32f4xx_ll_gpio.h"

#include "FreeRTOS.h"
#include "task.h"


uint8_t BMS_Driver::Ping(USART_TypeDef* usart, uint8_t board_id)
{
  dynamixel_master_response_s response;
  DynamixelMaster.Ping(usart, board_id, &response);
  while(response.is_data_ready==0) taskYIELD();
  return response.bus_error;
}
    
void BMS_Driver::Set_ID(USART_TypeDef* usart, uint8_t current_id, uint8_t new_id)
{
  DynamixelMaster.Write(usart, current_id, BMS_BOARD_ID, &new_id, 1, nullptr); 
}

uint8_t BMS_Driver::GetBatteryState(USART_TypeDef* usart, uint8_t board_id, bms_battery_state_s* battery_state)
{
  dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart, board_id, BMS_POWER_CONSUMPTION, (BMS_BATT_CELLS_NUMBER - BMS_POWER_CONSUMPTION + 1), &response);

  while(response.is_data_ready==0) taskYIELD();
  memcpy(battery_state, response.data, sizeof(bms_battery_state_s));

  return response.bus_error;
}

uint8_t BMS_Driver::GetBatteryCellsVoltage(USART_TypeDef* usart, uint8_t board_id, uint16_t* cells_voltage_mv)
{
  dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart, board_id, BMS_BATT_CELL_1_VOLTAGE, 8, &response);

  while(response.is_data_ready==0) taskYIELD();
  memcpy(cells_voltage_mv, response.data, 8);

  return response.bus_error;
}

uint8_t BMS_Driver::GetBatteryVoltage(USART_TypeDef* usart, uint8_t board_id, uint16_t* battery_voltage_mv)
{
  dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart, board_id, BMS_BATT_VOLTAGE, 2, &response);

  while(response.is_data_ready==0) taskYIELD();
  ByteTo_Word16(response.data, battery_voltage_mv);

  return response.bus_error;
}

uint8_t BMS_Driver::GetBatteryCurrent(USART_TypeDef* usart, uint8_t board_id, uint16_t* battery_current_ma)
{
  dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart, board_id, BMS_BATT_CURRENT, 2, &response);

  while(response.is_data_ready==0) taskYIELD();
  ByteTo_Word16(response.data, battery_current_ma);

  return response.bus_error;
}

uint8_t BMS_Driver::GetPowerConsumption(USART_TypeDef* usart, uint8_t board_id, uint32_t* power_consumption_mw)
{
  dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart, board_id, BMS_POWER_CONSUMPTION, 4, &response);

  while(response.is_data_ready==0) taskYIELD();
  ByteTo_Word32(response.data, power_consumption_mw);

  return response.bus_error;
}

uint8_t BMS_Driver::GetBattery_SOC(USART_TypeDef* usart, uint8_t board_id, uint8_t* battery_soc)
{
  dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart, board_id, BMS_BATT_SOC, 1, &response);

  while(response.is_data_ready==0) taskYIELD();
  *battery_soc = *response.data;

  return response.bus_error;
}

uint8_t BMS_Driver::GetBatteryCellsNumber(USART_TypeDef* usart, uint8_t board_id, uint8_t* cells_number)
{
  dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart, board_id, BMS_BATT_CELLS_NUMBER, 1, &response);

  while(response.is_data_ready==0) taskYIELD();
  *cells_number = *response.data;

  return response.bus_error;
}

uint8_t BMS_Driver::SetCalibrationValue(USART_TypeDef* usart, uint8_t board_id, uint8_t calibration_register_address, uint16_t calibration_value)
{
  dynamixel_master_response_s response;
  
  DynamixelMaster.Write(usart, board_id, calibration_register_address, (uint8_t*)(&calibration_value), 2, &response);

  while(response.is_data_ready==0) taskYIELD();

  return response.bus_error;
}

constinit BMS_Driver BMS;

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
