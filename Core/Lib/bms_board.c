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
#include "bms_board.h"
#include "dynamixel_master.h"
#include "byte2word.h"

#include "board_gpio_defines.h"
#include "stm32f4xx_ll_gpio.h"

#include "FreeRTOS.h"
#include "task.h"


static uint8_t Ping(USART_TypeDef* usart, uint8_t board_id)
{
  struct dynamixel_master_response_s response;
  DynamixelMaster.Ping(usart, board_id, &response);
  while(response.is_data_ready==0) taskYIELD();
  return response.bus_error;
}
    
static void Set_ID(USART_TypeDef* usart, uint8_t current_id, uint8_t new_id)
{
  DynamixelMaster.Write(usart, current_id, BMS_BOARD_ID, &new_id, 1, NULL); 
}

static uint8_t GetBatteryState(USART_TypeDef* usart, uint8_t board_id, struct bms_battery_state_s* battery_state)
{
  struct dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart, board_id, BMS_POWER_CONSUMPTION, (BMS_BATT_CELLS_NUMBER - BMS_POWER_CONSUMPTION + 1), &response);

  while(response.is_data_ready==0) taskYIELD();
  memcpy(battery_state, response.data, sizeof(struct bms_battery_state_s));

  return response.bus_error;
}

static uint8_t GetBatteryCellsVoltage(USART_TypeDef* usart, uint8_t board_id, uint16_t* cells_voltage_mv)
{
  struct dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart, board_id, BMS_BATT_CELL_1_VOLTAGE, 8, &response);

  while(response.is_data_ready==0) taskYIELD();
  memcpy(cells_voltage_mv, response.data, 8);

  return response.bus_error;
}

static uint8_t GetBatteryVoltage(USART_TypeDef* usart, uint8_t board_id, uint16_t* battery_voltage_mv)
{
  struct dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart, board_id, BMS_BATT_VOLTAGE, 2, &response);

  while(response.is_data_ready==0) taskYIELD();
  ByteTo_Word16(response.data, battery_voltage_mv);

  return response.bus_error;
}

static uint8_t GetBatteryCurrent(USART_TypeDef* usart, uint8_t board_id, uint16_t* battery_current_ma)
{
  struct dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart, board_id, BMS_BATT_CURRENT, 2, &response);

  while(response.is_data_ready==0) taskYIELD();
  ByteTo_Word16(response.data, battery_current_ma);

  return response.bus_error;
}

uint8_t GetPowerConsumption(USART_TypeDef* usart, uint8_t board_id, uint32_t* power_consumption_mw)
{
  struct dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart, board_id, BMS_POWER_CONSUMPTION, 4, &response);

  while(response.is_data_ready==0) taskYIELD();
  ByteTo_Word32(response.data, power_consumption_mw);

  return response.bus_error;
}

static uint8_t GetBattery_SOC(USART_TypeDef* usart, uint8_t board_id, uint8_t* battery_soc)
{
  struct dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart, board_id, BMS_BATT_SOC, 1, &response);

  while(response.is_data_ready==0) taskYIELD();
  *battery_soc = *response.data;

  return response.bus_error;
}

static uint8_t GetBatteryCellsNumber(USART_TypeDef* usart, uint8_t board_id, uint8_t* cells_number)
{
  struct dynamixel_master_response_s response;
  
  DynamixelMaster.Read(usart, board_id, BMS_BATT_CELLS_NUMBER, 1, &response);

  while(response.is_data_ready==0) taskYIELD();
  *cells_number = *response.data;

  return response.bus_error;
}

static uint8_t SetCalibrationValue(USART_TypeDef* usart, uint8_t board_id, uint8_t calibration_register_address, uint16_t calibration_value)
{
  struct dynamixel_master_response_s response;
  
  DynamixelMaster.Write(usart, board_id, calibration_register_address, (uint8_t*)(&calibration_value), 2, &response);

  while(response.is_data_ready==0) taskYIELD();

  return response.bus_error;
}

const struct bms_board_s BMS = {.Ping = Ping,
                                .Set_ID = Set_ID,
                                .GetBatteryState = GetBatteryState,
                                .GetBatteryCellsVoltage = GetBatteryCellsVoltage,
                                .GetBatteryVoltage = GetBatteryVoltage,
                                .GetBatteryCurrent = GetBatteryCurrent,
                                .GetBattery_SOC = GetBattery_SOC,
                                .GetBatteryCellsNumber = GetBatteryCellsNumber,
                                .SetCalibrationValue = SetCalibrationValue};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
