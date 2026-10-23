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
#ifndef __BMS_BOARD_H
#define __BMS_BOARD_H

#include "stdint.h"
#include "dynamixel_master.hpp"

/*	BMS Board FLASH MEMORY Area registers define ----------------------------------------------------------- */
#define BMS_BOARD_ID              	              0x01  /*  1 byte  */

/*	BMS Board SRAM MEMORY Area registers define ------------------------------------------------------------ */
#define BMS_POWER_CONSUMPTION                     0x04  /*  4 bytes [mw]  */
#define BMS_BATT_CELL_1_VOLTAGE                   0x08  /*  2 bytes [mv]  */
#define BMS_BATT_CELL_2_VOLTAGE                   0x0A  /*  2 bytes [mv]  */
#define BMS_BATT_CELL_3_VOLTAGE                   0x0C  /*  2 bytes [mv]  */
#define BMS_BATT_CELL_4_VOLTAGE                   0x0E  /*  2 bytes [mv]  */
#define BMS_BATT_VOLTAGE                          0x10  /*  2 bytes [mv]  */
#define BMS_BATT_CURRENT                          0x12  /*  2 bytes [ma]  */
#define BMS_BATT_SOC                              0x14  /*  1 byte [%]  */
#define BMS_BATT_CELLS_NUMBER                     0x15  /*  1 byte  */
#define BMS_CALIBRATION_BATT_CELL_1_VOLTAGE       0x16  /*  2 bytes [mv]  */
#define BMS_CALIBRATION_BATT_CELL_2_VOLTAGE       0x18  /*  2 bytes [mv]  */
#define BMS_CALIBRATION_BATT_CELL_3_VOLTAGE       0x1A  /*  2 bytes [mv]  */
#define BMS_CALIBRATION_BATT_CELL_4_VOLTAGE       0x1C  /*  2 bytes [mv]  */
#define BMS_CALIBRATION_BATT_VOLTAGE              0x1E  /*  2 bytes [mv]  */
#define BMS_CALIBRATION_BATT_CURRENT              0x20  /*  2 bytes [ma]  */

struct bms_battery_state_s{
  uint32_t power_consumption_mw;  /* [mw] */
  uint16_t cells_voltage_mv[4];   /* [mv] */
  uint16_t batt_voltage_mv;       /* [mv] */
  uint16_t batt_current_ma;       /* [ma] */
  uint8_t batt_soc;               /* [%] */
  uint8_t cells_number;
};

/*  Battery management board, Dynamixel protocol  */
class BMS_Driver{
public:
    constexpr explicit BMS_Driver(DynamixelMasterDriver& bus) : bus(bus) {}

    uint8_t Ping(uint8_t board_id);
    void Set_ID(uint8_t current_id, uint8_t new_id);
    uint8_t GetBatteryState(uint8_t board_id, bms_battery_state_s* battery_state);
    uint8_t GetBatteryCellsVoltage(uint8_t board_id, uint16_t* cells_voltage_mv);
    uint8_t GetBatteryVoltage(uint8_t board_id, uint16_t* battery_voltage_mv);
    uint8_t GetBatteryCurrent(uint8_t board_id, uint16_t* battery_current_ma);
    uint8_t GetPowerConsumption(uint8_t board_id, uint32_t* power_consumption_mw);
    uint8_t GetBattery_SOC(uint8_t board_id, uint8_t* battery_soc);
    uint8_t GetBatteryCellsNumber(uint8_t board_id, uint8_t* cells_number);
    uint8_t SetCalibrationValue(uint8_t board_id, uint8_t calibration_register_address, uint16_t calibration_value);

private:
    DynamixelMasterDriver& bus;
};

/*  The object lives in HwInstance (hw_instance.hpp); this is a shortcut to it  */
extern BMS_Driver& BMS;

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
