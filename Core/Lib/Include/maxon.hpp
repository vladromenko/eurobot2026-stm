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
#ifndef __MAXON_H
#define __MAXON_H

#include "stdint.h"
#include "dynamixel_master.hpp"

/*	MAXON FLASH MEMORY Area registers define ----------------------------------------------------------- */
#define MAXON_BOARD_ID            0x01    /*  Maxon board id	              */
#define MAXON_RangeTPM            0x04    /*  Ticks Per Second (TPM) range  */

/*	MAXON SRAM MEMORY Area registers define ------------------------------------------------------------ */
#define MAXON_TPM                 0x10    /*  Ticks Per Second (TPM)        */
#define MAXON_TotalTicks          0x14    /*  Total number of ticks         */
#define MAXON_Enable              0x18    /*  Enable/disable state          */

/*  Maxon motor controller board, Dynamixel protocol  */
class MaxonDriver{
public:
    constexpr explicit MaxonDriver(DynamixelMasterDriver& bus) : bus(bus) {}

    uint8_t Ping(uint8_t maxon_id);
    void Set_ID(uint8_t current_id, uint8_t new_id);
    void SetRange_TPM(uint8_t maxon_id, uint32_t tpm_bottom, uint32_t tpm_top);
    uint8_t GetRange_TPM(uint8_t maxon_id, uint32_t* tpm_bottom, uint32_t* tpm_top);
    void Set_TPM(uint8_t maxon_id, int32_t tpm);
    uint8_t Get_TPM(uint8_t maxon_id, int32_t* tpm);
    void SetTotalTicks(uint8_t maxon_id, int32_t ticks);
    uint8_t GetTotalTicks(uint8_t maxon_id, int32_t* ticks);
    void Enable(uint8_t maxon_id);
    void Disable(uint8_t maxon_id);

private:
    DynamixelMasterDriver& bus;
};

/*  The object lives in HwInstance (hw_instance.hpp); this is a shortcut to it  */
extern MaxonDriver& Maxon;

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
