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

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"
#include "stm32f4xx_ll_usart.h"

	 
/*	MAXON FLASH MEMORY Area registers define ----------------------------------------------------------- */
#define MAXON_BOARD_ID            0x01    /*  Maxon board id	              */  
#define MAXON_RangeTPM            0x04    /*  Ticks Per Second (TPM) range  */

/*	MAXON SRAM MEMORY Area registers define ------------------------------------------------------------ */
#define MAXON_TPM                 0x10    /*  Ticks Per Second (TPM)        */
#define MAXON_TotalTicks          0x14    /*  Total number of ticks         */
#define MAXON_Enable              0x18    /*  Enable/disable state          */



struct maxon_s{
    uint8_t (*Ping)(USART_TypeDef* usart, uint8_t maxon_id);
    void (*Set_ID)(USART_TypeDef* usart, uint8_t current_id, uint8_t new_id);
    void (*SetRange_TPM)(USART_TypeDef* usart, uint8_t maxon_id, uint32_t tpm_bottom, uint32_t tpm_top);
    uint8_t (*GetRange_TPM)(USART_TypeDef* usart, uint8_t maxon_id, uint32_t* tpm_bottom, uint32_t* tpm_top);
    void (*Set_TPM)(USART_TypeDef* usart, uint8_t maxon_id, int32_t tpm);
    uint8_t (*Get_TPM)(USART_TypeDef* usart, uint8_t maxon_id, int32_t* tpm);
    void (*SetTotalTicks)(USART_TypeDef* usart, uint8_t maxon_id, int32_t ticks);
    uint8_t (*GetTotalTicks)(USART_TypeDef* usart, uint8_t maxon_id, int32_t* ticks);
    void (*Enable)(USART_TypeDef* usart, uint8_t maxon_id);
    void (*Disable)(USART_TypeDef* usart, uint8_t maxon_id);
};

extern const struct maxon_s Maxon;

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
