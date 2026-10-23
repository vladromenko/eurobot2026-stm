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
#ifndef __HL_RTU_H
#define __HL_RTU_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.hpp"
#include "stm32f4xx_ll_usart.h"
#include "usartx_communication.hpp"


#define HL_RTU_MAX_DATA_LENGTH     32


struct hl_rtu_request_s{
    uint8_t cmd;
    uint8_t data_length;
    uint8_t data[HL_RTU_MAX_DATA_LENGTH];
};

struct hl_rtu_response_s{
    uint8_t cmd;
    uint8_t data_length;
    uint8_t data[HL_RTU_MAX_DATA_LENGTH];
};

struct hl_rtu_s{
    void (*SendResponse)(USART_TypeDef*, struct hl_rtu_response_s*);
    uint8_t (*GetRequest)(USART_TypeDef*, struct hl_rtu_request_s*);
    struct usartx_protocol_s* (*GetProtocol)(void);
};

extern const struct hl_rtu_s HL_RTU;

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
