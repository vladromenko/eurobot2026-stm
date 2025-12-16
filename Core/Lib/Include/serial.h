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
#ifndef __SERIAL_H
#define __SERIAL_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.h"
#include "stm32f4xx_ll_usart.h"
#include "usartx_communication.h"

#define SERIAL_MAX_BYTES_NUMBER    64

struct serial_s{
    void (*WriteBytes)(USART_TypeDef* usart, uint8_t* bytes, uint8_t bytes_number);
    uint32_t (*ReadBytes)(USART_TypeDef* usart, uint8_t* bytes, uint32_t timeout_ms);
    struct usartx_protocol_s* (*GetProtocol)(void);
};

extern const struct serial_s Serial;

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
