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
#ifndef __USARTX_COMMUNICATION_H
#define __USARTX_COMMUNICATION_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.hpp"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"
#include "fifo_routine.hpp" 



struct usartx_line_resources_s{
  uint8_t* tx_buffer;
  uint8_t* rx_buffer;
  uint32_t tx_buffer_size;
  uint32_t rx_buffer_size;
  struct fifo_s tx_fifo;
  struct fifo_s rx_fifo;
  uint32_t is_line_busy;
};

struct usartx_line_config_s{
  void (*RxTimeoutCallback)(struct usartx_line_resources_s* line_resources);
  void (*TxCompleteCallback)(struct usartx_line_resources_s* line_resources);
  void (*RxCompleteCallback)(struct usartx_line_resources_s* line_resources, uint32_t data_length);
  uint32_t is_line_half_duplex;
};

struct usartx_control_s{
  struct usartx_line_resources_s* (*GetLineResources)(USART_TypeDef* usart);
  void (*StartTx)(struct usartx_line_resources_s* line_resources, uint32_t data_length);
  void (*StopTx)(struct usartx_line_resources_s* line_resources);
  void (*StartRx)(struct usartx_line_resources_s* line_resources);
  void (*StopRx)(struct usartx_line_resources_s* line_resources);
  void (*SetRxTimeout)(struct usartx_line_resources_s* line_resources, uint32_t timeout_us);
  void (*ResetRxTimeout)(struct usartx_line_resources_s* line_resources);
};

struct usartx_line_init_params_s{
  struct usartx_line_resources_s* line_resources;
  struct usartx_line_config_s* line_config;
  struct usartx_control_s* usartx_control;
};

struct usartx_protocol_s{
  void (*InitLine)(struct usartx_line_init_params_s* usartx_line_init_params);
};

struct usartx_s{
  void (*BindProtocol)(USART_TypeDef* usart, struct usartx_protocol_s* protocol);
};

extern const struct usartx_s USARTx;

void USARTx_IRQ_Process(USART_TypeDef* usartx);

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
