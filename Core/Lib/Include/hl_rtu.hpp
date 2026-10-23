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

/*  Request/response protocol of the high level (Raspberry Pi) link: 0xFF 0xFF | length | cmd | data | crc  */
class HL_RTU_Driver : public USARTx_Protocol{
public:
    void SendResponse(USART_TypeDef* usart, hl_rtu_response_s* response);
    /*  Returns 1 if a request was taken from the queue  */
    uint8_t GetRequest(USART_TypeDef* usart, hl_rtu_request_s* request);

private:
    void InitLine(USARTx_Line* line) override;
    void TxCompleteCallback(USARTx_Line* line) override;
    void RxCompleteCallback(USARTx_Line* line, uint32_t data_length) override;
};

extern HL_RTU_Driver HL_RTU;

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
