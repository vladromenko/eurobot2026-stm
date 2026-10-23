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

#include "std_def.hpp"
#include "stm32f4xx_ll_usart.h"
#include "usartx_communication.hpp"

#define SERIAL_MAX_BYTES_NUMBER    64

/*  Raw full-duplex byte stream; every IDLE-terminated chunk is one rx FIFO element  */
class SerialDriver : public USARTx_Protocol{
public:
    void WriteBytes(uint8_t* bytes, uint8_t bytes_number);
    /*  timeout_ms == 0: do not wait. Returns the number of bytes read  */
    uint32_t ReadBytes(uint8_t* bytes, uint32_t timeout_ms);

private:
    void InitLine() override;
    void TxCompleteCallback() override;
    void RxCompleteCallback(uint32_t data_length) override;
    void RxTimeoutCallback() override;

    volatile uint8_t is_timeout_expired = 0;
};

/*  Debug terminal, bound to UART5 in main()  */
extern SerialDriver Serial;

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
