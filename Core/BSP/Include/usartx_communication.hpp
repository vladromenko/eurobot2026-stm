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

#include "std_def.hpp"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"
#include "fifo_routine.hpp" 


class USARTx_Protocol;

/**
 * One USART line: DMA buffers, FIFOs and the protocol bound to it.
 * The protocol splits tx/rx buffers into a working packet and FIFOs in InitLine().
 */
class USARTx_Line{
public:
  void StartTx(uint32_t data_length);
  void StopTx();
  void StartRx();
  void StopRx();
  void SetRxTimeout(uint32_t timeout_us);
  void ResetRxTimeout();

  uint8_t* tx_buffer;
  uint8_t* rx_buffer;
  uint32_t tx_buffer_size;
  uint32_t rx_buffer_size;
  FIFO tx_fifo;
  FIFO rx_fifo;
  uint32_t is_line_busy;
  uint32_t is_line_half_duplex;

private:
  friend class USARTx_Driver;

  static void RxTimeoutHandler(void* line);

  USARTx_Protocol* protocol;
  USART_TypeDef* usart;
  DMA_TypeDef* tx_dma;
  DMA_TypeDef* rx_dma;
  uint32_t tx_dma_stream;
  uint32_t rx_dma_stream;
  GPIO_TypeDef* de_gpio;
  uint32_t de_pin;
  uint8_t timeout_channel;
};

/*  Protocol (Dynamixel, Serial, HL_RTU, ...) driving one or more USART lines  */
class USARTx_Protocol{
public:
  /*  Called once by USARTx.BindProtocol()  */
  virtual void InitLine(USARTx_Line* line) = 0;

  /*  Called from the USART/TIM14 interrupts  */
  virtual void TxCompleteCallback(USARTx_Line* line) = 0;
  virtual void RxCompleteCallback(USARTx_Line* line, uint32_t data_length) = 0;
  virtual void RxTimeoutCallback(USARTx_Line* line) { (void)line; }
};

class USARTx_Driver{
public:
  void BindProtocol(USART_TypeDef* usart, USARTx_Protocol& protocol);
  USARTx_Line* GetLine(USART_TypeDef* usart);
  void IRQ_Process(USART_TypeDef* usart);

private:
  USARTx_Line lines[6];
};

extern USARTx_Driver USARTx;

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
