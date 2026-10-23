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
#include "usartx_communication.hpp"
#include "board_gpio_defines.hpp"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "usartx_timeout.hpp" 


#define USART1_USED
// #define USART2_USED
#define USART3_USED
#define UART4_USED
#define UART5_USED
#define USART6_USED


/*	USART1 TX/RX FIFO DEFINITION	*/
#if defined (USART1_USED)

  #define USART1_TX_BUFFER_SIZE     256
  #define USART1_RX_BUFFER_SIZE     256

  static uint8_t usart1_tx_buffer[USART1_TX_BUFFER_SIZE];
  static uint8_t usart1_rx_buffer[USART1_RX_BUFFER_SIZE];

#endif


/*	USART2 TX/RX FIFO DEFINITION	*/
#if defined (USART2_USED)

  #define USART2_TX_BUFFER_SIZE     1
  #define USART2_RX_BUFFER_SIZE     1

  static uint8_t usart2_tx_buffer[USART2_TX_BUFFER_SIZE];
  static uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];

#endif


/*	USART3 TX/RX FIFO DEFINITION	*/
#if defined (USART3_USED)

  #define USART3_TX_BUFFER_SIZE     256
  #define USART3_RX_BUFFER_SIZE     128

  static uint8_t usart3_tx_buffer[USART3_TX_BUFFER_SIZE];
  static uint8_t usart3_rx_buffer[USART3_RX_BUFFER_SIZE];

#endif


/*	UART4 TX/RX FIFO DEFINITION	*/
#if defined (UART4_USED)

  #define UART4_TX_BUFFER_SIZE     512
  #define UART4_RX_BUFFER_SIZE     256

  static uint8_t uart4_tx_buffer[UART4_TX_BUFFER_SIZE];
  static uint8_t uart4_rx_buffer[UART4_RX_BUFFER_SIZE];

#endif


/*	UART5 TX/RX FIFO DEFINITION	*/
#if defined (UART5_USED)

  #define UART5_TX_BUFFER_SIZE     256
  #define UART5_RX_BUFFER_SIZE     256

  static uint8_t uart5_tx_buffer[UART5_TX_BUFFER_SIZE];
  static uint8_t uart5_rx_buffer[UART5_RX_BUFFER_SIZE];

#endif


/*	USART6 TX/RX FIFO DEFINITION	*/
#if defined (USART6_USED)

  #define USART6_TX_BUFFER_SIZE     512
  #define USART6_RX_BUFFER_SIZE     256

  static uint8_t usart6_tx_buffer[USART6_TX_BUFFER_SIZE];
  static uint8_t usart6_rx_buffer[USART6_RX_BUFFER_SIZE];

#endif



namespace {

void (* const LL_DMA_ClearFlag_TCx[8])(DMA_TypeDef* dma) = {LL_DMA_ClearFlag_TC0,
                                                            LL_DMA_ClearFlag_TC1,
                                                            LL_DMA_ClearFlag_TC2,
                                                            LL_DMA_ClearFlag_TC3,
                                                            LL_DMA_ClearFlag_TC4,
                                                            LL_DMA_ClearFlag_TC5,
                                                            LL_DMA_ClearFlag_TC6,
                                                            LL_DMA_ClearFlag_TC7};

inline void LL_DMA_ClearFlag_TC(DMA_TypeDef* dma, uint32_t dma_stream)
{
  LL_DMA_ClearFlag_TCx[dma_stream](dma);
}

inline void StartDMA(DMA_TypeDef* dma, uint32_t dma_stream, uint8_t* data, uint32_t length)
{
  LL_DMA_ClearFlag_TC(dma, dma_stream);
  LL_DMA_SetMemoryAddress(dma, dma_stream, (uint32_t)data);
	LL_DMA_SetDataLength(dma, dma_stream, length);
  LL_DMA_EnableStream(dma, dma_stream);
}

inline void StopDMA(DMA_TypeDef* dma, uint32_t dma_stream)
{
  LL_DMA_DisableStream(dma, dma_stream);
  LL_DMA_ClearFlag_TC(dma, dma_stream);
}

}


void USARTx_Line::StartTx(uint32_t data_length)
{
  /**  __disable_irq() is necessary because of connection issues
    * if some inetrrupt occurs between the LL_USART_EnableDirectionTx()
    * and something other before __enable_irq(). Did not figure out
    * what exectly it is, do not have time now. Maybe will do it later.
    * Anywhere it works fine now. I did the same for other functions here.
  */
  __disable_irq();
  if(is_line_half_duplex)
    LL_GPIO_SetOutputPin(de_gpio, de_pin);

  LL_USART_EnableDirectionTx(usart);

  LL_USART_ClearFlag_TC(usart);
  StartDMA(tx_dma, tx_dma_stream, tx_buffer, data_length);
  /** 
    *  You have to wait until dma feeds data to the TX buffer
    *  otherwise TC flag may rise immedietely after EnableIT_TC
    */
  while(LL_USART_IsActiveFlag_TXE(usart));
  LL_USART_EnableIT_TC(usart);
  __enable_irq();
}

void USARTx_Line::StopTx()
{
  __disable_irq();
  LL_USART_DisableDirectionTx(usart);

  if(is_line_half_duplex)
    LL_GPIO_ResetOutputPin(de_gpio, de_pin);

  StopDMA(tx_dma, tx_dma_stream);
  LL_USART_DisableIT_TC(usart);
  __enable_irq();
}

void USARTx_Line::StartRx()
{
  __disable_irq();
  if(is_line_half_duplex)
    LL_GPIO_ResetOutputPin(de_gpio, de_pin);
  
  LL_USART_EnableDirectionRx(usart);

  LL_USART_ClearFlag_ORE(usart);
  LL_USART_ClearFlag_RXNE(usart);
  LL_USART_ClearFlag_IDLE(usart);
  
  StartDMA(rx_dma, rx_dma_stream, rx_buffer, rx_buffer_size);
  LL_USART_EnableIT_IDLE(usart);
  __enable_irq();
}

void USARTx_Line::StopRx()
{
  __disable_irq();
  LL_USART_DisableDirectionRx(usart);

  StopDMA(rx_dma, rx_dma_stream);
  LL_USART_DisableIT_IDLE(usart);
  __enable_irq();
}

void USARTx_Line::SetRxTimeout(uint32_t timeout_us)
{
  USARTx_Timeout.SetTimeout(timeout_channel, timeout_us, RxTimeoutHandler, this);
}

void USARTx_Line::ResetRxTimeout()
{
  USARTx_Timeout.ResetTimeout(timeout_channel);
}

void USARTx_Line::RxTimeoutHandler(void* line)
{
  static_cast<USARTx_Line*>(line)->protocol->RxTimeoutCallback();
}


USARTx_Line* USARTx_Driver::GetLine(USART_TypeDef* usart)
{
  switch((uint32_t)usart)
	{
		case USART1_BASE: return &lines[0];
		case USART2_BASE: return &lines[1];
		case USART3_BASE: return &lines[2];
		case UART4_BASE: return &lines[3];
		case UART5_BASE: return &lines[4];
		case USART6_BASE: return &lines[5];
    default: return nullptr;
	}
}

void USARTx_Driver::IRQ_Process(USART_TypeDef* usart)
{
  USARTx_Line* line = GetLine(usart);

  /*	LL_USART_IsActiveFlag_TC	*/
	if(LL_USART_IsActiveFlag_TC(usart))
	{
		LL_USART_ClearFlag_TC(usart);

		if(LL_USART_IsEnabledIT_TC(usart)){
      line->protocol->TxCompleteCallback();
    }
	}

	/*	LL_USART_IsActiveFlag_IDLE	*/
	if(LL_USART_IsActiveFlag_IDLE(usart)!=0)
	{
		LL_USART_ClearFlag_IDLE(usart);

		if(LL_USART_IsEnabledIT_IDLE(usart)){
      uint32_t data_length = line->rx_buffer_size - LL_DMA_GetDataLength(line->rx_dma, line->rx_dma_stream);
      line->protocol->RxCompleteCallback(data_length);
		}
	}
}

void USARTx_Driver::BindProtocol(USART_TypeDef* usart, USARTx_Protocol& protocol)
{
  USARTx_Line* line = GetLine(usart);

  switch ((uint32_t)usart)
  {
    #if defined (USART1_USED)
      case USART1_BASE:
        line->usart = USART1;
        line->tx_dma = DMA2;
        line->tx_dma_stream = LL_DMA_STREAM_7;
        line->rx_dma = DMA2;
        line->rx_dma_stream = LL_DMA_STREAM_5;
        line->tx_buffer_size = USART1_TX_BUFFER_SIZE;
        line->rx_buffer_size = USART1_RX_BUFFER_SIZE;
        line->tx_buffer = usart1_tx_buffer;
        line->rx_buffer = usart1_rx_buffer;
        line->timeout_channel = 0;
      break;
    #endif

    #if defined (USART2_USED)

    #endif

    #if defined (USART3_USED)
      case USART3_BASE:
        line->usart = USART3;
        line->tx_dma = DMA1;
        line->tx_dma_stream = LL_DMA_STREAM_3;
        line->rx_dma = DMA1;
        line->rx_dma_stream = LL_DMA_STREAM_1;
        line->tx_buffer_size = USART3_TX_BUFFER_SIZE;
        line->rx_buffer_size = USART3_RX_BUFFER_SIZE;
        line->tx_buffer = usart3_tx_buffer;
        line->rx_buffer = usart3_rx_buffer;
        line->timeout_channel = 2;
        line->de_gpio = GPIOD;
        line->de_pin = UART3DE_PD9;
        break;
    #endif

    #if defined (UART4_USED)
      case UART4_BASE:
        line->usart = UART4;
        line->tx_dma = DMA1;
        line->tx_dma_stream = LL_DMA_STREAM_4;
        line->rx_dma = DMA1;
        line->rx_dma_stream = LL_DMA_STREAM_2;
        line->tx_buffer_size = UART4_TX_BUFFER_SIZE;
        line->rx_buffer_size = UART4_RX_BUFFER_SIZE;
        line->tx_buffer = uart4_tx_buffer;
        line->rx_buffer = uart4_rx_buffer;
        line->timeout_channel = 3;
        line->de_gpio = GPIOA;
        line->de_pin = UART4DE_PA15;
        break;
    #endif

    #if defined (UART5_USED)
      case UART5_BASE:
        line->usart = UART5;
        line->tx_dma = DMA1;
        line->tx_dma_stream = LL_DMA_STREAM_7;
        line->rx_dma = DMA1;
        line->rx_dma_stream = LL_DMA_STREAM_0;
        line->tx_buffer_size = UART5_TX_BUFFER_SIZE;
        line->rx_buffer_size = UART5_RX_BUFFER_SIZE;
        line->tx_buffer = uart5_tx_buffer;
        line->rx_buffer = uart5_rx_buffer;
        line->timeout_channel = 4;
        break;
    #endif

    #if defined (USART6_USED)
      case USART6_BASE:
        line->usart = USART6;
        line->tx_dma = DMA2;
        line->tx_dma_stream = LL_DMA_STREAM_6;
        line->rx_dma = DMA2;
        line->rx_dma_stream = LL_DMA_STREAM_2;
        line->tx_buffer_size = USART6_TX_BUFFER_SIZE;
        line->rx_buffer_size = USART6_RX_BUFFER_SIZE;
        line->tx_buffer = usart6_tx_buffer;
        line->rx_buffer = usart6_rx_buffer;
        line->timeout_channel = 5;
        line->de_gpio = GPIOC;
        line->de_pin = UART6DE_PC7;
        break;
    #endif

    default:

      break;
  }

  line->protocol = &protocol;
  protocol.line = line;
  protocol.InitLine();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
