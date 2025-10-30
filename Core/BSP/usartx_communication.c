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
#include "usartx_communication.h"
#include "board_gpio_defines.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "usartx_timeout.h" 


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

  #define UART5_TX_BUFFER_SIZE     128
  #define UART5_RX_BUFFER_SIZE     128

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

struct usartx_line_s{
  struct usartx_line_resources_s resources;
  struct usartx_line_config_s config;
  USART_TypeDef* usart;
  DMA_TypeDef* tx_dma;
  DMA_TypeDef* rx_dma;
  uint32_t tx_dma_stream;
	uint32_t rx_dma_stream;
  GPIO_TypeDef* de_gpio;
	uint32_t de_pin;
  uint8_t timeout_channel;
};

static struct usartx_line_s lines[6];

static struct usartx_line_s* GetLine(USART_TypeDef* usart)
{
  switch((uint32_t)usart)
	{
		case (uint32_t)USART1: return &lines[0];
		case (uint32_t)USART2: return &lines[1];
		case (uint32_t)USART3: return &lines[2];
		case (uint32_t)UART4: return &lines[3];
		case (uint32_t)UART5: return &lines[4];
		case (uint32_t)USART6: return &lines[5];
    default: return NULL;
	}
}

static struct usartx_line_resources_s* GetLineResources(USART_TypeDef* usart)
{
  return &GetLine(usart)->resources;
}

static void (*LL_DMA_ClearFlag_TCx[8])(DMA_TypeDef* dma) = {LL_DMA_ClearFlag_TC0,
                                                            LL_DMA_ClearFlag_TC1,
                                                            LL_DMA_ClearFlag_TC2,
                                                            LL_DMA_ClearFlag_TC3,
                                                            LL_DMA_ClearFlag_TC4,
                                                            LL_DMA_ClearFlag_TC5,
                                                            LL_DMA_ClearFlag_TC6,
                                                            LL_DMA_ClearFlag_TC7};

static inline void LL_DMA_ClearFlag_TC(DMA_TypeDef* dma, uint32_t dma_stream)
{
  LL_DMA_ClearFlag_TCx[dma_stream](dma);
}

static inline void StartDMA(DMA_TypeDef* dma, uint32_t dma_stream, uint8_t* data, uint32_t length)
{
  LL_DMA_ClearFlag_TC(dma, dma_stream);
  LL_DMA_SetMemoryAddress(dma, dma_stream, (uint32_t)data);
	LL_DMA_SetDataLength(dma, dma_stream, length);
  LL_DMA_EnableStream(dma, dma_stream);
}

static inline void StopDMA(DMA_TypeDef* dma, uint32_t dma_stream)
{
  LL_DMA_DisableStream(dma, dma_stream);
  LL_DMA_ClearFlag_TC(dma, dma_stream);
}

static void StartTx(struct usartx_line_resources_s* line_resources, uint32_t data_length)
{
  struct usartx_line_s* line = (struct usartx_line_s*)line_resources;
  
  /**  __disable_irq() is necessary because of connection issues
    * if some inetrrupt occurs between the LL_USART_EnableDirectionTx()
    * and something other before __enable_irq(). Did not figure out
    * what exectly it is, do not have time now. Maybe will do it later.
    * Anywhere it works fine now. I did the same for other functions here.
  */
  __disable_irq();
  if(line->config.is_line_half_duplex)
    LL_GPIO_SetOutputPin(line->de_gpio, line->de_pin);

  LL_USART_EnableDirectionTx(line->usart);

  LL_USART_ClearFlag_TC(line->usart);
  StartDMA(line->tx_dma, line->tx_dma_stream, line->resources.tx_buffer, data_length);
  /** 
    *  You have to wait until dma feeds data to the TX buffer
    *  otherwise TC flag may rise immedietely after EnableIT_TC
    */
  while(LL_USART_IsActiveFlag_TXE(line->usart));
  LL_USART_EnableIT_TC(line->usart);
  __enable_irq();
}

static void StopTx(struct usartx_line_resources_s* line_resources)
{
  struct usartx_line_s* line = (struct usartx_line_s*)line_resources;

  __disable_irq();
  LL_USART_DisableDirectionTx(line->usart);

  if(line->config.is_line_half_duplex)
    LL_GPIO_ResetOutputPin(line->de_gpio, line->de_pin);

  StopDMA(line->tx_dma, line->tx_dma_stream);
  LL_USART_DisableIT_TC(line->usart);
  __enable_irq();
}

static void StartRx(struct usartx_line_resources_s* line_resources)
{
  struct usartx_line_s* line = (struct usartx_line_s*)line_resources;

  __disable_irq();
  if(line->config.is_line_half_duplex)
    LL_GPIO_ResetOutputPin(line->de_gpio, line->de_pin);
  
  LL_USART_EnableDirectionRx(line->usart);

  LL_USART_ClearFlag_ORE(line->usart);
  LL_USART_ClearFlag_RXNE(line->usart);
  LL_USART_ClearFlag_IDLE(line->usart);
  
  StartDMA(line->rx_dma, line->rx_dma_stream, line->resources.rx_buffer, line->resources.rx_buffer_size);
  LL_USART_EnableIT_IDLE(line->usart);
  __enable_irq();
}

static void StopRx(struct usartx_line_resources_s* line_resources)
{
  struct usartx_line_s* line = (struct usartx_line_s*)line_resources;
  
  __disable_irq();
  LL_USART_DisableDirectionRx(line->usart);

  StopDMA(line->rx_dma, line->rx_dma_stream);
  LL_USART_DisableIT_IDLE(line->usart);
  __enable_irq();
}

static void SetRxTimeout(struct usartx_line_resources_s* line_resources, uint32_t timeout_us)
{
  struct usartx_line_s* line = (struct usartx_line_s*)line_resources;

  SetTimeout(line->timeout_channel, timeout_us, (void (*)(void*))line->config.RxTimeoutCallback, line_resources);
}

static void ResetRxTimeout(struct usartx_line_resources_s* line_resources)
{
  struct usartx_line_s* line = (struct usartx_line_s*)line_resources;

  ResetTimeout(line->timeout_channel);
}

static struct usartx_control_s usartx_control = {.GetLineResources = GetLineResources,
                                                  .StartTx = StartTx,
                                                  .StopTx = StopTx,
                                                  .StartRx = StartRx,
                                                  .StopRx = StopRx,
                                                  .SetRxTimeout = SetRxTimeout,
                                                  .ResetRxTimeout = ResetRxTimeout};

void USARTx_IRQ_Process(USART_TypeDef* usart)
{
  struct usartx_line_s* line = GetLine(usart);

  /*	LL_USART_IsActiveFlag_TC	*/
	if(LL_USART_IsActiveFlag_TC(usart))
	{
		LL_USART_ClearFlag_TC(usart);

		if(LL_USART_IsEnabledIT_TC(usart)){
      line->config.TxCompleteCallback(&line->resources);
    }
	}

	/*	LL_USART_IsActiveFlag_IDLE	*/
	if(LL_USART_IsActiveFlag_IDLE(usart)!=0)
	{
		LL_USART_ClearFlag_IDLE(usart);

		if(LL_USART_IsEnabledIT_IDLE(usart)){
      uint32_t data_length = line->resources.rx_buffer_size - LL_DMA_GetDataLength(line->rx_dma, line->rx_dma_stream);
      line->config.RxCompleteCallback(&line->resources, data_length);
		}
	}
}

static void BindProtocol(USART_TypeDef* usart, struct usartx_protocol_s* protocol)
{
  struct usartx_line_s* line = GetLine(usart);
  struct usartx_line_init_params_s line_init_params;

  line_init_params.line_resources = &line->resources;
  line_init_params.line_config = &line->config;
  line_init_params.usartx_control = &usartx_control;
  
  switch ((uint32_t)usart)
  {
    #if defined (USART1_USED)
      case (uint32_t)USART1:
        line->usart = USART1;
        line->tx_dma = DMA2;
        line->tx_dma_stream = LL_DMA_STREAM_7;
        line->rx_dma = DMA2;
        line->rx_dma_stream = LL_DMA_STREAM_5;
        line->resources.tx_buffer_size = USART1_TX_BUFFER_SIZE;
        line->resources.rx_buffer_size = USART1_RX_BUFFER_SIZE;
        line->resources.tx_buffer = usart1_tx_buffer;
        line->resources.rx_buffer = usart1_rx_buffer;
        line->timeout_channel = 0;
      break;
    #endif

    #if defined (USART2_USED)

    #endif

    #if defined (USART3_USED)
      case (uint32_t)USART3:
        line->usart = USART3;
        line->tx_dma = DMA1;
        line->tx_dma_stream = LL_DMA_STREAM_3;
        line->rx_dma = DMA1;
        line->rx_dma_stream = LL_DMA_STREAM_1;
        line->resources.tx_buffer_size = USART3_TX_BUFFER_SIZE;
        line->resources.rx_buffer_size = USART3_RX_BUFFER_SIZE;
        line->resources.tx_buffer = usart3_tx_buffer;
        line->resources.rx_buffer = usart3_rx_buffer;
        line->timeout_channel = 2;
        line->de_gpio = GPIOD;
        line->de_pin = UART3DE_PD9;
        break;
    #endif

    #if defined (UART4_USED)
      case (uint32_t)UART4:
        line->usart = UART4;
        line->tx_dma = DMA1;
        line->tx_dma_stream = LL_DMA_STREAM_4;
        line->rx_dma = DMA1;
        line->rx_dma_stream = LL_DMA_STREAM_2;
        line->resources.tx_buffer_size = UART4_TX_BUFFER_SIZE;
        line->resources.rx_buffer_size = UART4_RX_BUFFER_SIZE;
        line->resources.tx_buffer = uart4_tx_buffer;
        line->resources.rx_buffer = uart4_rx_buffer;
        line->timeout_channel = 3;
        line->de_gpio = GPIOA;
        line->de_pin = UART4DE_PA15;
        break;
    #endif

    #if defined (UART5_USED)
      case (uint32_t)UART5:
        line->usart = UART5;
        line->tx_dma = DMA1;
        line->tx_dma_stream = LL_DMA_STREAM_7;
        line->rx_dma = DMA1;
        line->rx_dma_stream = LL_DMA_STREAM_0;
        line->resources.tx_buffer_size = UART5_TX_BUFFER_SIZE;
        line->resources.rx_buffer_size = UART5_RX_BUFFER_SIZE;
        line->resources.tx_buffer = uart5_tx_buffer;
        line->resources.rx_buffer = uart5_rx_buffer;
        line->timeout_channel = 4;
        break;
    #endif

    #if defined (USART6_USED)
      case (uint32_t)USART6:
        line->usart = USART6;
        line->tx_dma = DMA2;
        line->tx_dma_stream = LL_DMA_STREAM_6;
        line->rx_dma = DMA2;
        line->rx_dma_stream = LL_DMA_STREAM_2;
        line->resources.tx_buffer_size = USART6_TX_BUFFER_SIZE;
        line->resources.rx_buffer_size = USART6_RX_BUFFER_SIZE;
        line->resources.tx_buffer = usart6_tx_buffer;
        line->resources.rx_buffer = usart6_rx_buffer;
        line->timeout_channel = 5;
        line->de_gpio = GPIOC;
        line->de_pin = UART6DE_PC7;
        break;
    #endif

    default:

      break;
  }

  protocol->InitLine(&line_init_params);
}

const struct usartx_s USARTx = {.BindProtocol = BindProtocol};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
