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
#include "ws2812b.hpp" 
#include "byte2word.hpp"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_tim.h"
#include "board_gpio_defines.hpp"
#include "stm32f4xx_ll_gpio.h"


#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

namespace {

constexpr uint16_t CCR_HIGH = 142;
constexpr uint16_t CCR_LOW = 68;

/*  One timer compare value per bit, 24 bits per LED. Kept outside the class
    so the linker can drop it when the strip is not used  */
uint16_t leds_dma_data[WS2812B_Driver::MAX_LEDS_NUMBER*24];

}

constinit WS2812B_Driver WS2812B;


void WS2812B_Driver::Set_RGB_Color(const rgb_color_s* leds, uint8_t leds_number)
{
  if(leds_number>MAX_LEDS_NUMBER)
    return;

  /*  Colors go out byte by byte in memory order (G, R, B), MSB first  */
  const uint8_t* color_bytes = reinterpret_cast<const uint8_t*>(leds);
  uint32_t dma_data_size = (uint32_t)leds_number*24;

  for(uint32_t bit_number = 0; bit_number < dma_data_size; bit_number++)
    leds_dma_data[bit_number] = (color_bytes[bit_number/8]&(0x80>>(bit_number%8))) ? CCR_HIGH : CCR_LOW;

  LL_DMA_SetMemoryAddress(DMA2,LL_DMA_STREAM_1,(uint32_t)leds_dma_data);
	LL_DMA_SetDataLength(DMA2,LL_DMA_STREAM_1,dma_data_size);
  LL_DMA_ClearFlag_TC1(DMA2);
  LL_DMA_EnableStream(DMA2,LL_DMA_STREAM_1);

  LL_DMA_EnableIT_TC(DMA2,LL_DMA_STREAM_1);

  vTaskSuspendAll();
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH3);
  LL_TIM_GenerateEvent_COM(TIM8);
  LL_TIM_EnableCounter(TIM8);
  xTaskResumeAll();
}

void WS2812B_Driver::Set_HSV_Color(const hsv_color_s* leds, uint8_t leds_number)
{
  (void)leds;
  if(leds_number>MAX_LEDS_NUMBER)
    return;

}

void WS2812B_Driver::DMA_IRQ_Process()
{
  if(LL_DMA_IsActiveFlag_TC1(DMA2))
  {
    LL_DMA_ClearFlag_TC1(DMA2);

    if(LL_DMA_IsEnabledIT_TC(DMA2,LL_DMA_STREAM_1)){
      LL_DMA_DisableIT_TC(DMA2,LL_DMA_STREAM_1);
      LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_1);
      LL_TIM_CC_DisableChannel(TIM8, LL_TIM_CHANNEL_CH3);
      LL_TIM_GenerateEvent_COM(TIM8);
      LL_TIM_DisableCounter(TIM8);
    }
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
