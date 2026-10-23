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

#define MAX_LEDS_NUMBER   60

#define CCR_HIGH          142      
#define CCR_LOW           68 


static uint16_t leds_dma_data[MAX_LEDS_NUMBER*24];


static void Set_RGB_Color(struct rgb_color_s* leds, uint8_t leds_number)
{
  if(leds_number>MAX_LEDS_NUMBER)
    return;

  uint32_t dma_data_size = (uint32_t)leds_number*24;
  uint32_t bit_number = 0;

  while(bit_number!=dma_data_size)
  {
    for(uint8_t ik=0; ik<8; ik++){
      leds_dma_data[bit_number] = ((*(uint8_t*)leds)&((0x80>>ik))) ? CCR_HIGH : CCR_LOW;
      bit_number++;
    }
    
		leds = (struct rgb_color_s*)(*(uint32_t*)&leds + 1);
  }

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

static void Set_HSV_Color(struct hsv_color_s* leds, uint8_t leds_number)
{
  if(leds_number>MAX_LEDS_NUMBER)
    return;

}

void DMA_WS2812B_IRQ_Process(void)
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

const struct ws2812b_s WS2812B = {.Set_RGB_Color = Set_RGB_Color,
                                  .Set_HSV_Color = Set_HSV_Color};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
