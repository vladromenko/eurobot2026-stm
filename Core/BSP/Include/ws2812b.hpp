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
#ifndef __WS2812B_H
#define __WS2812B_H

#include "std_def.hpp"

struct rgb_color_s{
  uint8_t g;
  uint8_t r;
  uint8_t b;
};

struct hsv_color_s{
  uint8_t h;
  uint8_t s;
  uint8_t v;
};

/*  LED strip on TIM8 CH3 (PC8), fed by DMA2 Stream1  */
class WS2812B_Driver{
public:
  static constexpr uint32_t MAX_LEDS_NUMBER = 60;

  void Set_RGB_Color(const rgb_color_s* leds, uint8_t leds_number);
  void Set_HSV_Color(const hsv_color_s* leds, uint8_t leds_number);
  void DMA_IRQ_Process();
};

/*  The object lives in HwInstance (hw_instance.hpp); this is a shortcut to it  */
extern WS2812B_Driver& WS2812B;

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
