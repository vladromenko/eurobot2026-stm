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

#ifdef __cplusplus
 extern "C" {
#endif
	 
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

struct ws2812b_s{
  void (*Set_RGB_Color)(struct rgb_color_s* leds, uint8_t leds_number);
  void (*Set_HSV_Color)(struct hsv_color_s* leds, uint8_t leds_number);
};

extern const struct ws2812b_s WS2812B;

void DMA_WS2812B_IRQ_Process(void);

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
