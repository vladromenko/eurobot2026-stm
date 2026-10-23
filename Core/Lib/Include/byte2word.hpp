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
#ifndef __BYTE_TO_WORD_H
#define __BYTE_TO_WORD_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "string.h" 


static inline void ByteTo_Word32(uint8_t* byte, void* var)
{
  memcpy(var,byte,4);
}

static inline void Word32_ToByte(void* var, uint8_t* byte)
{
  memcpy(byte,var,4);
}

static inline void ByteTo_Word16(uint8_t* byte, void* var)
{
  memcpy(var,byte,2);
}

static inline void Word16_ToByte(void* var, uint8_t* byte)
{
  memcpy(byte,var,2);
}

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
