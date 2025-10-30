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
#ifndef __SPI_GPO_H
#define __SPI_GPO_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.h"

void SPI_GPO_SetOutputPin(uint32_t SPI_GPO_PIN);
void SPI_GPO_ResetOutputPin(uint32_t SPI_GPO_PIN);

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
