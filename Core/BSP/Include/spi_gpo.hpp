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

#include "std_def.hpp"

/*  24 outputs of the shift registers on SPI1, latched by STCP (PA4)  */
class SPI_GPO_Driver{
public:
  void SetOutputPin(uint32_t spi_gpo_pin);
  void ResetOutputPin(uint32_t spi_gpo_pin);

private:
  void Update();

  uint32_t spi_gpo_reg = 0;
};

extern SPI_GPO_Driver SPI_GPO;

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
