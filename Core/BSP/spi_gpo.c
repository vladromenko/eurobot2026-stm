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

#include "spi_gpo.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.h"

#include "FreeRTOS.h"
#include "task.h"

// #pragma O0

uint32_t SPI_GPO_REG = 0x0;

void SPI_GPO_SetOutputPin(uint32_t SPI_GPO_PIN)
{
	uint8_t temp;
	
	SPI_GPO_REG |= SPI_GPO_PIN;

	vTaskSuspendAll();
	
	LL_GPIO_ResetOutputPin(GPIOA, STCP_PA4);
	
	LL_SPI_TransmitData8(SPI1, SPI_GPO_REG&0xFF);
	temp = (SPI_GPO_REG>>8)&0xFF;
	while(LL_SPI_IsActiveFlag_TXE(SPI1)==0);
	LL_SPI_TransmitData8(SPI1, temp);
	temp = (SPI_GPO_REG>>16)&0xFF;
	while(LL_SPI_IsActiveFlag_TXE(SPI1)==0);
	LL_SPI_TransmitData8(SPI1, temp);
	while(LL_SPI_IsActiveFlag_TXE(SPI1)==0);
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	LL_GPIO_SetOutputPin(GPIOA, STCP_PA4);

	xTaskResumeAll();
}

void SPI_GPO_ResetOutputPin(uint32_t SPI_GPO_PIN)
{
	uint8_t temp;
	
	SPI_GPO_REG &= ~SPI_GPO_PIN;

	vTaskSuspendAll();
	
	LL_GPIO_ResetOutputPin(GPIOA, STCP_PA4);
	
	LL_SPI_TransmitData8(SPI1, SPI_GPO_REG&0xFF);
	temp = (SPI_GPO_REG>>8)&0xFF;
	while(LL_SPI_IsActiveFlag_TXE(SPI1)==0);
	LL_SPI_TransmitData8(SPI1, temp);
	temp = (SPI_GPO_REG>>16)&0xFF;
	while(LL_SPI_IsActiveFlag_TXE(SPI1)==0);
	LL_SPI_TransmitData8(SPI1, temp);
	while(LL_SPI_IsActiveFlag_TXE(SPI1)==0);
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	LL_GPIO_SetOutputPin(GPIOA, STCP_PA4);

	xTaskResumeAll();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
