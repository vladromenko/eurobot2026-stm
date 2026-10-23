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
#include "l6474_drvr.hpp" 
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.hpp"
#include "spi_gpo_defines.hpp"
#include "spi_gpo.hpp" 

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

namespace {

constexpr uint32_t spi_nrst_gpio[] = {SPI_L6474_NRST1,SPI_L6474_NRST2,SPI_L6474_NRST3,SPI_L6474_NRST4};
GPIO_TypeDef* const spi_ncs_gpio[] = {GPIOE,GPIOD,GPIOD,GPIOC};
constexpr uint32_t spi_ncs_pin[] = {NCS1_PE7,NCS2_PD10,NCS3_PD11,NCS4_PC9};
GPIO_TypeDef* const dir_gpio[] = {GPIOE,GPIOE,GPIOE,GPIOE};
constexpr uint32_t dir_pin[] = {DIR1_PE8,DIR2_PE10,DIR3_PE12,DIR4_PE15};

}

L6474H_Driver L6474H;


void L6474H_Driver::SetParam(l6474h_motor_s motor, uint8_t address, uint32_t data, uint8_t length)
{
	vTaskSuspendAll();

	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV32);
	
	LL_GPIO_ResetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
	
	LL_SPI_TransmitData8(SPI1, L6474_SetParam_CMD|(address&0x1F));
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	while(length>0){
		LL_GPIO_SetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
		length--;
		LL_GPIO_ResetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
		
		LL_SPI_TransmitData8(SPI1, (data>>(length*8))&0xFF);
		while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	}
	
	LL_GPIO_SetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV2);

	xTaskResumeAll();
}

uint32_t L6474H_Driver::GetParam(l6474h_motor_s motor, uint8_t address, uint8_t length)
{
	volatile uint32_t temp;
	
	temp = 0;

	vTaskSuspendAll();
	
	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV32);
	
	LL_GPIO_ResetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
	
	LL_SPI_TransmitData8(SPI1, L6474_GetParam_CMD|(address&0x1F));
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	while(LL_SPI_IsActiveFlag_RXNE(SPI1)!=0)
		LL_SPI_ReceiveData8(SPI1);
	
	while(length>0){
		LL_GPIO_SetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
		length--;
		LL_GPIO_ResetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
		
		LL_SPI_TransmitData8(SPI1, L6474_NOP_CMD);
		while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
		
		temp |= ((uint32_t)LL_SPI_ReceiveData8(SPI1))<<(length*8);
	}

	LL_GPIO_SetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
	
	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV2);

	xTaskResumeAll();
	
	return temp;
}

uint32_t L6474H_Driver::GetStatus(l6474h_motor_s motor)
{
	volatile uint32_t temp;

	vTaskSuspendAll();
	
	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV32);
	
	LL_GPIO_ResetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
	
	LL_SPI_TransmitData8(SPI1, L6474_GetStatus_CMD);
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	while(LL_SPI_IsActiveFlag_RXNE(SPI1)!=0)
		LL_SPI_ReceiveData8(SPI1);
	
	
	LL_GPIO_SetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
	LL_GPIO_ResetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
	
	LL_SPI_TransmitData8(SPI1, L6474_NOP_CMD);
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	temp = ((uint32_t)LL_SPI_ReceiveData8(SPI1))<<8;
	
	LL_GPIO_SetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
	LL_GPIO_ResetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
	
	LL_SPI_TransmitData8(SPI1, L6474_NOP_CMD);
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	
	temp |= LL_SPI_ReceiveData8(SPI1);
	
	
	LL_GPIO_SetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
	
	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV2);

	xTaskResumeAll();
	
	return temp;
}

void L6474H_Driver::Enable(l6474h_motor_s motor)
{
	vTaskSuspendAll();

	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV32);
	
	LL_GPIO_ResetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
	
	LL_SPI_TransmitData8(SPI1, L6474_ENABLE_CMD);
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	LL_GPIO_SetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
	
	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV2);

	xTaskResumeAll();
}

void L6474H_Driver::Disable(l6474h_motor_s motor)
{
	vTaskSuspendAll();

	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV32);
	
	LL_GPIO_ResetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
	
	LL_SPI_TransmitData8(SPI1, L6474_DISABLE_CMD);
	while(LL_SPI_IsActiveFlag_BSY(SPI1)!=0);
	
	LL_GPIO_SetOutputPin(spi_ncs_gpio[motor], spi_ncs_pin[motor]);
	
	LL_SPI_SetBaudRatePrescaler(SPI1,LL_SPI_BAUDRATEPRESCALER_DIV2);

	xTaskResumeAll();
}

void L6474H_Driver::SetStepMode(l6474h_motor_s motor, uint32_t step_mode)
{
	SetParam(motor, L6474_STEP_MODE, (step_mode&L6474_STEP_SEL_MASK)|0x88, L6474_STEP_MODE_LENGTH);
}

void L6474H_Driver::SetCurrentLimit(l6474h_motor_s motor, uint32_t current_limit)
{
	SetParam(motor, L6474_TVAL, current_limit&L6474_TVAL_MASK,1);
}

void L6474H_Driver::SetCurrentLimit_mA(l6474h_motor_s motor, float current_limit_mA)
{
	uint8_t temp;
	if(current_limit_mA>=31.25f){
		temp = (uint8_t)((current_limit_mA - 31.25f)/31.25f);
	}else{
		temp = 0;
	}
	SetParam(motor, L6474_TVAL, temp&L6474_TVAL_MASK, L6474_TVAL_LENGTH);
}

void L6474H_Driver::SetDirection(l6474h_motor_s motor, l6474h_motor_dir_s direction)
{
	if(direction)
		LL_GPIO_SetOutputPin(dir_gpio[motor], dir_pin[motor]);
	else
		LL_GPIO_ResetOutputPin(dir_gpio[motor], dir_pin[motor]);
}

void L6474H_Driver::HardReset(l6474h_motor_s motor)
{
	SPI_GPO.ResetOutputPin(spi_nrst_gpio[motor]);
	vTaskDelay(10);
	SPI_GPO.SetOutputPin(spi_nrst_gpio[motor]);
	vTaskDelay(10);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
