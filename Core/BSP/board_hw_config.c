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
#include "board_hw_config.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_system.h"


#define STEPPERS_CLOCK_FREQUENCY_HZ			(19200)

/*	USARTx_NVIC_IRQ_PRIORITY and USARTx_TIMEOUT_NVIC_IRQ_PRIORITY 
	must to be the same for correct operation!					*/
#define USARTx_NVIC_ALL_IRQ_PRIORITY		3
#define USARTx_NVIC_IRQ_PRIORITY 			USARTx_NVIC_ALL_IRQ_PRIORITY
#define USARTx_TIMEOUT_NVIC_IRQ_PRIORITY 	USARTx_NVIC_ALL_IRQ_PRIORITY
#define TIM1_STEPPERS_NVIC_IRQ_PRIORITY		1
#define DMA_WS2812B_NVIC_IRQ_PRIORITY		5



/**
  * @brief  The application entry point.
  * @retval int
  */
void SYSCFG_PWR_Init(void)
{
	/* Init the low level hardware */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClockConfig(void)
{
	/*	Configure the main internal regulator output voltage	*/
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
	
	/*	Enable HSE	*/
	LL_RCC_HSE_Enable();
	
	while(LL_RCC_HSE_IsReady()==0x0);
	
	/*	Configure PLL	for getting 168 MHz from 16 MHz*/
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE,LL_RCC_PLLM_DIV_8,168,LL_RCC_PLLP_DIV_2);
	LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE,LL_RCC_PLLM_DIV_8,168,LL_RCC_PLLQ_DIV_4);
	
	LL_RCC_PLL_Enable();
	
	while(LL_RCC_PLL_IsReady()==0x0);
	
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
	
	/*	Set PLL as system clock	*/
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Just need for FreeRTOS	*/
	SystemCoreClockUpdate();
}

void HW_ClockConfig(void)
{
	/*	Enable ALL GPIOx Clock	*/
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA); 
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC); 
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);

	/*	Force Reset DMA2	*/
	LL_AHB1_GRP1_ForceReset(LL_AHB1_GRP1_PERIPH_DMA2);
	/*	Release Reset DMA2	*/
	LL_AHB1_GRP1_ReleaseReset(LL_AHB1_GRP1_PERIPH_DMA2);
	/*	Enable DMA2 clock	*/
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

	/*	Force Reset DMA1	*/
	LL_AHB1_GRP1_ForceReset(LL_AHB1_GRP1_PERIPH_DMA1);
	/*	Release Reset DMA1	*/
	LL_AHB1_GRP1_ReleaseReset(LL_AHB1_GRP1_PERIPH_DMA1);
	/*	Enable DMA1 clock	*/
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
}

void USART1_TerminalInit(void)
{
	/*	Force Reset USART1	*/
	LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_USART1);
	/*	Release Reset USART1	*/
	LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_USART1);
	/*	Enable USART1 clock	*/
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
	
	LL_USART_ConfigAsyncMode(USART1);

	LL_USART_SetBaudRate(USART1,84000000,LL_USART_OVERSAMPLING_8,921600); //256000 1000000 921600 230400
	LL_USART_SetStopBitsLength(USART1,LL_USART_STOPBITS_1);
	LL_USART_SetOverSampling(USART1,LL_USART_OVERSAMPLING_8);
	LL_USART_SetDataWidth(USART1,LL_USART_DATAWIDTH_8B);
	LL_USART_SetParity(USART1,LL_USART_PARITY_NONE);
	LL_USART_SetTransferDirection(USART1,LL_USART_DIRECTION_TX_RX);

	LL_USART_Enable(USART1);
	
	/*	DMA TX config	*/
	LL_DMA_InitTypeDef DMA_InitStruct;
	
	LL_DMA_StructInit(&DMA_InitStruct);
	
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
	DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&(USART1->DR));
	DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_InitStruct.Channel = LL_DMA_CHANNEL_4;
	
	LL_DMA_Init(DMA2,LL_DMA_STREAM_7,&DMA_InitStruct);
	
	/*	Enable DMA Req TX Interrupt	*/
	LL_USART_EnableDMAReq_TX(USART1);
	
	
	/*	DMA RX config	*/
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&(USART1->DR));
	DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_InitStruct.Channel = LL_DMA_CHANNEL_4;
	
	LL_DMA_Init(DMA2,LL_DMA_STREAM_5,&DMA_InitStruct);
	
	/*	Enable DMA Req RX Interrupt	*/
	LL_USART_EnableDMAReq_RX(USART1);
	
	/*	Set Priority for NVIC USART1_IRQn	*/
	NVIC_SetPriority(USART1_IRQn, USARTx_NVIC_IRQ_PRIORITY);
	/*	Disable NVIC IRQ for USART1_IRQn	*/
	NVIC_EnableIRQ(USART1_IRQn);
}

void USART3_DynamixelInit(void)
{
	/*	Force Reset USART3	*/
	LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_USART3);
	/*	Release Reset USART3	*/
	LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_USART3);
	/*	Enable USART3 clock	*/
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
	
	LL_USART_ConfigHalfDuplexMode(USART3);
	
	LL_USART_SetBaudRate(USART3,42000000,LL_USART_OVERSAMPLING_8,1000000); //1000000 115000
	
	LL_USART_SetStopBitsLength(USART3,LL_USART_STOPBITS_1);
	LL_USART_SetOverSampling(USART3,LL_USART_OVERSAMPLING_8);
	LL_USART_SetDataWidth(USART3,LL_USART_DATAWIDTH_8B);
	LL_USART_SetParity(USART3,LL_USART_PARITY_NONE);
	LL_USART_SetTransferDirection(USART3,LL_USART_DIRECTION_RX);
	
	LL_USART_Enable(USART3);


	/*	DMA TX config	*/
	LL_DMA_InitTypeDef DMA_InitStruct;
	
	LL_DMA_StructInit(&DMA_InitStruct);
	
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
	DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&(USART3->DR));
	DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_InitStruct.Channel = LL_DMA_CHANNEL_4;
	
	LL_DMA_Init(DMA1,LL_DMA_STREAM_3,&DMA_InitStruct);
	
	/*	Enable DMA Req TX Interrupt	*/
	LL_USART_EnableDMAReq_TX(USART3);
	
	
	/*	DMA RX config	*/
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&(USART3->DR));
	DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_InitStruct.Channel = LL_DMA_CHANNEL_4;
	
	LL_DMA_Init(DMA1,LL_DMA_STREAM_1,&DMA_InitStruct);
	
	/*	Enable DMA Req RX Interrupt	*/
	LL_USART_EnableDMAReq_RX(USART3);
	
	
	/*	DMA M2M config	*/
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_MEMORY;
	DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT; 
	DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_INCREMENT;
	DMA_InitStruct.Channel = LL_DMA_CHANNEL_1;
	
	LL_DMA_Init(DMA2,LL_DMA_STREAM_0,&DMA_InitStruct);
	
	/*	Set Priority for NVIC USART3_IRQn	*/
	NVIC_SetPriority(USART3_IRQn, USARTx_NVIC_IRQ_PRIORITY);
	/*	Disable NVIC IRQ for USART3_IRQn	*/
	NVIC_EnableIRQ(USART3_IRQn);
}

void UART4_RS485_Init(void)
{
	/*	Force Reset UART4	*/
	LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_UART4);
	/*	Release Reset UART4	*/
	LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_UART4);
	/*	Enable UART4 clock	*/
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
	
	LL_USART_ConfigHalfDuplexMode(UART4);
	
	/*	Clock here = 42 MHz	*/
	LL_USART_SetBaudRate(UART4,42000000,LL_USART_OVERSAMPLING_16,2000000);
	LL_USART_SetStopBitsLength(UART4,LL_USART_STOPBITS_1);
	LL_USART_SetOverSampling(UART4,LL_USART_OVERSAMPLING_16);
	LL_USART_SetDataWidth(UART4,LL_USART_DATAWIDTH_8B);
	LL_USART_SetParity(UART4,LL_USART_PARITY_NONE);
	LL_USART_SetTransferDirection(UART4,LL_USART_DIRECTION_RX);
	
	LL_USART_Enable(UART4);


	/*	DMA TX config	*/
	LL_DMA_InitTypeDef DMA_InitStruct;
	
	LL_DMA_StructInit(&DMA_InitStruct);
	
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
	DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&(UART4->DR));
	DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_InitStruct.Channel = LL_DMA_CHANNEL_4;
	
	LL_DMA_Init(DMA1,LL_DMA_STREAM_4,&DMA_InitStruct);
	
	/*	Enable DMA Req TX Interrupt	*/
	LL_USART_EnableDMAReq_TX(UART4);
	
	
	/*	DMA RX config	*/
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&(UART4->DR));
	DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_InitStruct.Channel = LL_DMA_CHANNEL_4;
	
	LL_DMA_Init(DMA1,LL_DMA_STREAM_2,&DMA_InitStruct);
	
	/*	Enable DMA Req RX Interrupt	*/
	LL_USART_EnableDMAReq_RX(UART4);


	/*	Set Priority for NVIC UART4_IRQn	*/
	NVIC_SetPriority(UART4_IRQn, USARTx_NVIC_IRQ_PRIORITY);
	/*	Disable NVIC IRQ for UART4_IRQn	*/
	NVIC_EnableIRQ(UART4_IRQn);
}

void UART5_SerialInit(void)
{
	/*	Force Reset UART5	*/
	LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_UART5);
	/*	Release Reset UART5	*/
	LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_UART5);
	/*	Enable UART5 clock	*/
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
	
	LL_USART_ConfigAsyncMode(UART5);

	LL_USART_SetBaudRate(UART5,42000000,LL_USART_OVERSAMPLING_8,115200);
	LL_USART_SetStopBitsLength(UART5,LL_USART_STOPBITS_1);
	LL_USART_SetOverSampling(UART5,LL_USART_OVERSAMPLING_8);
	LL_USART_SetDataWidth(UART5,LL_USART_DATAWIDTH_8B);
	LL_USART_SetParity(UART5,LL_USART_PARITY_NONE);
	LL_USART_SetTransferDirection(UART5,LL_USART_DIRECTION_RX);

	LL_USART_Enable(UART5);
	
	/*	DMA TX config	*/
	LL_DMA_InitTypeDef DMA_InitStruct;
	
	LL_DMA_StructInit(&DMA_InitStruct);
	
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
	DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&(UART5->DR));
	DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_InitStruct.Channel = LL_DMA_CHANNEL_4;
	
	LL_DMA_Init(DMA1,LL_DMA_STREAM_7,&DMA_InitStruct);
	
	/*	Enable DMA Req TX Interrupt	*/
	LL_USART_EnableDMAReq_TX(UART5);
	
	
	/*	DMA RX config	*/
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&(UART5->DR));
	DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_InitStruct.Channel = LL_DMA_CHANNEL_4;
	
	LL_DMA_Init(DMA1,LL_DMA_STREAM_0,&DMA_InitStruct);
	
	/*	Enable DMA Req RX Interrupt	*/
	LL_USART_EnableDMAReq_RX(UART5);
	
	/*	Set Priority for NVIC UART5_IRQn	*/
	NVIC_SetPriority(UART5_IRQn, USARTx_NVIC_IRQ_PRIORITY);
	/*	Disable NVIC IRQ for UART5_IRQn	*/
	NVIC_EnableIRQ(UART5_IRQn);
}

void USART6_RS485_Init(void)
{
	/*	Force Reset USART6	*/
	LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_USART6);
	/*	Release Reset USART6	*/
	LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_USART6);
	/*	Enable USART6 clock	*/
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);
	
	LL_USART_ConfigHalfDuplexMode(USART6);
	
	LL_USART_SetBaudRate(USART6,84000000,LL_USART_OVERSAMPLING_16,2000000);
	LL_USART_SetStopBitsLength(USART6,LL_USART_STOPBITS_1);
	LL_USART_SetOverSampling(USART6,LL_USART_OVERSAMPLING_16);
	LL_USART_SetDataWidth(USART6,LL_USART_DATAWIDTH_8B);
	LL_USART_SetParity(USART6,LL_USART_PARITY_NONE);
	LL_USART_SetTransferDirection(USART6,LL_USART_DIRECTION_RX);
	
	LL_USART_Enable(USART6);


	/*	DMA TX config	*/
	LL_DMA_InitTypeDef DMA_InitStruct;
	
	LL_DMA_StructInit(&DMA_InitStruct);
	
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
	DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&(USART6->DR));
	DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_InitStruct.Channel = LL_DMA_CHANNEL_5;
	
	LL_DMA_Init(DMA2,LL_DMA_STREAM_6,&DMA_InitStruct);
	
	/*	Enable DMA Req TX Interrupt	*/
	LL_USART_EnableDMAReq_TX(USART6);
	
	
	/*	DMA RX config	*/
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&(USART6->DR));
	DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_InitStruct.Channel = LL_DMA_CHANNEL_5;
	
	LL_DMA_Init(DMA2,LL_DMA_STREAM_2,&DMA_InitStruct);
	
	/*	Enable DMA Req RX Interrupt	*/
	LL_USART_EnableDMAReq_RX(USART6);


	/*	Set Priority for NVIC USART6_IRQn	*/
	NVIC_SetPriority(USART6_IRQn, USARTx_NVIC_IRQ_PRIORITY);
	/*	Disable NVIC IRQ for USART6_IRQn	*/
	NVIC_EnableIRQ(USART6_IRQn);
}

void SPI1_Board_HW_Init(void)
{
	/*	Force Reset SPI1	*/
	LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_SPI1);
	/*	Release Reset SPI1	*/
	LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_SPI1);
	/*	Enable SPI1 clock	*/
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	
	LL_SPI_InitTypeDef SPI_InitStruct;
	
	LL_SPI_StructInit(&SPI_InitStruct);
	
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER; 
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
	
	LL_SPI_Init(SPI1, &SPI_InitStruct);
	
	LL_SPI_Enable(SPI1);
}

void TIM1_Steppers_PWM_Init(void)
{
	/*	Force Reset TIM1	*/
	LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM1);
	/*	Release Reset TIM1	*/
	LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM1);
	/*	Enable TIM1 clock	*/
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

	uint32_t tim_period;
	uint32_t prescaler = 0;

	while(1){
		tim_period = SystemCoreClock/(prescaler + 1)/STEPPERS_CLOCK_FREQUENCY_HZ;
		if(tim_period > 0xFFFF) prescaler ++;
		else break;
	}
	
	LL_TIM_SetClockSource(TIM1,LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_SetPrescaler(TIM1, prescaler);
	LL_TIM_SetClockDivision(TIM1, LL_TIM_CLOCKDIVISION_DIV1);
	LL_TIM_CC_EnablePreload(TIM1);
	
	LL_TIM_GenerateEvent_UPDATE(TIM1);

	
	LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetCounter(TIM1, 0x0); //16-bit
	LL_TIM_EnableARRPreload(TIM1);
	LL_TIM_SetAutoReload(TIM1, (tim_period - 1));
	
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
	
	LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH4,LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH3,LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH2,LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH1,LL_TIM_OCMODE_PWM1);

	LL_TIM_OC_ConfigOutput(TIM1,LL_TIM_CHANNEL_CH4,LL_TIM_OCPOLARITY_HIGH|LL_TIM_OCIDLESTATE_LOW);
	LL_TIM_OC_ConfigOutput(TIM1,LL_TIM_CHANNEL_CH3,LL_TIM_OCPOLARITY_HIGH|LL_TIM_OCIDLESTATE_LOW);
	LL_TIM_OC_ConfigOutput(TIM1,LL_TIM_CHANNEL_CH2,LL_TIM_OCPOLARITY_HIGH|LL_TIM_OCIDLESTATE_LOW);
	LL_TIM_OC_ConfigOutput(TIM1,LL_TIM_CHANNEL_CH1,LL_TIM_OCPOLARITY_HIGH|LL_TIM_OCIDLESTATE_LOW);
	
	LL_TIM_OC_SetCompareCH4(TIM1, (tim_period>>1));
	LL_TIM_OC_SetCompareCH3(TIM1, (tim_period>>1));
	LL_TIM_OC_SetCompareCH2(TIM1, (tim_period>>1));
	LL_TIM_OC_SetCompareCH1(TIM1, (tim_period>>1));
	
	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH4);
	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	
	LL_TIM_EnableCounter(TIM1);
	LL_TIM_EnableAllOutputs(TIM1);

	/*	Set Priority for NVIC TIM1_CC_IRQn	*/
	NVIC_SetPriority(TIM1_CC_IRQn, TIM1_STEPPERS_NVIC_IRQ_PRIORITY);
	/*	Enable NVIC IRQ for TIM1_CC_IRQn	*/
	NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void TIM8_WS2812B_Init(void)
{
	/*	Force Reset TIM8	*/
	LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM8);
	/*	Release Reset TIM8	*/
	LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM8);
	/*	Enable TIM8 clock	*/
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
	
	LL_TIM_SetClockSource(TIM8,LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_SetPrescaler(TIM8, 0);
	LL_TIM_SetClockDivision(TIM8, LL_TIM_CLOCKDIVISION_DIV1);
	LL_TIM_CC_EnablePreload(TIM8);
	
	LL_TIM_SetCounterMode(TIM8, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetCounter(TIM8, 0x0);
	LL_TIM_EnableARRPreload(TIM8);
	LL_TIM_SetAutoReload(TIM8, 210);
	
	LL_TIM_OC_EnablePreload(TIM8, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_SetMode(TIM8,LL_TIM_CHANNEL_CH3,LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_ConfigOutput(TIM8,LL_TIM_CHANNEL_CH3,LL_TIM_OCPOLARITY_HIGH|LL_TIM_OCIDLESTATE_LOW);
	LL_TIM_OC_SetCompareCH3(TIM8, 2);
	
	LL_TIM_GenerateEvent_UPDATE(TIM8);
	LL_TIM_EnableAllOutputs(TIM8);

	/*	Enable DMA Req TX Interrupt	*/
	LL_TIM_EnableDMAReq_UPDATE(TIM8);

	/*	DMA TX config	*/
	LL_DMA_InitTypeDef DMA_InitStruct;
	
	LL_DMA_StructInit(&DMA_InitStruct);
	
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
	DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&(TIM8->CCR3));
	DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
	DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
	DMA_InitStruct.Channel = LL_DMA_CHANNEL_7; 
	
	LL_DMA_Init(DMA2,LL_DMA_STREAM_1,&DMA_InitStruct);

	/*	Set Priority for NVIC DMA2_Stream1_IRQn	*/
	NVIC_SetPriority(DMA2_Stream1_IRQn, DMA_WS2812B_NVIC_IRQ_PRIORITY);
	/*	Enable NVIC IRQ for DMA2_Stream1_IRQn	*/
	NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

void TIM4_PWM_Group1234_Init(void)
{
	/*	Force Reset TIM4	*/
	LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM4);
	/*	Release Reset TIM4	*/
	LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM4);
	/*	Enable TIM4 clock	*/
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	
	/*	CLOCK HERE APB1 = 84 MHz	*/
	LL_TIM_SetClockSource(TIM4,LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_SetPrescaler(TIM4, (84-1));
	LL_TIM_CC_EnablePreload(TIM4);
	LL_TIM_SetCounterMode(TIM4, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetCounter(TIM4, 0x0); //16-bit
	LL_TIM_EnableARRPreload(TIM4);
	LL_TIM_SetAutoReload(TIM4, (20000-1)); //50 Hz

	LL_TIM_OC_SetMode(TIM4,LL_TIM_CHANNEL_CH1,LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM4,LL_TIM_CHANNEL_CH2,LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM4,LL_TIM_CHANNEL_CH3,LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM4,LL_TIM_CHANNEL_CH4,LL_TIM_OCMODE_PWM1);
	
	LL_TIM_OC_ConfigOutput(TIM4,LL_TIM_CHANNEL_CH1,LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_ConfigOutput(TIM4,LL_TIM_CHANNEL_CH2,LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_ConfigOutput(TIM4,LL_TIM_CHANNEL_CH3,LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_ConfigOutput(TIM4,LL_TIM_CHANNEL_CH4,LL_TIM_OCPOLARITY_HIGH);
	
	LL_TIM_OC_EnableFast(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_EnableFast(TIM4, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_EnableFast(TIM4, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_EnableFast(TIM4, LL_TIM_CHANNEL_CH4);
	
	LL_TIM_GenerateEvent_UPDATE(TIM4);
	LL_TIM_EnableCounter(TIM4);
	LL_TIM_EnableAllOutputs(TIM4);
}

void TIM5_PWM_Group5678_Init(void)
{
	/*	Force Reset TIM1	*/
	LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM5);
	/*	Release Reset TIM1	*/
	LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM5);
	/*	Enable TIM1 clock	*/
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
	
	/*	CLOCK HERE APB1 = 84 MHz	*/
	LL_TIM_SetClockSource(TIM5,LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_SetPrescaler(TIM5, (84-1));
	LL_TIM_CC_EnablePreload(TIM5);
	LL_TIM_SetCounterMode(TIM5, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetCounter(TIM5, 0x0); //16-bit
	LL_TIM_EnableARRPreload(TIM5);
	LL_TIM_SetAutoReload(TIM5, (20000-1)); //50 Hz

	LL_TIM_OC_SetMode(TIM5,LL_TIM_CHANNEL_CH1,LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM5,LL_TIM_CHANNEL_CH2,LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM5,LL_TIM_CHANNEL_CH3,LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM5,LL_TIM_CHANNEL_CH4,LL_TIM_OCMODE_PWM1);
	
	LL_TIM_OC_ConfigOutput(TIM5,LL_TIM_CHANNEL_CH1,LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_ConfigOutput(TIM5,LL_TIM_CHANNEL_CH2,LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_ConfigOutput(TIM5,LL_TIM_CHANNEL_CH3,LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_ConfigOutput(TIM5,LL_TIM_CHANNEL_CH4,LL_TIM_OCPOLARITY_HIGH);
	
	LL_TIM_OC_EnableFast(TIM5, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_EnableFast(TIM5, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_EnableFast(TIM5, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_EnableFast(TIM5, LL_TIM_CHANNEL_CH4);
	
	LL_TIM_GenerateEvent_UPDATE(TIM5);
	LL_TIM_EnableCounter(TIM5);
	LL_TIM_EnableAllOutputs(TIM5);
}

void TIM14_USARTx_TimeoutInit(void)
{
	/*	Force Reset TIM14 ch1	*/
	LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM14);
	/*	Release Reset TIM14	*/
	LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM14);
	/*	Enable TIM14 clock	*/
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);
	
	LL_TIM_SetPrescaler(TIM14, 0);
	LL_TIM_SetCounterMode(TIM14, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetCounter(TIM14, 0x0);
	LL_TIM_EnableARRPreload(TIM14);
	LL_TIM_SetAutoReload(TIM14, 8400);
	
	LL_TIM_DisableCounter(TIM14);
	
	/*	Set Priority for NVIC TIM8_TRG_COM_TIM14_IRQn	*/
	NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, USARTx_TIMEOUT_NVIC_IRQ_PRIORITY);
	/*	Enable NVIC IRQ for TIM8_TRG_COM_TIM14_IRQn	*/
	NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
