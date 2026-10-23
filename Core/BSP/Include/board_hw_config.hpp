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
#ifndef __BOARD_HW_CONFIG_H
#define __BOARD_HW_CONFIG_H


#include "std_def.hpp"

	 
void SYSCFG_PWR_Init(void);
void SystemClockConfig(void);
void HW_ClockConfig(void);

void IWDG_Init(void);

void SPI1_Board_HW_Init(void);

void I2C2_Init(void);

void USART1_TerminalInit(void);
void USART3_DynamixelInit(void);
void UART4_RS485_Init(void);
void UART5_SerialInit(void);
void USART6_RS485_Init(void);

void TIM1_Steppers_PWM_Init(void);
void TIM4_PWM_Group1234_Init(void);
void TIM5_PWM_Group5678_Init(void);
void TIM8_WS2812B_Init(void);
void TIM14_USARTx_TimeoutInit(void);



#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
