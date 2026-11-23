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


#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_system.h"
#include "board_gpio_defines.h"
#include "board_gpio_config.h"


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void Board_GPIO_Config(void)
{
	LL_GPIO_InitTypeDef GPIO_Struct;

	/*	RS-485 & USARTx ------------------------------------------------------------------*/

	/*	USART1 RX pin	*/
	GPIO_Struct.Pin = UART1RX_PA10;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_7;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);
	
	/*	USART1 TX pin	*/
	GPIO_Struct.Pin = UART1TX_PA9;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_7;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);

	/*	USART2TXRX RS-485	*/
	GPIO_Struct.Pin = UART2TXRX_PD5;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	//GPIO_Struct.Alternate  = LL_GPIO_AF_8;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);
	
	/*	USART2DE RS-485	*/
	LL_GPIO_ResetOutputPin(GPIOD, UART2DE_PD6);
	
	GPIO_Struct.Pin = UART2DE_PD6;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	USART4TXRX RS-485	*/
	GPIO_Struct.Pin = UART4TXRX_PC10;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_8;
	
	LL_GPIO_Init(GPIOC, &GPIO_Struct);
	
	/*	USART4DE RS-485	*/
	LL_GPIO_ResetOutputPin(GPIOA, UART4DE_PA15);
	
	GPIO_Struct.Pin = UART4DE_PA15;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);

	/*	USART3 TX/RX DYNAMIXEL	*/
	GPIO_Struct.Pin = UART3TXRX_PD8;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_7;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);
	
	/*	USART3 DE DYNAMIXEL	*/
	LL_GPIO_ResetOutputPin(GPIOD, UART3DE_PD9);
	
	GPIO_Struct.Pin = UART3DE_PD9;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	UART5 RX pin	*/
	GPIO_Struct.Pin = UART5RX_PD2;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_8;
	// LL_GPIO_ResetOutputPin(GPIOD, UART5RX_PD2);
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);
	
	/*	UART5 TX pin	*/
	GPIO_Struct.Pin = UART5TX_PC12;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_8;
	
	LL_GPIO_Init(GPIOC, &GPIO_Struct);
	
	/*	USART6TXRX RS-485	*/
	GPIO_Struct.Pin = UART6TXRX_PC6;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_8;
	
	LL_GPIO_Init(GPIOC, &GPIO_Struct);
	
	/*	USART6DE RS-485	*/
	LL_GPIO_ResetOutputPin(GPIOC, UART6DE_PC7);
	
	GPIO_Struct.Pin = UART6DE_PC7;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOC, &GPIO_Struct);
	
	

	
	/*	Shift Register Control  ------------------------------------------------------------------*/

	/*	STCP	*/
	LL_GPIO_SetOutputPin(GPIOA, STCP_PA4);
	
	GPIO_Struct.Pin = STCP_PA4;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);

	/*	SPI1	*/
	GPIO_Struct.Pin = SPI1_SCK_PA5|SPI1_MISO_PA6|SPI1_MOSI_PA7;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_5;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);


	/*	4x L6474H Control ------------------------------------------------------------------*/
	
	/*	NCS1	*/
	LL_GPIO_SetOutputPin(GPIOE, NCS1_PE7);
	
	GPIO_Struct.Pin = NCS1_PE7;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);

	/*	NCS2	*/
	LL_GPIO_SetOutputPin(GPIOD, NCS2_PD10);
	
	GPIO_Struct.Pin = NCS2_PD10;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	NCS3	*/
	LL_GPIO_SetOutputPin(GPIOD, NCS3_PD11);
	
	GPIO_Struct.Pin = NCS3_PD11;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	NCS4	*/
	LL_GPIO_SetOutputPin(GPIOC, NCS4_PC9);
	
	GPIO_Struct.Pin = NCS4_PC9;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOC, &GPIO_Struct);

	/*	STCK1	*/
	GPIO_Struct.Pin = STCK1_PE9;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_1;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);

	/*	STCK2	*/
	GPIO_Struct.Pin = STCK2_PE11;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_1;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);
	
	/*	STCK3	*/
	GPIO_Struct.Pin = STCK3_PE13;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_1;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);
	
	/*	STCK4	*/
	GPIO_Struct.Pin = STCK4_PE14;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_1;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);

	/*	DIR1	*/
	LL_GPIO_SetOutputPin(GPIOE, DIR1_PE8);
	
	GPIO_Struct.Pin = DIR1_PE8;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);

	/*	DIR2	*/
	LL_GPIO_SetOutputPin(GPIOE, DIR2_PE10);
	
	GPIO_Struct.Pin = DIR2_PE10;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);
	
	/*	DIR3	*/
	LL_GPIO_SetOutputPin(GPIOE, DIR3_PE12);
	
	GPIO_Struct.Pin = DIR3_PE12;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);
	
	/*	DIR4	*/
	LL_GPIO_SetOutputPin(GPIOE, DIR4_PE15);
	
	GPIO_Struct.Pin = DIR4_PE15;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);
	
	
	/*	I2C2 (PB10=SCL, PB11=SDA) -----------------------------------------------*/

	GPIO_Struct.Pin = I2C2_2_SCL_PB10;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_UP;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_4;
	LL_GPIO_Init(GPIOB, &GPIO_Struct);

	GPIO_Struct.Pin = I2C2_3_SDA_PB11;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_UP;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_4;
	LL_GPIO_Init(GPIOB, &GPIO_Struct);


	/*	PWM Pins ------------------------------------------------------------------*/
	
	/*	PWM1	*/
	GPIO_Struct.Pin = PWM1_PD15;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	PWM2	*/
	GPIO_Struct.Pin = PWM2_PD14;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	PWM3	*/
	GPIO_Struct.Pin = PWM3_PD13;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	PWM4	*/
	GPIO_Struct.Pin = PWM4_PD12;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	/*	PWM5	*/
	GPIO_Struct.Pin = PWM5_PA0;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);

	/*	PWM6	*/
	GPIO_Struct.Pin = PWM6_PA1;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);

	/*	PWM7	*/
	GPIO_Struct.Pin = PWM7_PA2;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);

	/*	PWM8	*/
	GPIO_Struct.Pin = PWM8_PA3;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOA, &GPIO_Struct);


	/*	WS2812B ------------------------------------------------------------------*/
	
	/*	WS2812B pin	*/
	GPIO_Struct.Pin = WS2812B_PC8;
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_UP;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_3;
	
	LL_GPIO_Init(GPIOC, &GPIO_Struct);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
