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
#include "stm32f4xx_ll_gpio.h"
#include "robot_gpio_defines.hpp"
#include "robot_gpio_config.hpp"


void Robot_GPIO_Config(void)
{
	LL_GPIO_InitTypeDef GPIO_Struct;

	/*	Common Robot Pins ------------------------------------------------------------------*/

	/*	STARTING_CORD_BTN_GPIO14_PC15	*/
	GPIO_Struct.Pin = STARTING_CORD_BTN_GPIO14_PC15;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOC, &GPIO_Struct);
	
	/*	PLAYGROUND_SIDE_BTN_GPIO13_PE6	*/
	GPIO_Struct.Pin = PLAYGROUND_SIDE_BTN_GPIO13_PE6;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);
	
	/*	STRATEGY_BTN_GPIO12_PE5	*/
	GPIO_Struct.Pin = STRATEGY_BTN_GPIO12_PE5;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);
	
	/*	MAXON_ENABLE_BTN_GPIO11_PE2	*/
	GPIO_Struct.Pin = MAXON_ENABLE_BTN_GPIO11_PE2;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOE, &GPIO_Struct);

	/* Linear switches pins en*/
	// linear switch 1
	GPIO_Struct.Pin = LINEAR_SWITCH_1_GPIO28;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOC, &GPIO_Struct);

	// linear switch 2
	GPIO_Struct.Pin = LINEAR_SWITCH_2_GPIO26;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOC, &GPIO_Struct);
	
	// linear switch 3
	GPIO_Struct.Pin = LINEAR_SWITCH_3_GPIO24;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOE, &GPIO_Struct);
	
	// linear switch 4
	GPIO_Struct.Pin = LINEAR_SWITCH_4_GPIO22;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOE, &GPIO_Struct);
	
	// linear switch 5
	GPIO_Struct.Pin = LINEAR_SWITCH_5_GPIO20;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOE, &GPIO_Struct);
	
	// linear switch 6
	GPIO_Struct.Pin = LINEAR_SWITCH_6_GPIO18;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOB, &GPIO_Struct);
	
	// linear switch 7
	GPIO_Struct.Pin = LINEAR_SWITCH_7_GPIO16;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOB, &GPIO_Struct);
	
	// linear switch 8
	GPIO_Struct.Pin = LINEAR_SWITCH_8_GPIO14;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOB, &GPIO_Struct);
	
	// linear switch 9
	GPIO_Struct.Pin = LINEAR_SWITCH_9_GPIO12;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOD, &GPIO_Struct);

	// // Enable ANALOG pin
	// GPIO_Struct.Pin = AIN1_PC3;
	// GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	// GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	// GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	// GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	// LL_GPIO_Init(GPIOE, &GPIO_Struct);


	/*	Specific Robot Pins ------------------------------------------------------------------*/

	/*  Specific Robot GUIDO Pins  */
	#if defined (ROBOT_GUIDO)

	/*	CART_FRONT_BTN_GPIOD_PIN4	*/
	GPIO_Struct.Pin = CART_FRONT_BTN_GPIOD_PIN4;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);
	
	/*	CART_BACK_BTN_GPIOD_PIN3	*/
	GPIO_Struct.Pin = CART_BACK_BTN_GPIOD_PIN3;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);
	
	/*	STATUE_BTN_GPIOD_PIN7	*/
	GPIO_Struct.Pin = STATUE_BTN_GPIOD_PIN7;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);
	
	#endif
	

	/*  Specific Robot BIG Pins  */
	#if defined (ROBOT_BIG)

	/*	STEPPER_LEFT_BTN_GPIOD_PIN4	*/
	GPIO_Struct.Pin = STEPPER_LEFT_BTN_GPIOD_PIN4;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);
	
	/*	STEPPER_RIGHT_BTN_GPIOD_PIN3	*/
	GPIO_Struct.Pin = STEPPER_RIGHT_BTN_GPIOD_PIN3;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);
	
	/*	GRIPPER_BTN_GPIOD_PIN7	*/
	GPIO_Struct.Pin = GRIPPER_BTN_GPIOD_PIN7;
	GPIO_Struct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_DOWN;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(GPIOD, &GPIO_Struct);
	
	#endif
	
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
