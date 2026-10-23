/**
  ******************************************************************************
  * @file    	GPIOs_Defines.h
  * @brief   HAL configuration file.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_ROBOT_DEFINES_H
#define __GPIO_ROBOT_DEFINES_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "board_gpio_defines.hpp"


/*  Common Robot Pins  */
#define	STARTING_CORD_BTN_GPIO14_PC15			GPIO27_PC15
#define	PLAYGROUND_SIDE_BTN_GPIO13_PE6	  GPIO25_PE6
#define	STRATEGY_BTN_GPIO12_PE5				    GPIO23_PE5
#define	MAXON_ENABLE_BTN_GPIO11_PE2				GPIO21_PE2

/*  Additional linear switchees pins  */
#define	LINEAR_SWITCH_1_GPIO28			GPIO28_PC14
#define	LINEAR_SWITCH_2_GPIO26			GPIO26_PC13
#define	LINEAR_SWITCH_3_GPIO24			GPIO24_PE4
#define	LINEAR_SWITCH_4_GPIO22			GPIO22_PE3 
#define	LINEAR_SWITCH_5_GPIO20			GPIO20_PE0
#define	LINEAR_SWITCH_6_GPIO18			GPIO18_PB7
#define	LINEAR_SWITCH_7_GPIO16			GPIO16_PB4
#define	LINEAR_SWITCH_8_GPIO14			GPIO14_PB3
#define	LINEAR_SWITCH_9_GPIO12			GPIO12_PD3

/*  Specific Robot GUIDO Pins  */
#if defined (ROBOT_GUIDO)

#define	CART_FRONT_BTN_GPIOD_PIN4			LL_GPIO_PIN_4
#define	CART_BACK_BTN_GPIOD_PIN3			LL_GPIO_PIN_3
#define	STATUE_BTN_GPIOD_PIN7				  LL_GPIO_PIN_7

#endif

/*  Specified Robot BIG Pins  */
#if defined (ROBOT_BIG)

#define	STEPPER_LEFT_BTN_GPIOD_PIN4			LL_GPIO_PIN_4
#define	STEPPER_RIGHT_BTN_GPIOD_PIN3		LL_GPIO_PIN_3
#define	GRIPPER_BTN_GPIOD_PIN7				  LL_GPIO_PIN_7

#endif


#ifdef __cplusplus
}
#endif

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
