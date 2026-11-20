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

#include "board_gpio_defines.h"


/*  Common Robot Pins  */
#define	STARTING_CORD_BTN_GPIO14_PC15			GPIO27_PC15
#define	PLAYGROUND_SIDE_BTN_GPIO13_PE6	  GPIO25_PE6
#define	STRATEGY_BTN_GPIO12_PE5				    GPIO23_PE5
#define	MAXON_ENABLE_BTN_GPIO11_PE2				GPIO21_PE2

/*  Impeller: speed signal PWM8/PA3 (RC_Servo 8), button (open-drain to GND) on GPIO5/PA8  */
#define	IMPELLER_SPEED_RC_SERVO			RC_SERVO_8
#define	IMPELLER_SPEED_GPIO_PORT		GPIOA
#define	IMPELLER_SPEED_PWM8_PA3			PWM8_PA3
#define	IMPELLER_BTN_GPIO_PORT			GPIOA
#define	IMPELLER_BTN_GPIO5_PA8			GPIO5_PA8


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
