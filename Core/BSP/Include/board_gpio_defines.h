/**
  ******************************************************************************
  * @file    	GPIOs_Defines.h
  * @brief   HAL configuration file.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_GPIO_DEFINES_H
#define __BOARD_GPIO_DEFINES_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_ll_gpio.h"


/*  RS-485 & USARTx  */
#define	UART1RX_PA10				LL_GPIO_PIN_10
#define	UART1TX_PA9				  LL_GPIO_PIN_9

#define	UART2TXRX_PD5			  LL_GPIO_PIN_5
#define	UART2DE_PD6				  LL_GPIO_PIN_6

#define	UART3TXRX_PD8			  LL_GPIO_PIN_8
#define	UART3DE_PD9				  LL_GPIO_PIN_9
	 
#define	UART4TXRX_PC10			LL_GPIO_PIN_10
#define	UART4DE_PA15				LL_GPIO_PIN_15

#define	UART5RX_PD2				  LL_GPIO_PIN_2
#define	UART5TX_PC12				LL_GPIO_PIN_12

#define	UART6TXRX_PC6			  LL_GPIO_PIN_6
#define	UART6DE_PC7				  LL_GPIO_PIN_7

/*  4x L6474H Control  */
#define	NCS4_PC9						LL_GPIO_PIN_9
#define	STCK4_PE14					LL_GPIO_PIN_14
#define	DIR4_PE15					  LL_GPIO_PIN_15

#define	NCS3_PD11					  LL_GPIO_PIN_11
#define	STCK3_PE13					LL_GPIO_PIN_13
#define	DIR3_PE12					  LL_GPIO_PIN_12

#define	NCS2_PD10					  LL_GPIO_PIN_10
#define	STCK2_PE11					LL_GPIO_PIN_11
#define	DIR2_PE10					  LL_GPIO_PIN_10

#define	NCS1_PE7						LL_GPIO_PIN_7
#define	STCK1_PE9					  LL_GPIO_PIN_9
#define	DIR1_PE8						LL_GPIO_PIN_8

/*  SPI1 for the Board HW  */
#define	SPI1_SCK_PA5				LL_GPIO_PIN_5
#define	SPI1_MISO_PA6			  LL_GPIO_PIN_6
#define	SPI1_MOSI_PA7			  LL_GPIO_PIN_7

/*  STCP Signal for the Shift Register  */
#define	STCP_PA4						LL_GPIO_PIN_4

/*  WS2812B LED Strip  */
#define	WS2812B_PC8				  LL_GPIO_PIN_8

/*  SPI2  */
#define	SPI2_2_SCK_PB13		        LL_GPIO_PIN_13
#define	SPI2_3_MISO_PB14		      LL_GPIO_PIN_14
#define	SPI2_4_MOSI_PB15		      LL_GPIO_PIN_15
#define	SPI2_5_NSS_PB12	          LL_GPIO_PIN_12
#define	SPI2_6_GPIO_PD7			      LL_GPIO_PIN_7
#define	SPI2_7_GPIO_PB7			      LL_GPIO_PIN_7

/*  SPI3  */
#define	SPI3_2_SCK_PB3			      LL_GPIO_PIN_3
#define	SPI3_3_MISO_PB4		        LL_GPIO_PIN_4
#define	SPI3_4_MOSI_PB5		        LL_GPIO_PIN_5
#define	SPI3_5_NSS_PB6			      LL_GPIO_PIN_6
#define	SPI3_6_GPIO_PE1			      LL_GPIO_PIN_1
#define	SPI3_7_GPIO_PE0			      LL_GPIO_PIN_0

/*  I2C1  */
#define	I2C1_2_SCL_PB8				    LL_GPIO_PIN_8
#define	I2C1_3_SDA_PB9			      LL_GPIO_PIN_9
#define	I2C1_4_GPIO_PE3           LL_GPIO_PIN_3
#define	I2C1_5_GPIO_PC13          LL_GPIO_PIN_13
#define	I2C1_6_GPIO_PC14          LL_GPIO_PIN_14
#define	I2C1_7_GPIO_PE4           LL_GPIO_PIN_4

/*  I2C2  */
#define	I2C2_2_SCL_PB10			      LL_GPIO_PIN_10
#define	I2C2_3_SDA_PB11			      LL_GPIO_PIN_11
#define	I2C2_4_GPIO_PE5           LL_GPIO_PIN_5
#define	I2C2_5_GPIO_PC15          LL_GPIO_PIN_15
#define	I2C2_6_GPIO_PE6           LL_GPIO_PIN_6
#define	I2C2_7_GPIO_PE2           LL_GPIO_PIN_2

/*  PWM Outputs  */
#define	PWM1_PD15					        LL_GPIO_PIN_15
#define	PWM2_PD14					        LL_GPIO_PIN_14
#define	PWM3_PD13					        LL_GPIO_PIN_13
#define	PWM4_PD12					        LL_GPIO_PIN_12
#define	PWM5_PA0					        LL_GPIO_PIN_0
#define	PWM6_PA1					        LL_GPIO_PIN_1
#define	PWM7_PA2					        LL_GPIO_PIN_2
#define	PWM8_PA3					        LL_GPIO_PIN_3

/*  Analog Inputs  */
#define	AIN1_PC3				          LL_GPIO_PIN_3
#define	AIN2_PC2				          LL_GPIO_PIN_2
#define	AIN3_PC0				          LL_GPIO_PIN_0
#define	AIN4_PC1				          LL_GPIO_PIN_1
#define	AIN5_PC5				          LL_GPIO_PIN_5
#define	AIN6_PC4				          LL_GPIO_PIN_4
#define	AIN7_PB1				          LL_GPIO_PIN_1
#define	AIN8_PB0				          LL_GPIO_PIN_0

/*  28 GPIO HEADER DEFINES */
#define GPIO1_PD12                PWM4_PD12
#define GPIO2_PD13                PWM3_PD13
#define GPIO3_PD15                PWM1_PD15
#define GPIO4_PD14                PWM2_PD14
#define GPIO5_PA8                 LL_GPIO_PIN_8
#define GPIO6_PC11                LL_GPIO_PIN_11
#define GPIO7_PD1                 LL_GPIO_PIN_1
#define GPIO8_PD0                 LL_GPIO_PIN_0
#define GPIO9_PB9                 I2C1_3_SDA_PB9
#define GPIO10_PB8                I2C1_2_SCL_PB8
#define GPIO11_PD4                LL_GPIO_PIN_4
#define GPIO12_PD3                LL_GPIO_PIN_3 
#define GPIO13_PD7                SPI2_6_GPIO_PD7
#define GPIO14_PB3                SPI3_2_SCK_PB3

#define GPIO15_PB5                SPI3_4_MOSI_PB5
#define GPIO16_PB4                SPI3_3_MISO_PB4
#define GPIO17_PB6                SPI3_5_NSS_PB6
#define GPIO18_PB7                SPI2_7_GPIO_PB7  
#define GPIO19_PE1                SPI3_6_GPIO_PE1
#define GPIO20_PE0                SPI3_7_GPIO_PE0
#define GPIO21_PE2                I2C2_7_GPIO_PE2
#define GPIO22_PE3                I2C1_4_GPIO_PE3 
#define GPIO23_PE5                I2C2_4_GPIO_PE5
#define GPIO24_PE4                I2C1_7_GPIO_PE4 
#define GPIO25_PE6                I2C2_6_GPIO_PE6
#define GPIO26_PC13               I2C1_5_GPIO_PC13
#define GPIO27_PC15               I2C2_5_GPIO_PC15
#define GPIO28_PC14               I2C1_6_GPIO_PC14 


#ifdef __cplusplus
}
#endif

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
