/**
  ******************************************************************************
  * @file    	GPIOs_Defines.h
  * @brief   HAL configuration file.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_GPIO_DEFINES_H
#define __SPI_GPIO_DEFINES_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "std_def.hpp"

#define	SPI_POWER_OUTPUT_1				(uint32_t)(1<<20)
#define	SPI_POWER_OUTPUT_2				(uint32_t)(1<<21)
#define	SPI_POWER_OUTPUT_3				(uint32_t)(1<<18)
#define	SPI_POWER_OUTPUT_4				(uint32_t)(1<<19)
#define	SPI_POWER_OUTPUT_5				(uint32_t)(1<<22)
#define	SPI_POWER_OUTPUT_6				(uint32_t)(1<<23)
#define	SPI_POWER_OUTPUT_7				(uint32_t)(1<<10)
#define	SPI_POWER_OUTPUT_8				(uint32_t)(1<<11)
#define	SPI_POWER_OUTPUT_9				(uint32_t)(1<<14)
#define	SPI_POWER_OUTPUT_10				(uint32_t)(1<<15)
#define	SPI_POWER_OUTPUT_11				(uint32_t)(1<<12)
#define	SPI_POWER_OUTPUT_12				(uint32_t)(1<<13)
#define	SPI_POWER_OUTPUT_13				(uint32_t)(1<<6)
#define	SPI_POWER_OUTPUT_14				(uint32_t)(1<<7)
#define	SPI_POWER_OUTPUT_15				(uint32_t)(1<<4)
#define	SPI_POWER_OUTPUT_16				(uint32_t)(1<<5)
#define	SPI_POWER_OUTPUT_17				(uint32_t)(1<<2)
#define	SPI_POWER_OUTPUT_18				(uint32_t)(1<<3)
#define	SPI_POWER_OUTPUT_19				(uint32_t)(1<<1)
#define	SPI_POWER_OUTPUT_20				(uint32_t)(1<<0)

#define	SPI_L6474_NRST1						(uint32_t)(1<<8)
#define	SPI_L6474_NRST2						(uint32_t)(1<<9)
#define	SPI_L6474_NRST3						(uint32_t)(1<<17)
#define	SPI_L6474_NRST4						(uint32_t)(1<<16)

#define	SPI_GPO_ALL						    (uint32_t)0xFFFFFF


#ifdef __cplusplus
}
#endif

#endif /* __STM32F0xx_HAL_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
