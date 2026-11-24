/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c_master.h
  * @brief   Blocking I2C master register access with bounded waits (I2C1, I2C2).
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __I2C_MASTER_H
#define __I2C_MASTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_ll_i2c.h"

enum
{
  I2C_MASTER_OK = 0,
  I2C_MASTER_ERROR_PARAM = -1,
  I2C_MASTER_ERROR_TIMEOUT = -2,
  I2C_MASTER_ERROR_NACK = -3,
  I2C_MASTER_ERROR_BUS = -4,
  I2C_MASTER_ERROR_ARBITRATION = -5,
  I2C_MASTER_ERROR_OVERRUN = -6
};

/*  Write size bytes starting at register reg, or read them. 0 on success, I2C_MASTER_ERROR_* otherwise  */
int I2C_Master_Write(I2C_TypeDef* I2Cx, uint8_t dev_addr_7bit, uint8_t reg, const uint8_t *data, uint16_t size);
int I2C_Master_Read(I2C_TypeDef* I2Cx, uint8_t dev_addr_7bit, uint8_t reg, uint8_t *data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
