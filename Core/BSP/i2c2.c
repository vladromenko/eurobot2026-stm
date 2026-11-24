/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c2.c
  * @brief   PCA9685 bus access on I2C2.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "pca9685.h"
#include "i2c_master.h"

int PCA9685_I2C_Write(uint8_t dev_addr_7bit, uint8_t reg,
                      const uint8_t *data, uint16_t size)
{
  return I2C_Master_Write(I2C2, dev_addr_7bit, reg, data, size);
}

int PCA9685_I2C_Read(uint8_t dev_addr_7bit, uint8_t reg,
                     uint8_t *data, uint16_t size)
{
  return I2C_Master_Read(I2C2, dev_addr_7bit, reg, data, size);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
