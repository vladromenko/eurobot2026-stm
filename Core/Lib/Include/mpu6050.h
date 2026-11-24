/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   MPU-6050 accelerometer and gyroscope (I2C).
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stm32f4xx_ll_i2c.h"

#define MPU6050_ADDR_AD0_LOW        0x68U   /* AD0 pin to GND (default on GY-521) */
#define MPU6050_ADDR_AD0_HIGH       0x69U

struct mpu6050_data_s{
  float ax, ay, az;     /* acceleration, g (+-4 g range) */
  float gx, gy, gz;     /* angular rate, deg/s (+-500 deg/s range) */
  float temp_c;         /* die temperature, deg C */
};

struct mpu6050_s{
  /*  Wakes the sensor up and sets 100 Hz sampling, 44 Hz low-pass filter, +-4 g, +-500 deg/s.
      Blocks for about 100 ms (reset), call from a task. 0 on success, I2C_MASTER_ERROR_* otherwise  */
  int (*Init)(I2C_TypeDef* I2Cx, uint8_t addr_7bit);
  /*  WHO_AM_I register: 0x68 for MPU-6050, clones may answer 0x70..0x73 or 0x98  */
  uint8_t (*GetWhoAmI)(void);
  /*  Reads all axes and the temperature in one transaction. 0 on success  */
  int (*Read)(struct mpu6050_data_s* data);
};

extern const struct mpu6050_s MPU6050;

#ifdef __cplusplus
}
#endif

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
