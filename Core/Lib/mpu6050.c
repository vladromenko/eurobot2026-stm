/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU-6050 accelerometer and gyroscope (I2C).
  ******************************************************************************
  */
/* USER CODE END Header */

#include "mpu6050.h"
#include "i2c_master.h"
#include "FreeRTOS.h"
#include "task.h"

#define MPU6050_REG_SMPLRT_DIV      0x19U
#define MPU6050_REG_CONFIG          0x1AU
#define MPU6050_REG_GYRO_CONFIG     0x1BU
#define MPU6050_REG_ACCEL_CONFIG    0x1CU
#define MPU6050_REG_ACCEL_XOUT_H    0x3BU   /* accel X/Y/Z, temperature, gyro X/Y/Z: 14 bytes, big-endian */
#define MPU6050_REG_PWR_MGMT_1      0x6BU
#define MPU6050_REG_WHO_AM_I        0x75U

#define MPU6050_PWR_RESET           0x80U
#define MPU6050_PWR_CLK_PLL_GYRO_X  0x01U   /* wake up, PLL with the X gyro as the clock */
#define MPU6050_SMPLRT_100HZ        9U      /* 1 kHz with the low-pass filter on / (1 + 9) */
#define MPU6050_DLPF_44HZ           0x03U
#define MPU6050_GYRO_FS_500         0x08U
#define MPU6050_ACCEL_FS_4G         0x08U

#define MPU6050_ACCEL_LSB_PER_G     8192.0f
#define MPU6050_GYRO_LSB_PER_DPS    65.5f
#define MPU6050_TEMP_LSB_PER_C      340.0f
#define MPU6050_TEMP_OFFSET_C       36.53f

static I2C_TypeDef* bus;
static uint8_t addr;
static uint8_t who_am_i;


static int WriteReg(uint8_t reg, uint8_t value)
{
  return I2C_Master_Write(bus, addr, reg, &value, 1U);
}

static int16_t ToInt16(const uint8_t* bytes)
{
  return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

static int Init(I2C_TypeDef* I2Cx, uint8_t addr_7bit)
{
  int status;

  bus = I2Cx;
  addr = addr_7bit;

  status = I2C_Master_Read(bus, addr, MPU6050_REG_WHO_AM_I, &who_am_i, 1U);
  if (status != I2C_MASTER_OK) return status;

  status = WriteReg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_RESET);
  if (status != I2C_MASTER_OK) return status;
  vTaskDelay(pdMS_TO_TICKS(100));

  status = WriteReg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_CLK_PLL_GYRO_X);
  if (status != I2C_MASTER_OK) return status;
  status = WriteReg(MPU6050_REG_SMPLRT_DIV, MPU6050_SMPLRT_100HZ);
  if (status != I2C_MASTER_OK) return status;
  status = WriteReg(MPU6050_REG_CONFIG, MPU6050_DLPF_44HZ);
  if (status != I2C_MASTER_OK) return status;
  status = WriteReg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_500);
  if (status != I2C_MASTER_OK) return status;
  return WriteReg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_4G);
}

static uint8_t GetWhoAmI(void)
{
  return who_am_i;
}

static int Read(struct mpu6050_data_s* data)
{
  uint8_t raw[14];
  int status = I2C_Master_Read(bus, addr, MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw));

  if (status != I2C_MASTER_OK) return status;

  data->ax = ToInt16(&raw[0]) / MPU6050_ACCEL_LSB_PER_G;
  data->ay = ToInt16(&raw[2]) / MPU6050_ACCEL_LSB_PER_G;
  data->az = ToInt16(&raw[4]) / MPU6050_ACCEL_LSB_PER_G;
  data->temp_c = ToInt16(&raw[6]) / MPU6050_TEMP_LSB_PER_C + MPU6050_TEMP_OFFSET_C;
  data->gx = ToInt16(&raw[8]) / MPU6050_GYRO_LSB_PER_DPS;
  data->gy = ToInt16(&raw[10]) / MPU6050_GYRO_LSB_PER_DPS;
  data->gz = ToInt16(&raw[12]) / MPU6050_GYRO_LSB_PER_DPS;
  return I2C_MASTER_OK;
}

const struct mpu6050_s MPU6050 = {.Init = Init,
                                  .GetWhoAmI = GetWhoAmI,
                                  .Read = Read};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
