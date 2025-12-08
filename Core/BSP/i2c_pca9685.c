/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c_pca9685.c
  * @brief   Блокирующие функции I2C2 для работы с PCA9685.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "pca9685.h"
#include "stm32f4xx_ll_i2c.h"

/* Простейшая блокирующая передача по I2C2 (Master, 7‑битный адрес) */

int PCA9685_I2C_Write(uint8_t dev_addr_7bit, uint8_t reg,
                      const uint8_t *data, uint16_t size)
{
  if (data == 0 && size != 0U) return -1;

  /* START + адрес + запись */
  while (LL_I2C_IsActiveFlag_BUSY(I2C2));

  LL_I2C_GenerateStartCondition(I2C2);
  while (!LL_I2C_IsActiveFlag_SB(I2C2));

  LL_I2C_TransmitData8(I2C2, (uint8_t)((dev_addr_7bit << 1) | 0U));
  while (!LL_I2C_IsActiveFlag_ADDR(I2C2));
  (void)I2C2->SR2;

  /* Первый байт – адрес регистра */
  while (!LL_I2C_IsActiveFlag_TXE(I2C2));
  LL_I2C_TransmitData8(I2C2, reg);

  /* Основные данные */
  while (size > 0U)
  {
    while (!LL_I2C_IsActiveFlag_TXE(I2C2));
    LL_I2C_TransmitData8(I2C2, *data++);
    size--;
  }

  while (!LL_I2C_IsActiveFlag_BTF(I2C2));

  LL_I2C_GenerateStopCondition(I2C2);

  return 0;
}

int PCA9685_I2C_Read(uint8_t dev_addr_7bit, uint8_t reg,
                     uint8_t *data, uint16_t size)
{
  if ((data == 0) || (size == 0U)) return -1;

  /* Записать адрес регистра (без STOP) */
  while (LL_I2C_IsActiveFlag_BUSY(I2C2));

  LL_I2C_GenerateStartCondition(I2C2);
  while (!LL_I2C_IsActiveFlag_SB(I2C2));

  LL_I2C_TransmitData8(I2C2, (uint8_t)((dev_addr_7bit << 1) | 0U));
  while (!LL_I2C_IsActiveFlag_ADDR(I2C2));
  (void)I2C2->SR2;

  while (!LL_I2C_IsActiveFlag_TXE(I2C2));
  LL_I2C_TransmitData8(I2C2, reg);
  while (!LL_I2C_IsActiveFlag_TXE(I2C2));
  while (!LL_I2C_IsActiveFlag_BTF(I2C2));

  /* Повторный START + чтение */
  LL_I2C_GenerateStartCondition(I2C2);
  while (!LL_I2C_IsActiveFlag_SB(I2C2));

  LL_I2C_TransmitData8(I2C2, (uint8_t)((dev_addr_7bit << 1) | 1U));
  while (!LL_I2C_IsActiveFlag_ADDR(I2C2));

  if (size == 1U)
  {
    LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_NACK);
    (void)I2C2->SR2;
    LL_I2C_GenerateStopCondition(I2C2);

    while (!LL_I2C_IsActiveFlag_RXNE(I2C2));
    *data = LL_I2C_ReceiveData8(I2C2);
  }
  else
  {
    (void)I2C2->SR2;

    while (size > 0U)
    {
      if (size == 1U)
      {
        LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_NACK);
        LL_I2C_GenerateStopCondition(I2C2);
      }

      while (!LL_I2C_IsActiveFlag_RXNE(I2C2));
      *data++ = LL_I2C_ReceiveData8(I2C2);
      size--;
    }
  }

  return 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
