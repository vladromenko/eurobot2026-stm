/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c2.c
  * @brief   Blocking I2C2 functions for PCA9685 with bounded waits.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "i2c2.hpp"
#include "stm32f4xx_ll_i2c.h"

namespace {

constexpr uint32_t PCA9685_I2C_TIMEOUT_MS = 20U;
constexpr uint32_t PCA9685_I2C_MAX_POLL_COUNT = 5000000U;

enum
{
  PCA9685_I2C_OK = 0,
  PCA9685_I2C_ERROR_PARAM = -1,
  PCA9685_I2C_ERROR_TIMEOUT = -2,
  PCA9685_I2C_ERROR_NACK = -3,
  PCA9685_I2C_ERROR_BUS = -4,
  PCA9685_I2C_ERROR_ARBITRATION = -5,
  PCA9685_I2C_ERROR_OVERRUN = -6
};

uint32_t I2C2_TimeoutStart(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  return DWT->CYCCNT;
}

int I2C2_CheckError(void)
{
  if (LL_I2C_IsActiveFlag_AF(I2C2))
  {
    LL_I2C_ClearFlag_AF(I2C2);
    return PCA9685_I2C_ERROR_NACK;
  }
  if (LL_I2C_IsActiveFlag_BERR(I2C2))
  {
    LL_I2C_ClearFlag_BERR(I2C2);
    return PCA9685_I2C_ERROR_BUS;
  }
  if (LL_I2C_IsActiveFlag_ARLO(I2C2))
  {
    LL_I2C_ClearFlag_ARLO(I2C2);
    return PCA9685_I2C_ERROR_ARBITRATION;
  }
  if (LL_I2C_IsActiveFlag_OVR(I2C2))
  {
    LL_I2C_ClearFlag_OVR(I2C2);
    return PCA9685_I2C_ERROR_OVERRUN;
  }
  return PCA9685_I2C_OK;
}

int I2C2_WaitSR1Set(uint32_t flag)
{
  const uint32_t start = I2C2_TimeoutStart();
  const uint32_t timeout_cycles = SystemCoreClock / (1000U / PCA9685_I2C_TIMEOUT_MS);
  uint32_t polls = 0U;

  while ((I2C2->SR1 & flag) == 0U)
  {
    int status = I2C2_CheckError();
    if (status != PCA9685_I2C_OK)
    {
      return status;
    }
    if (((uint32_t)(DWT->CYCCNT - start) >= timeout_cycles) ||
        (++polls >= PCA9685_I2C_MAX_POLL_COUNT))
    {
      return PCA9685_I2C_ERROR_TIMEOUT;
    }
  }
  return PCA9685_I2C_OK;
}

int I2C2_WaitBusFree(void)
{
  const uint32_t start = I2C2_TimeoutStart();
  const uint32_t timeout_cycles = SystemCoreClock / (1000U / PCA9685_I2C_TIMEOUT_MS);
  uint32_t polls = 0U;

  while (LL_I2C_IsActiveFlag_BUSY(I2C2))
  {
    int status = I2C2_CheckError();
    if (status != PCA9685_I2C_OK)
    {
      return status;
    }
    if (((uint32_t)(DWT->CYCCNT - start) >= timeout_cycles) ||
        (++polls >= PCA9685_I2C_MAX_POLL_COUNT))
    {
      return PCA9685_I2C_ERROR_TIMEOUT;
    }
  }
  return PCA9685_I2C_OK;
}

void I2C2_FinishTransaction(void)
{
  if (LL_I2C_IsActiveFlag_MSL(I2C2))
  {
    LL_I2C_GenerateStopCondition(I2C2);
  }
  LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);
  LL_I2C_DisableBitPOS(I2C2);
}

int I2C2_StartAndAddress(uint8_t address_byte)
{
  int status;

  LL_I2C_GenerateStartCondition(I2C2);
  status = I2C2_WaitSR1Set(I2C_SR1_SB);
  if (status != PCA9685_I2C_OK) return status;

  LL_I2C_TransmitData8(I2C2, address_byte);
  return I2C2_WaitSR1Set(I2C_SR1_ADDR);
}

}

constinit I2C2_Driver I2C2_Bus;

int I2C2_Driver::Write(uint8_t dev_addr_7bit, uint8_t reg,
                       const uint8_t *data, uint16_t size)
{
  int status;

  if (((data == nullptr) && (size != 0U)) || (dev_addr_7bit > 0x7FU))
  {
    return PCA9685_I2C_ERROR_PARAM;
  }

  status = I2C2_WaitBusFree();
  if (status != PCA9685_I2C_OK) goto fail;

  status = I2C2_StartAndAddress((uint8_t)(dev_addr_7bit << 1));
  if (status != PCA9685_I2C_OK) goto fail;
  (void)I2C2->SR2;

  status = I2C2_WaitSR1Set(I2C_SR1_TXE);
  if (status != PCA9685_I2C_OK) goto fail;
  LL_I2C_TransmitData8(I2C2, reg);

  while (size > 0U)
  {
    status = I2C2_WaitSR1Set(I2C_SR1_TXE);
    if (status != PCA9685_I2C_OK) goto fail;
    LL_I2C_TransmitData8(I2C2, *data++);
    size--;
  }

  status = I2C2_WaitSR1Set(I2C_SR1_BTF);
  if (status != PCA9685_I2C_OK) goto fail;
  LL_I2C_GenerateStopCondition(I2C2);
  return PCA9685_I2C_OK;

fail:
  I2C2_FinishTransaction();
  return status;
}

int I2C2_Driver::Read(uint8_t dev_addr_7bit, uint8_t reg,
                      uint8_t *data, uint16_t size)
{
  int status;

  if ((data == nullptr) || (size == 0U) || (dev_addr_7bit > 0x7FU))
  {
    return PCA9685_I2C_ERROR_PARAM;
  }

  LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);
  LL_I2C_DisableBitPOS(I2C2);

  status = I2C2_WaitBusFree();
  if (status != PCA9685_I2C_OK) goto fail;

  status = I2C2_StartAndAddress((uint8_t)(dev_addr_7bit << 1));
  if (status != PCA9685_I2C_OK) goto fail;
  (void)I2C2->SR2;

  status = I2C2_WaitSR1Set(I2C_SR1_TXE);
  if (status != PCA9685_I2C_OK) goto fail;
  LL_I2C_TransmitData8(I2C2, reg);

  status = I2C2_WaitSR1Set(I2C_SR1_BTF);
  if (status != PCA9685_I2C_OK) goto fail;

  status = I2C2_StartAndAddress((uint8_t)((dev_addr_7bit << 1) | 1U));
  if (status != PCA9685_I2C_OK) goto fail;

  if (size == 1U)
  {
    LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_NACK);
    (void)I2C2->SR2;
    LL_I2C_GenerateStopCondition(I2C2);

    status = I2C2_WaitSR1Set(I2C_SR1_RXNE);
    if (status != PCA9685_I2C_OK) goto fail;
    *data = LL_I2C_ReceiveData8(I2C2);
  }
  else if (size == 2U)
  {
    LL_I2C_EnableBitPOS(I2C2);
    LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_NACK);
    (void)I2C2->SR2;

    status = I2C2_WaitSR1Set(I2C_SR1_BTF);
    if (status != PCA9685_I2C_OK) goto fail;
    LL_I2C_GenerateStopCondition(I2C2);
    *data++ = LL_I2C_ReceiveData8(I2C2);
    *data = LL_I2C_ReceiveData8(I2C2);
  }
  else
  {
    (void)I2C2->SR2;

    while (size > 3U)
    {
      status = I2C2_WaitSR1Set(I2C_SR1_RXNE);
      if (status != PCA9685_I2C_OK) goto fail;
      *data++ = LL_I2C_ReceiveData8(I2C2);
      size--;
    }

    status = I2C2_WaitSR1Set(I2C_SR1_BTF);
    if (status != PCA9685_I2C_OK) goto fail;
    LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_NACK);
    *data++ = LL_I2C_ReceiveData8(I2C2);
    size--;
    LL_I2C_GenerateStopCondition(I2C2);
    *data++ = LL_I2C_ReceiveData8(I2C2);
    size--;

    status = I2C2_WaitSR1Set(I2C_SR1_RXNE);
    if (status != PCA9685_I2C_OK) goto fail;
    *data = LL_I2C_ReceiveData8(I2C2);
  }

  LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);
  LL_I2C_DisableBitPOS(I2C2);
  return PCA9685_I2C_OK;

fail:
  I2C2_FinishTransaction();
  return status;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
