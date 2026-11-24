/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c_master.c
  * @brief   Blocking I2C master register access with bounded waits (I2C1, I2C2).
  ******************************************************************************
  */
/* USER CODE END Header */

#include "i2c_master.h"
#include "FreeRTOS.h"
#include "task.h"

#define I2C_MASTER_TIMEOUT_MS       20U
#define I2C_MASTER_MAX_POLL_COUNT   5000000U

static uint32_t TimeoutStart(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  return DWT->CYCCNT;
}

static int CheckError(I2C_TypeDef* I2Cx)
{
  if (LL_I2C_IsActiveFlag_AF(I2Cx))
  {
    LL_I2C_ClearFlag_AF(I2Cx);
    return I2C_MASTER_ERROR_NACK;
  }
  if (LL_I2C_IsActiveFlag_BERR(I2Cx))
  {
    LL_I2C_ClearFlag_BERR(I2Cx);
    return I2C_MASTER_ERROR_BUS;
  }
  if (LL_I2C_IsActiveFlag_ARLO(I2Cx))
  {
    LL_I2C_ClearFlag_ARLO(I2Cx);
    return I2C_MASTER_ERROR_ARBITRATION;
  }
  if (LL_I2C_IsActiveFlag_OVR(I2Cx))
  {
    LL_I2C_ClearFlag_OVR(I2Cx);
    return I2C_MASTER_ERROR_OVERRUN;
  }
  return I2C_MASTER_OK;
}

static int WaitSR1Set(I2C_TypeDef* I2Cx, uint32_t flag)
{
  const uint32_t start = TimeoutStart();
  const uint32_t timeout_cycles = SystemCoreClock / (1000U / I2C_MASTER_TIMEOUT_MS);
  uint32_t polls = 0U;

  while ((I2Cx->SR1 & flag) == 0U)
  {
    int status = CheckError(I2Cx);
    if (status != I2C_MASTER_OK)
    {
      return status;
    }
    if (((uint32_t)(DWT->CYCCNT - start) >= timeout_cycles) ||
        (++polls >= I2C_MASTER_MAX_POLL_COUNT))
    {
      return I2C_MASTER_ERROR_TIMEOUT;
    }
  }
  return I2C_MASTER_OK;
}

static int WaitBusFree(I2C_TypeDef* I2Cx)
{
  const uint32_t start = TimeoutStart();
  const uint32_t timeout_cycles = SystemCoreClock / (1000U / I2C_MASTER_TIMEOUT_MS);
  uint32_t polls = 0U;

  while (LL_I2C_IsActiveFlag_BUSY(I2Cx))
  {
    int status = CheckError(I2Cx);
    if (status != I2C_MASTER_OK)
    {
      return status;
    }
    if (((uint32_t)(DWT->CYCCNT - start) >= timeout_cycles) ||
        (++polls >= I2C_MASTER_MAX_POLL_COUNT))
    {
      return I2C_MASTER_ERROR_TIMEOUT;
    }
  }
  return I2C_MASTER_OK;
}

static void FinishTransaction(I2C_TypeDef* I2Cx)
{
  if (LL_I2C_IsActiveFlag_MSL(I2Cx))
  {
    LL_I2C_GenerateStopCondition(I2Cx);
  }
  LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
  LL_I2C_DisableBitPOS(I2Cx);
}

static int StartAndAddress(I2C_TypeDef* I2Cx, uint8_t address_byte)
{
  int status;

  LL_I2C_GenerateStartCondition(I2Cx);
  status = WaitSR1Set(I2Cx, I2C_SR1_SB);
  if (status != I2C_MASTER_OK) return status;

  LL_I2C_TransmitData8(I2Cx, address_byte);
  return WaitSR1Set(I2Cx, I2C_SR1_ADDR);
}

static int WriteRegs(I2C_TypeDef* I2Cx, uint8_t dev_addr_7bit, uint8_t reg,
                     const uint8_t *data, uint16_t size)
{
  int status;

  if (((data == 0) && (size != 0U)) || (dev_addr_7bit > 0x7FU))
  {
    return I2C_MASTER_ERROR_PARAM;
  }

  status = WaitBusFree(I2Cx);
  if (status != I2C_MASTER_OK) goto fail;

  status = StartAndAddress(I2Cx, (uint8_t)(dev_addr_7bit << 1));
  if (status != I2C_MASTER_OK) goto fail;
  (void)I2Cx->SR2;

  status = WaitSR1Set(I2Cx, I2C_SR1_TXE);
  if (status != I2C_MASTER_OK) goto fail;
  LL_I2C_TransmitData8(I2Cx, reg);

  while (size > 0U)
  {
    status = WaitSR1Set(I2Cx, I2C_SR1_TXE);
    if (status != I2C_MASTER_OK) goto fail;
    LL_I2C_TransmitData8(I2Cx, *data++);
    size--;
  }

  status = WaitSR1Set(I2Cx, I2C_SR1_BTF);
  if (status != I2C_MASTER_OK) goto fail;
  LL_I2C_GenerateStopCondition(I2Cx);
  return I2C_MASTER_OK;

fail:
  FinishTransaction(I2Cx);
  return status;
}

static int ReadRegs(I2C_TypeDef* I2Cx, uint8_t dev_addr_7bit, uint8_t reg,
                    uint8_t *data, uint16_t size)
{
  int status;

  if ((data == 0) || (size == 0U) || (dev_addr_7bit > 0x7FU))
  {
    return I2C_MASTER_ERROR_PARAM;
  }

  LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
  LL_I2C_DisableBitPOS(I2Cx);

  status = WaitBusFree(I2Cx);
  if (status != I2C_MASTER_OK) goto fail;

  status = StartAndAddress(I2Cx, (uint8_t)(dev_addr_7bit << 1));
  if (status != I2C_MASTER_OK) goto fail;
  (void)I2Cx->SR2;

  status = WaitSR1Set(I2Cx, I2C_SR1_TXE);
  if (status != I2C_MASTER_OK) goto fail;
  LL_I2C_TransmitData8(I2Cx, reg);

  status = WaitSR1Set(I2Cx, I2C_SR1_BTF);
  if (status != I2C_MASTER_OK) goto fail;

  status = StartAndAddress(I2Cx, (uint8_t)((dev_addr_7bit << 1) | 1U));
  if (status != I2C_MASTER_OK) goto fail;

  if (size == 1U)
  {
    LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
    (void)I2Cx->SR2;
    LL_I2C_GenerateStopCondition(I2Cx);

    status = WaitSR1Set(I2Cx, I2C_SR1_RXNE);
    if (status != I2C_MASTER_OK) goto fail;
    *data = LL_I2C_ReceiveData8(I2Cx);
  }
  else if (size == 2U)
  {
    LL_I2C_EnableBitPOS(I2Cx);
    LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
    (void)I2Cx->SR2;

    status = WaitSR1Set(I2Cx, I2C_SR1_BTF);
    if (status != I2C_MASTER_OK) goto fail;
    LL_I2C_GenerateStopCondition(I2Cx);
    *data++ = LL_I2C_ReceiveData8(I2Cx);
    *data = LL_I2C_ReceiveData8(I2Cx);
  }
  else
  {
    (void)I2Cx->SR2;

    while (size > 3U)
    {
      status = WaitSR1Set(I2Cx, I2C_SR1_RXNE);
      if (status != I2C_MASTER_OK) goto fail;
      *data++ = LL_I2C_ReceiveData8(I2Cx);
      size--;
    }

    status = WaitSR1Set(I2Cx, I2C_SR1_BTF);
    if (status != I2C_MASTER_OK) goto fail;
    LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
    *data++ = LL_I2C_ReceiveData8(I2Cx);
    size--;
    LL_I2C_GenerateStopCondition(I2Cx);
    *data++ = LL_I2C_ReceiveData8(I2Cx);
    size--;

    status = WaitSR1Set(I2Cx, I2C_SR1_RXNE);
    if (status != I2C_MASTER_OK) goto fail;
    *data = LL_I2C_ReceiveData8(I2Cx);
  }

  LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
  LL_I2C_DisableBitPOS(I2Cx);
  return I2C_MASTER_OK;

fail:
  FinishTransaction(I2Cx);
  return status;
}

/* Waits are bounded by DWT cycles, not by ticks, so a transaction is safe with the scheduler
   suspended. Suspending keeps the timing of the 1/2-byte read sequences and lets tasks share a bus. */
int I2C_Master_Write(I2C_TypeDef* I2Cx, uint8_t dev_addr_7bit, uint8_t reg,
                     const uint8_t *data, uint16_t size)
{
  int status;

  vTaskSuspendAll();
  status = WriteRegs(I2Cx, dev_addr_7bit, reg, data, size);
  xTaskResumeAll();
  return status;
}

int I2C_Master_Read(I2C_TypeDef* I2Cx, uint8_t dev_addr_7bit, uint8_t reg,
                    uint8_t *data, uint16_t size)
{
  int status;

  vTaskSuspendAll();
  status = ReadRegs(I2Cx, dev_addr_7bit, reg, data, size);
  xTaskResumeAll();
  return status;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
