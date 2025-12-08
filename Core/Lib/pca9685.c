/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pca9685.c
  * @brief   PCA9685 16‑channel PWM driver (I2C) – сервоприводы.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "pca9685.h"

/* Внутреннее состояние ------------------------------------------------------*/
static struct{
  uint8_t  i2c_addr;
  float    pwm_freq_hz;
  uint16_t pulse_min_us;
  uint16_t pulse_max_us;
  uint16_t max_angle_deg;
} pca9685_cfg = {
  .i2c_addr     = PCA9685_I2C_BASE_ADDR,
  .pwm_freq_hz  = 50.0f,
  .pulse_min_us = 500U,
  .pulse_max_us = 2500U,
  .max_angle_deg = 180U
};

/* Вспомогательные функции ---------------------------------------------------*/
static int WriteReg(uint8_t reg, uint8_t value)
{
  return PCA9685_I2C_Write(pca9685_cfg.i2c_addr, reg, &value, 1U);
}

static int ReadReg(uint8_t reg, uint8_t *value)
{
  return PCA9685_I2C_Read(pca9685_cfg.i2c_addr, reg, value, 1U);
}

static int WriteRegMulti(uint8_t reg, const uint8_t *data, uint16_t size)
{
  return PCA9685_I2C_Write(pca9685_cfg.i2c_addr, reg, data, size);
}

/* Пересчёт частоты PWM в значение прескейлера ------------------------------*/
static uint8_t CalcPrescale(float pwm_freq_hz)
{
  /* Частота генератора внутри PCA9685: 25 МГц */
  const float osc_clk_hz = 25000000.0f;
  float prescale_f = (osc_clk_hz / (4096.0f * pwm_freq_hz)) - 1.0f;

  if (prescale_f < 3.0f) prescale_f = 3.0f;
  if (prescale_f > 255.0f) prescale_f = 255.0f;

  uint8_t prescale = (uint8_t)(prescale_f + 0.5f);
  return prescale;
}

/* Перевод длительности импульса (мкс) в счётчик 0..4095 --------------------*/
static uint16_t PulseUsToTicks(uint16_t pulse_us)
{
  float period_us = 1000000.0f / pca9685_cfg.pwm_freq_hz;
  if (pulse_us > (uint16_t)period_us) pulse_us = (uint16_t)period_us;

  float ticks_f = (4096.0f * (float)pulse_us) / period_us;
  if (ticks_f < 0.0f) ticks_f = 0.0f;
  if (ticks_f > 4095.0f) ticks_f = 4095.0f;

  return (uint16_t)(ticks_f + 0.5f);
}

/* Высокоуровневые функции ---------------------------------------------------*/
static int PCA9685_Init_Impl(uint8_t i2c_addr_7bit,
                             float   pwm_freq_hz,
                             uint16_t pulse_min_us,
                             uint16_t pulse_max_us,
                             uint16_t max_angle_deg)
{
  uint8_t mode1 = 0;
  uint8_t prescale;
  int status;

  if (pwm_freq_hz < 24.0f)  pwm_freq_hz = 24.0f;
  if (pwm_freq_hz > 1526.0f) pwm_freq_hz = 1526.0f;
  if (max_angle_deg == 0U) max_angle_deg = 180U;

  pca9685_cfg.i2c_addr     = i2c_addr_7bit;
  pca9685_cfg.pwm_freq_hz  = pwm_freq_hz;
  pca9685_cfg.pulse_min_us = pulse_min_us;
  pca9685_cfg.pulse_max_us = pulse_max_us;
  pca9685_cfg.max_angle_deg = max_angle_deg;

  /* Прочитать MODE1 */
  status = ReadReg(PCA9685_REG_MODE1, &mode1);
  if (status != 0) return status;

  /* Перевести в режим SLEEP для записи прескейлера */
  uint8_t mode1_sleep = (mode1 & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
  status = WriteReg(PCA9685_REG_MODE1, mode1_sleep);
  if (status != 0) return status;

  /* Установить прескейлер */
  prescale = CalcPrescale(pwm_freq_hz);
  status = WriteReg(PCA9685_REG_PRESCALE, prescale);
  if (status != 0) return status;

  /* Выйти из SLEEP */
  status = WriteReg(PCA9685_REG_MODE1,
                    (uint8_t)((mode1_sleep & ~PCA9685_MODE1_SLEEP) |
                              PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL));
  if (status != 0) return status;

  /* MODE2: драйвер push‑pull, изменять выход по событию */
  status = WriteReg(PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV);
  if (status != 0) return status;

  /* Небольшая задержка для запуска генератора – здесь просто «холостое» чтение */
  (void)ReadReg(PCA9685_REG_MODE1, &mode1);

  return 0;
}

static int PCA9685_SetPWM_Impl(enum pca9685_channel_e ch,
                               uint16_t on,
                               uint16_t off)
{
  if (ch > PCA9685_CH15) return -1;

  uint8_t reg = (uint8_t)(PCA9685_REG_LED0_ON_L + 4U * (uint8_t)ch);
  uint8_t buf[4];

  buf[0] = (uint8_t)(on  & 0xFFU);
  buf[1] = (uint8_t)((on  >> 8) & 0x0FU);
  buf[2] = (uint8_t)(off & 0xFFU);
  buf[3] = (uint8_t)((off >> 8) & 0x0FU);

  return WriteRegMulti(reg, buf, 4U);
}

static int PCA9685_SetServoAngle_Impl(enum pca9685_channel_e ch,
                                      uint16_t angle_deg)
{
  if (ch > PCA9685_CH15) return -1;
  if (angle_deg > pca9685_cfg.max_angle_deg)
  {
    angle_deg = pca9685_cfg.max_angle_deg;
  }

  uint32_t pulse_us = pca9685_cfg.pulse_min_us +
                      (uint32_t)((((uint32_t)pca9685_cfg.pulse_max_us -
                                   (uint32_t)pca9685_cfg.pulse_min_us) *
                                  (uint32_t)angle_deg) /
                                 (uint32_t)pca9685_cfg.max_angle_deg);

  uint16_t ticks = PulseUsToTicks((uint16_t)pulse_us);

  return PCA9685_SetPWM_Impl(ch, 0U, ticks);
}

static int PCA9685_SetAllServoPulseUs_Impl(uint16_t pulse_us)
{
  uint16_t ticks = PulseUsToTicks(pulse_us);
  uint8_t buf[4];

  buf[0] = 0U; /* ON_L  */
  buf[1] = 0U; /* ON_H  */
  buf[2] = (uint8_t)(ticks & 0xFFU);
  buf[3] = (uint8_t)((ticks >> 8) & 0x0FU);

  return WriteRegMulti(PCA9685_REG_ALL_LED_ON_L, buf, 4U);
}

static int PCA9685_Sleep_Impl(void)
{
  uint8_t mode1;
  int status = ReadReg(PCA9685_REG_MODE1, &mode1);
  if (status != 0) return status;

  mode1 |= PCA9685_MODE1_SLEEP;
  return WriteReg(PCA9685_REG_MODE1, mode1);
}

static int PCA9685_Wakeup_Impl(void)
{
  uint8_t mode1;
  int status = ReadReg(PCA9685_REG_MODE1, &mode1);
  if (status != 0) return status;

  mode1 &= (uint8_t)~PCA9685_MODE1_SLEEP;
  mode1 |= PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL;
  status = WriteReg(PCA9685_REG_MODE1, mode1);
  if (status != 0) return status;

  return 0;
}

static int PCA9685_SetServoAngleWithAddr_Impl(uint8_t i2c_addr_7bit,
                                               enum pca9685_channel_e ch,
                                               uint16_t angle_deg)
{
  if (ch > PCA9685_CH15) return -1;
  if (angle_deg > pca9685_cfg.max_angle_deg)
  {
    angle_deg = pca9685_cfg.max_angle_deg;
  }

  /* Сохраняем текущий адрес */
  uint8_t old_addr = pca9685_cfg.i2c_addr;
  
  /* Временно меняем адрес */
  pca9685_cfg.i2c_addr = i2c_addr_7bit;

  /* Вычисляем длительность импульса */
  uint32_t pulse_us = pca9685_cfg.pulse_min_us +
                      (uint32_t)((((uint32_t)pca9685_cfg.pulse_max_us -
                                   (uint32_t)pca9685_cfg.pulse_min_us) *
                                  (uint32_t)angle_deg) /
                                 (uint32_t)pca9685_cfg.max_angle_deg);

  uint16_t ticks = PulseUsToTicks((uint16_t)pulse_us);

  /* Устанавливаем PWM */
  int status = PCA9685_SetPWM_Impl(ch, 0U, ticks);

  /* Восстанавливаем адрес */
  pca9685_cfg.i2c_addr = old_addr;

  return status;
}

/* Публичный объект ----------------------------------------------------------*/
struct pca9685_s PCA9685 = {
  .Init              = PCA9685_Init_Impl,
  .SetPWM            = PCA9685_SetPWM_Impl,
  .SetServoAngle     = PCA9685_SetServoAngle_Impl,
  .SetServoAngleWithAddr = PCA9685_SetServoAngleWithAddr_Impl,
  .SetAllServoPulseUs= PCA9685_SetAllServoPulseUs_Impl,
  .Sleep             = PCA9685_Sleep_Impl,
  .Wakeup            = PCA9685_Wakeup_Impl
};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




