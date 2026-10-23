/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pca9685.c
  * @brief   PCA9685 16‑channel PWM driver (I2C) – сервоприводы.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "pca9685.hpp"


namespace {

/* Частота генератора внутри PCA9685: 25 МГц */
constexpr float OSC_CLK_HZ = 25000000.0f;

/* Пересчёт частоты PWM в значение прескейлера ------------------------------*/
uint8_t CalcPrescale(float pwm_freq_hz)
{
  float prescale_f = (OSC_CLK_HZ / (4096.0f * pwm_freq_hz)) - 1.0f;

  if (prescale_f < 3.0f) prescale_f = 3.0f;
  if (prescale_f > 255.0f) prescale_f = 255.0f;

  uint8_t prescale = (uint8_t)(prescale_f + 0.5f);
  return prescale;
}

}

constinit PCA9685_Driver PCA9685;


/* Вспомогательные функции ---------------------------------------------------*/
int PCA9685_Driver::WriteReg(uint8_t reg, uint8_t value)
{
  return I2C2_Bus.Write(i2c_addr, reg, &value, 1U);
}

int PCA9685_Driver::ReadReg(uint8_t reg, uint8_t *value)
{
  return I2C2_Bus.Read(i2c_addr, reg, value, 1U);
}

/* Перевод длительности импульса (мкс) в счётчик 0..4095 --------------------*/
uint16_t PCA9685_Driver::PulseUsToTicks(uint16_t pulse_us) const
{
  float period_us = 1000000.0f / pwm_freq_hz;
  if (pulse_us > (uint16_t)period_us) pulse_us = (uint16_t)period_us;

  float ticks_f = (4096.0f * (float)pulse_us) / period_us;
  if (ticks_f < 0.0f) ticks_f = 0.0f;
  if (ticks_f > 4095.0f) ticks_f = 4095.0f;

  return (uint16_t)(ticks_f + 0.5f);
}

/* Угол (с ограничением max_angle_deg) -> счётчик 0..4095 --------------------*/
uint16_t PCA9685_Driver::AngleToTicks(uint16_t angle_deg) const
{
  if (angle_deg > max_angle_deg)
  {
    angle_deg = max_angle_deg;
  }

  uint32_t pulse_us = pulse_min_us +
                      (uint32_t)((((uint32_t)pulse_max_us -
                                   (uint32_t)pulse_min_us) *
                                  (uint32_t)angle_deg) /
                                 (uint32_t)max_angle_deg);

  return PulseUsToTicks((uint16_t)pulse_us);
}

int PCA9685_Driver::SetChannelPWM(uint8_t i2c_addr_7bit, pca9685_channel_e ch, uint16_t on, uint16_t off)
{
  if (ch > PCA9685_CH15) return -1;

  /* Вычисляем адрес регистра для конкретного канала */
  /* LED0_ON_L = 0x06, каждый канал занимает 4 регистра */
  uint8_t reg = (uint8_t)(PCA9685_REG_LED0_ON_L + 4U * (uint8_t)ch);
  
  /* Защита: убеждаемся, что не попадаем в регистры ALL_LED (0xFA-0xFD) */
  if (reg >= PCA9685_REG_ALL_LED_ON_L) return -1;
  
  uint8_t buf[4];

  buf[0] = (uint8_t)(on  & 0xFFU);
  buf[1] = (uint8_t)((on  >> 8) & 0x0FU);
  buf[2] = (uint8_t)(off & 0xFFU);
  buf[3] = (uint8_t)((off >> 8) & 0x0FU);

  return I2C2_Bus.Write(i2c_addr_7bit, reg, buf, 4U);
}

/* Высокоуровневые функции ---------------------------------------------------*/
int PCA9685_Driver::Init(uint8_t i2c_addr_7bit,
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

  this->i2c_addr      = i2c_addr_7bit;
  this->pwm_freq_hz   = pwm_freq_hz;
  this->pulse_min_us  = pulse_min_us;
  this->pulse_max_us  = pulse_max_us;
  this->max_angle_deg = max_angle_deg;

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

  /* Сбросить все каналы в 0 (выключить все выходы) */
  uint8_t all_off[4] = {0U, 0U, 0U, 0U};
  status = I2C2_Bus.Write(i2c_addr, PCA9685_REG_ALL_LED_ON_L, all_off, 4U);
  if (status != 0) return status;

  /* Небольшая задержка для запуска генератора – здесь просто «холостое» чтение */
  (void)ReadReg(PCA9685_REG_MODE1, &mode1);

  return 0;
}

int PCA9685_Driver::SetPWM(pca9685_channel_e ch, uint16_t on, uint16_t off)
{
  return SetChannelPWM(i2c_addr, ch, on, off);
}

int PCA9685_Driver::SetServoAngle(pca9685_channel_e ch, uint16_t angle_deg)
{
  return SetServoAngleWithAddr(i2c_addr, ch, angle_deg);
}

int PCA9685_Driver::SetServoAngleWithAddr(uint8_t i2c_addr_7bit, pca9685_channel_e ch, uint16_t angle_deg)
{
  if (ch > PCA9685_CH15) return -1;

  return SetChannelPWM(i2c_addr_7bit, ch, 0U, AngleToTicks(angle_deg));
}

int PCA9685_Driver::SetAllServoPulseUs(uint16_t pulse_us)
{
  uint16_t ticks = PulseUsToTicks(pulse_us);
  uint8_t buf[4];

  buf[0] = 0U; /* ON_L  */
  buf[1] = 0U; /* ON_H  */
  buf[2] = (uint8_t)(ticks & 0xFFU);
  buf[3] = (uint8_t)((ticks >> 8) & 0x0FU);

  return I2C2_Bus.Write(i2c_addr, PCA9685_REG_ALL_LED_ON_L, buf, 4U);
}

int PCA9685_Driver::Sleep()
{
  uint8_t mode1;
  int status = ReadReg(PCA9685_REG_MODE1, &mode1);
  if (status != 0) return status;

  mode1 |= PCA9685_MODE1_SLEEP;
  return WriteReg(PCA9685_REG_MODE1, mode1);
}

int PCA9685_Driver::Wakeup()
{
  uint8_t mode1;
  int status = ReadReg(PCA9685_REG_MODE1, &mode1);
  if (status != 0) return status;

  mode1 &= (uint8_t)~PCA9685_MODE1_SLEEP;
  mode1 |= PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL;
  return WriteReg(PCA9685_REG_MODE1, mode1);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
