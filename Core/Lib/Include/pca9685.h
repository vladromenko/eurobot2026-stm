/**
  ******************************************************************************
  * @file    pca9685.h
  * @brief   PCA9685 16‑channel PWM driver (I2C) – сервоприводы.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PCA9685_H
#define __PCA9685_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"

/* Стандартный 7‑битный базовый адрес PCA9685 (A5..A0 = 0) */
#define PCA9685_I2C_BASE_ADDR          0x40U

/* Регистры PCA9685 ---------------------------------------------------------*/
#define PCA9685_REG_MODE1              0x00U
#define PCA9685_REG_MODE2              0x01U
#define PCA9685_REG_SUBADR1            0x02U
#define PCA9685_REG_SUBADR2            0x03U
#define PCA9685_REG_SUBADR3            0x04U
#define PCA9685_REG_ALLCALLADR         0x05U

#define PCA9685_REG_LED0_ON_L          0x06U
#define PCA9685_REG_LED0_ON_H          0x07U
#define PCA9685_REG_LED0_OFF_L         0x08U
#define PCA9685_REG_LED0_OFF_H         0x09U

#define PCA9685_REG_ALL_LED_ON_L       0xFAU
#define PCA9685_REG_ALL_LED_ON_H       0xFBU
#define PCA9685_REG_ALL_LED_OFF_L      0xFCU
#define PCA9685_REG_ALL_LED_OFF_H      0xFDU
#define PCA9685_REG_PRESCALE           0xFEU

/* Биты MODE1 */
#define PCA9685_MODE1_RESTART          (1U << 7)
#define PCA9685_MODE1_EXTCLK           (1U << 6)
#define PCA9685_MODE1_AI               (1U << 5)
#define PCA9685_MODE1_SLEEP            (1U << 4)
#define PCA9685_MODE1_SUB1             (1U << 3)
#define PCA9685_MODE1_SUB2             (1U << 2)
#define PCA9685_MODE1_SUB3             (1U << 1)
#define PCA9685_MODE1_ALLCALL          (1U << 0)

/* Биты MODE2 */
#define PCA9685_MODE2_INVRT            (1U << 4)
#define PCA9685_MODE2_OCH              (1U << 3)
#define PCA9685_MODE2_OUTDRV           (1U << 2)
#define PCA9685_MODE2_OUTNE1           (1U << 1)
#define PCA9685_MODE2_OUTNE0           (1U << 0)

/* Каналы 0..15 -------------------------------------------------------------*/
enum pca9685_channel_e{
  PCA9685_CH0 = 0,
  PCA9685_CH1,
  PCA9685_CH2,
  PCA9685_CH3,
  PCA9685_CH4,
  PCA9685_CH5,
  PCA9685_CH6,
  PCA9685_CH7,
  PCA9685_CH8,
  PCA9685_CH9,
  PCA9685_CH10,
  PCA9685_CH11,
  PCA9685_CH12,
  PCA9685_CH13,
  PCA9685_CH14,
  PCA9685_CH15
};

/* Типы функций для низкоуровневого I2C ------------------------------------
 *
 * Ожидается, что вы реализуете эти функции где‑то в BSP (например, bsp_i2c.c)
 * с использованием HAL или LL:
 *
 * int PCA9685_I2C_Write(uint8_t dev_addr_7bit, uint8_t reg,
 *                       const uint8_t *data, uint16_t size);
 * int PCA9685_I2C_Read (uint8_t dev_addr_7bit, uint8_t reg,
 *                       uint8_t *data, uint16_t size);
 *
 * Функции должны вернуть 0 при успехе и ненулевое значение при ошибке.
 */
int PCA9685_I2C_Write(uint8_t dev_addr_7bit, uint8_t reg,
                      const uint8_t *data, uint16_t size);
int PCA9685_I2C_Read (uint8_t dev_addr_7bit, uint8_t reg,
                      uint8_t *data, uint16_t size);

/* Высокоуровневый интерфейс -----------------------------------------------*/
struct pca9685_s{
  /* Инициализация:
   * - i2c_addr_7bit  – адрес PCA9685 (0x40..0x7F)
   * - pwm_freq_hz    – частота PWM (для серв обычно 50 Гц)
   * - pulse_min_us   – ширина импульса для 0° (обычно 500–600 мкс)
   * - pulse_max_us   – ширина импульса для max_angle_deg (обычно 2400 мкс)
   * - max_angle_deg  – полный ход сервопривода (обычно 180° или 270°)
   */
  int  (*Init)(uint8_t i2c_addr_7bit,
               float   pwm_freq_hz,
               uint16_t pulse_min_us,
               uint16_t pulse_max_us,
               uint16_t max_angle_deg);

  /* Установка «сырых» значений ON/OFF (0..4095) для канала */
  int  (*SetPWM)(enum pca9685_channel_e ch,
                 uint16_t on,
                 uint16_t off);

  /* Установка угла сервопривода (в градусах) на канале */
  int  (*SetServoAngle)(enum pca9685_channel_e ch,
                        uint16_t angle_deg);

  /* Установка угла сервопривода с указанием I2C адреса платы */
  int  (*SetServoAngleWithAddr)(uint8_t i2c_addr_7bit,
                                 enum pca9685_channel_e ch,
                                 uint16_t angle_deg);

  /* Включить/выключить все каналы сразу (по ширине импульса в мкс) */
  int  (*SetAllServoPulseUs)(uint16_t pulse_us);

  /* Перевод микросхемы в сон и выход из сна */
  int  (*Sleep)(void);
  int  (*Wakeup)(void);
};

extern struct pca9685_s PCA9685;

#ifdef __cplusplus
}
#endif

#endif /* __PCA9685_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




