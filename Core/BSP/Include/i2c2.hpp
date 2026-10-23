/**
  ******************************************************************************
  * @file    i2c2.hpp
  * @brief   Blocking I2C2 master with bounded waits (PCA9685 boards).
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C2_H
#define __I2C2_H

#include "std_def.hpp"

/*  Register access to a 7-bit addressed device. Return 0 on success, negative on error  */
class I2C2_Driver{
public:
  int Write(uint8_t dev_addr_7bit, uint8_t reg, const uint8_t *data, uint16_t size);
  int Read(uint8_t dev_addr_7bit, uint8_t reg, uint8_t *data, uint16_t size);
};

/*  The object lives in HwInstance (hw_instance.hpp); this is a shortcut to it  */
extern I2C2_Driver& I2C2_Bus;

#endif
