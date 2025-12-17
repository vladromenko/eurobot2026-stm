/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         functions.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "bus_device.h"


const struct bus_device_s bus_devices[BUS_DEVICES_NUMBER] = {[MAXON_MOTOR_1] = {USART6, 0x23}, //0x22 (X back, Y right) | 0x23 (X forw, Y right)
                                                              [MAXON_MOTOR_2] = {USART6, 0x21}, //0x21 | 0x21
                                                              [MAXON_MOTOR_3] = {USART6, 0x22}, //0x23 | 0x22
                                                              [BMS_BOARD] = {UART4, 0x11},
                                                              [SERVO_GRAB] = {USART3, 92},
                                                              [SERVO_LIFT11] = {USART3, 6},
																															[SERVO_LIFT12] = {USART3, 98},
                                                              [SERVO_LIFT21] = {USART3, 1},
																															[SERVO_LIFT22] = {USART3, 55},

};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
