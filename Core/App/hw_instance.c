/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         hw_instance.c
  * @brief        All peripheral drivers of the robot in one statically allocated object.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "hw_instance.h"


static const struct hw_instance_s hw_instance = {.stepper = &Stepper,
                                                 .l6474h = &L6474H,
                                                 .pwm = &PWM,
                                                 .spi_gpo = &SPI_GPO,
                                                 .ws2812b = &WS2812B,
                                                 .usartx = &USARTx,
                                                 .dynamixel_master = &DynamixelMaster,
                                                 .serial = &Serial,
                                                 .hl_rtu = &HL_RTU,
                                                 .dynamixel_ax = &Dynamixel_AX,
                                                 .dynamixel_mx = &Dynamixel_MX,
                                                 .maxon = &Maxon,
                                                 .bms = &BMS,
                                                 .rc_servo = &RC_Servo,
                                                 .h_bridge = &H_Bridge,
                                                 .button = &Button,
                                                 .bus_devices = bus_devices};

const struct hw_instance_s* GetHwInstance(void)
{
  return &hw_instance;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
