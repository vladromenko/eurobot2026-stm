/**
  ******************************************************************************
  * @file    	hw_instance.h
  * @brief   All peripheral drivers of the robot in one statically allocated object.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_INSTANCE_H
#define __HW_INSTANCE_H

#ifdef __cplusplus
 extern "C" {
#endif

/*  BSP  */
#include "stepper.h"
#include "l6474_drvr.h"
#include "pwm.h"
#include "spi_gpo.h"
#include "ws2812b.h"
#include "usartx_communication.h"

/*  Lib  */
#include "dynamixel_master.h"
#include "dynamixel_ax.h"
#include "dynamixel_mx.h"
#include "maxon.h"
#include "bms_board.h"
#include "serial.h"
#include "hl_rtu.h"
#include "rc_servo.h"
#include "h_bridge.h"
#include "button.h"

/*  App  */
#include "bus_device.h"


/*
 *  Application code reaches the hardware only through this object:
 *
 *    const struct hw_instance_s* hw = GetHwInstance();
 *    hw->maxon->Set_TPM(hw->bus_devices[MAXON_MOTOR_1].usart, hw->bus_devices[MAXON_MOTOR_1].id, tpm);
 *
 *  Drivers inside BSP/Lib still use each other directly to keep the layers independent of App.
 */
struct hw_instance_s{
    /*  BSP  */
    const struct stepper_s* stepper;
    const struct l6474h_s* l6474h;
    const struct pwm_s* pwm;
    const struct spi_gpo_s* spi_gpo;
    const struct ws2812b_s* ws2812b;
    const struct usartx_s* usartx;

    /*  Lib: protocols  */
    const struct dynamixel_master_s* dynamixel_master;  /*  USART3, UART4, USART6  */
    const struct serial_s* serial;                      /*  UART5  */
    const struct hl_rtu_s* hl_rtu;                      /*  USART1: terminal  */

    /*  Lib: devices  */
    const struct dynamixel_ax_s* dynamixel_ax;
    const struct dynamixel_mx_s* dynamixel_mx;
    const struct maxon_s* maxon;
    const struct bms_board_s* bms;
    const struct rc_servo_s* rc_servo;
    const struct h_bridge_s* h_bridge;
    const struct button_s* button;

    /*  Bus addresses of the devices, indexed by enum bus_devices_e  */
    const struct bus_device_s* bus_devices;
};

const struct hw_instance_s* GetHwInstance(void);

#ifdef __cplusplus
}
#endif

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
