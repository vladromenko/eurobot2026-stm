/**
  ******************************************************************************
  * @file    hw_instance.hpp
  * @brief   All peripheral drivers of the robot in one statically allocated object.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_INSTANCE_H
#define __HW_INSTANCE_H

/*  BSP  */
#include "stepper.hpp"
#include "l6474_drvr.hpp"
#include "pwm.hpp"
#include "spi_gpo.hpp"
#include "ws2812b.hpp"
#include "usartx_communication.hpp"
#include "usartx_timeout.hpp"
#include "i2c2.hpp"

/*  Lib  */
#include "dynamixel_master.hpp"
#include "dynamixel_ax.hpp"
#include "dynamixel_mx.hpp"
#include "maxon.hpp"
#include "bms_board.hpp"
#include "serial.hpp"
#include "hl_rtu.hpp"
#include "pca9685.hpp"
#include "rc_servo.hpp"
#include "h_bridge.hpp"
#include "button.hpp"

/**
 * Owner of every peripheral driver. The object is built at compile time
 * (constinit), nothing is constructed at startup and nothing is allocated.
 * Pass its fields to other classes when constructing them:
 *
 *   constinit OdometryModule Odometry{GetHwInstance().maxon, Kinematics};
 *
 * The old global names (Stepper, Maxon, ...) are references to these fields.
 */
struct HwInstance{
  /*  Wires the devices to their Dynamixel buses  */
  constexpr HwInstance() {}
  HwInstance(const HwInstance&) = delete;
  HwInstance& operator=(const HwInstance&) = delete;

  /*  BSP  */
  StepperDriver stepper{};
  L6474H_Driver l6474h{};
  PWM_Driver pwm{};
  SPI_GPO_Driver spi_gpo{};
  WS2812B_Driver ws2812b{};
  USARTx_Driver usartx{};
  USARTx_TimeoutDriver usartx_timeout{};
  I2C2_Driver i2c2{};

  /*  Lib: protocols, bound to their USARTs in main()  */
  DynamixelMasterDriver dynamixel_bus3{};   /*  USART3: AX servos  */
  DynamixelMasterDriver dynamixel_bus4{};   /*  UART4: BMS board  */
  DynamixelMasterDriver dynamixel_bus6{};   /*  USART6: Maxon motors  */
  SerialDriver serial{};                    /*  UART5: debug terminal  */
  HL_RTU_Driver hl_rtu{};                   /*  USART1: Raspberry Pi link  */

  /*  Lib: devices  */
  Dynamixel_AX_Driver dynamixel_ax{dynamixel_bus3};
  Dynamixel_MX_Driver dynamixel_mx{dynamixel_bus3};
  MaxonDriver maxon{dynamixel_bus6};
  BMS_Driver bms{dynamixel_bus4};
  PCA9685_Driver pca9685{};
  RC_ServoDriver rc_servo{};
  H_BridgeDriver h_bridge{};
  ButtonDriver button{};
};

/*  Use GetHwInstance(); declared here only so that the getter can be constexpr  */
extern HwInstance hw_instance;

constexpr HwInstance& GetHwInstance()
{
  return hw_instance;
}

#endif
