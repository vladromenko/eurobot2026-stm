/**
  ******************************************************************************
  * @file    hw_instance.cpp
  * @brief   All peripheral drivers of the robot in one statically allocated object.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw_instance.hpp"


constinit HwInstance hw_instance;

/*  Shortcuts kept for the existing code: Stepper.X() is GetHwInstance().stepper.X()  */
constinit StepperDriver& Stepper = hw_instance.stepper;
constinit L6474H_Driver& L6474H = hw_instance.l6474h;
constinit PWM_Driver& PWM = hw_instance.pwm;
constinit SPI_GPO_Driver& SPI_GPO = hw_instance.spi_gpo;
constinit WS2812B_Driver& WS2812B = hw_instance.ws2812b;
constinit USARTx_Driver& USARTx = hw_instance.usartx;
constinit USARTx_TimeoutDriver& USARTx_Timeout = hw_instance.usartx_timeout;
constinit I2C2_Driver& I2C2_Bus = hw_instance.i2c2;
constinit DynamixelMasterDriver& DynamixelBus3 = hw_instance.dynamixel_bus3;
constinit DynamixelMasterDriver& DynamixelBus4 = hw_instance.dynamixel_bus4;
constinit DynamixelMasterDriver& DynamixelBus6 = hw_instance.dynamixel_bus6;
constinit SerialDriver& Serial = hw_instance.serial;
constinit HL_RTU_Driver& HL_RTU = hw_instance.hl_rtu;
constinit Dynamixel_AX_Driver& Dynamixel_AX = hw_instance.dynamixel_ax;
constinit Dynamixel_MX_Driver& Dynamixel_MX = hw_instance.dynamixel_mx;
constinit MaxonDriver& Maxon = hw_instance.maxon;
constinit BMS_Driver& BMS = hw_instance.bms;
constinit PCA9685_Driver& PCA9685 = hw_instance.pca9685;
constinit RC_ServoDriver& RC_Servo = hw_instance.rc_servo;
constinit H_BridgeDriver& H_Bridge = hw_instance.h_bridge;
constinit ButtonDriver& Button = hw_instance.button;
