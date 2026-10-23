/**
  ******************************************************************************
  * @file    	functions.h
  * @brief   HAL configuration file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DYNAMIXEL_MX_H
#define __DYNAMIXEL_MX_H

#include "std_def.hpp"
#include "dynamixel_master.hpp"

/*	MX EEPROM Area registers define ------------------------------------------------------------- */
#define MX_ModelNumber           	0     /* Model Number (2 bytes)                       */
#define MX_FirmwareVersion       	2     /* Firmware Version (1 byte)	                  */
#define MX_ID              				3     /* DYNAMIXEL ID (1 byte)	                      */
#define MX_BaudRate              	4     /* Communication Speed (1 byte)	                */
#define MX_ReturnDelayTime       	5     /* Response Delay Time (1 byte)	                */
#define MX_CW_AngleLimit     			6     /* Clockwise Angle Limit (2 bytes)	            */
#define MX_CCW_AngleLimit     		8     /* Counter-Clockwise Angle Limit (2 bytes)	    */
#define MX_DriveMode      	      10    /* Dual Mode Setting (1 byte)                   */
#define MX_TemperatureLimit      	11    /* Maximum Internal Temperature Limit	(1 byte)  */
#define MX_Min_VoltageLimit  			12    /* Minimum Input Voltage Limit (1 byte)	        */
#define MX_Max_VoltageLimit  			13    /* Maximum Input Voltage Limit (1 byte)	        */
#define MX_MaxTorque           		14    /* Maximun Torque (2 bytes)	                    */
#define MX_StatusReturnLevel     	16    /* Select Types of Status Return (1 byte)	      */
#define MX_AlarmLED          			17    /* LED for Alarm (1 byte)	                      */
#define MX_Shutdown            		18    /* Shutdown Error Information (1 byte)	        */
#define MX_MultiTurnOffset        20    /* Adjust Position with Offset (2 bytes)	      */
#define MX_ResolutionDivider      22    /* Divider for Position Resolution (1 byte)	    */

/*	MX RAM Area registers define ---------------------------------------------------------------- */
#define MX_TorqueEnable          	24    /* Motor Torque On/Off (1 byte)	                */
#define MX_LED           					25    /* Status LED On/Off (1 byte)                   */
#define MX_D_Gain   	            26    /* Derivative Gain (1 byte) 	                  */
#define MX_I_Gain  	              27    /* Integral Gain (1 byte) 	                    */
#define MX_P_Gain    	            28    /* Proportional Gain (1 byte)	                  */
#define MX_GoalPosition          	30    /* Desired Position (2 bytes)	                  */
#define MX_MovingSpeed           	32    /* Moving Speed(Moving Velocity) (2 bytes)	    */
#define MX_TorqueLimit           	34    /* Torque Limit (2 bytes)	                      */
#define MX_PresentPosition       	36    /* Present Position (2 bytes)                   */
#define MX_PresentSpeed         	38    /* Present Speed (2 bytes)	                    */
#define MX_PresentLoad           	40    /* Present Load (2 bytes)	                      */
#define MX_PresentVoltage        	42    /* Present Voltage (1 byte)	                    */
#define MX_PresentTemperature    	43    /* Present Temperature (1 byte)	                */
#define MX_Registered  	          44    /* If Instruction is registered (1 byte)	      */
#define MX_Moving            			46    /* Movement Status (1 byte)	                    */
#define MX_Lock            				47    /* Locking EEPROM (1 byte)	                    */
#define MX_Punch            			48    /* Minimum Current Threshold (2 bytes)	        */
#define MX_RealtimeTick    	      50    /* Count Time in millisecond (2 bytes)	        */
#define MX_Current  	            68    /* Consuming Current (2 bytes)	                */
#define MX_TorqueCtrlModeEnable   70    /* Torque Control Mode On/Off (1 byte)	        */
#define MX_GoalTorque            	71    /* Goal Torque (2 bytes)	                      */
#define MX_GoalAcceleration       73    /* Goal Acceleration (1 byte)	                  */

/*  MX-28/64 servo on a Dynamixel line; angle in degrees 0..360  */
class Dynamixel_MX_Driver{
public:
    constexpr explicit Dynamixel_MX_Driver(DynamixelMasterDriver& bus) : bus(bus) {}

    uint8_t Ping(uint8_t servo_id);
    void Set_ID(uint8_t current_id, uint8_t new_id);
    void TorqueEnable(uint8_t servo_id);
    void TorqueDisable(uint8_t servo_id);
    void SetAngle(uint8_t servo_id, uint16_t angle);
    uint8_t GetAngle(uint8_t servo_id, uint16_t* angle);
    void SetWheelMode(uint8_t servo_id);
    void SetJointMode(uint8_t servo_id);
    void SetMovingSpeed(uint8_t servo_id, int16_t speed);
    uint8_t GetPresentLoad(uint8_t servo_id, int16_t* load);
    void Enable_LED(uint8_t servo_id);
    void Disable_LED(uint8_t servo_id);

private:
    DynamixelMasterDriver& bus;
};

/*  The object lives in HwInstance (hw_instance.hpp); this is a shortcut to it  */
extern Dynamixel_MX_Driver& Dynamixel_MX;

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
