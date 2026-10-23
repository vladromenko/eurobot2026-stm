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
#ifndef __DYNAMIXEL_AX_H
#define __DYNAMIXEL_AX_H

#include "std_def.hpp"
#include "dynamixel_master.hpp"

	 
/*	AX EEPROM Area registers define ------------------------------------------------------------- */
#define AX_ModelNumber           	0     /* Model Number (2 bytes)                         */
#define AX_FirmwareVersion       	2     /* Firmware Version (1 byte) 	                    */
#define AX_ID              				3     /* DYNAMIXEL ID (1 byte) 				                  */
#define AX_BaudRate              	4     /* Communication Speed (1 byte)  	                */
#define AX_ReturnDelayTime       	5     /* Response Delay Time (1 byte)  		              */
#define AX_CW_AngleLimit     			6     /* Clockwise Angle Limit (2 bytes)  	            */
#define AX_CCW_AngleLimit     		8     /* Counter-Clockwise Angle Limit (2 bytes)        */
#define AX_TemperatureLimit      	11    /* Maximum Internal Temperature Limit (1 byte)    */
#define AX_Min_VoltageLimit  			12    /* Minimum Input Voltage Limit (1 byte)           */
#define AX_Max_VoltageLimit  			13    /* Maximum Input Voltage Limit (1 byte)           */
#define AX_MaxTorque           		14    /* Maximun Torque (2 bytes)     							    */
#define AX_StatusReturnLevel     	16    /* Select Types of Status Return (1 byte)         */
#define AX_AlarmLED          			17    /* LED for Alarm (1 byte)     						        */
#define AX_Shutdown            		18    /* Shutdown Error Information (1 byte) 		        */


/*	AX RAM Area registers define ---------------------------------------------------------------- */
#define AX_TorqueEnable          	24    /* Motor Torque On/Off (1 byte)    						    */
#define AX_LED           					25    /* Status LED On/Off (1 byte)                     */
#define AX_CW_ComplianceMargin   	26    /* CW Compliance Margin (1 byte) 	                */
#define AX_CCW_ComplianceMargin  	27    /* CCW Compliance Margin (1 byte) 	              */
#define AX_CW_ComplianceSlope    	28    /* CW Compliance Slope (1 byte) 				          */
#define AX_CCW_ComplianceSlope   	29    /* CCW Compliance Slope (1 byte) 				          */
#define AX_GoalPosition          	30    /* Target Position (2 bytes)  			              */
#define AX_MovingSpeed           	32    /* Moving Speed (2 bytes)  					              */
#define AX_TorqueLimit           	34    /* Torque Limit (2 bytes) 				                */
#define AX_PresentPosition       	36    /* Present Position (2 bytes)   			            */
#define AX_PresentSpeed         	38    /* Present Speed (2 bytes)				                */
#define AX_PresentLoad           	40    /* Present Load (2 bytes)				                  */
#define AX_PresentVoltage        	42    /* Present Voltage (1 byte)				                */
#define AX_PresentTemperature    	43    /* Present Temperature (1 byte)				            */
#define AX_RegisteredInstruction 	44    /* If Instruction is registered (1 byte)	        */
#define AX_Moving            			46    /* Movement Status (1 byte)				                */
#define AX_Lock            				47    /* Locking EEPROM (1 byte)				                */
#define AX_Punch            			48    /* Minimum Current Threshold	 (2 bytes)	        */



/*  AX-12 servo on a Dynamixel line; angle in degrees 0..300  */
class Dynamixel_AX_Driver{
public:
    constexpr explicit Dynamixel_AX_Driver(DynamixelMasterDriver& bus) : bus(bus) {}

    uint8_t Ping(uint8_t servo_id);
    void Set_ID(uint8_t current_id, uint8_t new_id);
    void TorqueEnable(uint8_t servo_id);
    void TorqueDisable(uint8_t servo_id);
    void SetAngle(uint8_t servo_id, uint16_t angle);
    uint8_t GetAngle(uint8_t servo_id, uint16_t* angle);
    void SetWheelMode(uint8_t servo_id);
    void SetJointMode(uint8_t servo_id);
    void SetMovingSpeed(uint8_t servo_id, int16_t speed);
    uint8_t GetMovingSpeed(uint8_t servo_id, int16_t* speed);
    uint8_t GetPresentLoad(uint8_t servo_id, int16_t* load);
    void Enable_LED(uint8_t servo_id);
    void Disable_LED(uint8_t servo_id);

private:
    DynamixelMasterDriver& bus;
};

extern Dynamixel_AX_Driver Dynamixel_AX;


#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
