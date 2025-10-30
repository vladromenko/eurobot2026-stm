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

#include "terminal_cmd_handlers.h" 
#include "dynamixel_master.h"
#include "byte2word.h"

#include "board_gpio_defines.h"
#include "stm32f4xx_ll_gpio.h"
#include "terminal.h"

#include "dynamixel_ax.h" 
#include "dynamixel_mx.h" 
#include "stm32f4xx_ll_usart.h"
#include "os_tasks.h"
#include "kinematics.h"
#include "odometry.h" 
#include "bms_board.h"
#include "bus_device.h"
#include "robot_behavior_routine.h"
#include "spi_gpo.h" 
#include "spi_gpo_defines.h" 
#include "rc_servo.h" 
#include "stepper.h" 

extern int32_t stepper1_goal_position;
extern int32_t stepper2_goal_position;
extern int32_t stepper3_goal_position;
extern int32_t stepper4_goal_position;

static void Ping(uint8_t* data, uint8_t data_length)
{
  LL_GPIO_TogglePin(GPIOD, PWM2_PD14);

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_Ping;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void SetRobotSpeed(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct speed_s speed;
  
  ByteTo_Word32(data,&speed.dx);
  ByteTo_Word32(data+4,&speed.dy);
  ByteTo_Word32(data+8,&speed.dphi);

  Odometry.SetSpeed(&speed);

  terminal_response.cmd = BOARD_SetSpeed;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void GetRobotSpeed(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct speed_s speed;

  Odometry.GetSpeed(&speed);

  Word32_ToByte(&speed.dx,terminal_response.data);
  Word32_ToByte(&speed.dy,terminal_response.data+4);
  Word32_ToByte(&speed.dphi,terminal_response.data+8);

  terminal_response.cmd = BOARD_GetSpeed;
  terminal_response.data_length = 12;

  Terminal.SendResponse(&terminal_response);
}

static void SetRobotCoord(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct coord_s coord;
  
  ByteTo_Word32(data,&coord.x);
  ByteTo_Word32(data+4,&coord.y);
  ByteTo_Word32(data+8,&coord.phi);

  Odometry.SetCoord(&coord);

  terminal_response.cmd = BOARD_SetCoord;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void GetRobotCoord(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct coord_s coord;

  Odometry.GetCoord(&coord);

  Word32_ToByte(&coord.x,terminal_response.data);
  Word32_ToByte(&coord.y,terminal_response.data+4);
  Word32_ToByte(&coord.phi,terminal_response.data+8);

  terminal_response.cmd = BOARD_GetCoord;
  terminal_response.data_length = 12;
  
  Terminal.SendResponse(&terminal_response);
}

static void GetBatteryState(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct bms_battery_state_s battery_state;
  BMS.GetBatteryState(bus_devices[BMS_BOARD].usart, bus_devices[BMS_BOARD].id, &battery_state);
  memcpy(terminal_response.data, &battery_state, 18);

  terminal_response.cmd = BOARD_GetBatteryState;
  terminal_response.data_length = 18;
  
  Terminal.SendResponse(&terminal_response);
}

static void Set_BMS_CalibrationValue(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  uint16_t calibration_value;
  ByteTo_Word16(data+1, &calibration_value);

  BMS.SetCalibrationValue(bus_devices[BMS_BOARD].usart, bus_devices[BMS_BOARD].id, data[0], calibration_value);
  
  terminal_response.cmd = BOARD_Set_BMS_CalibrationValue;
  terminal_response.data_length = 0;
  
  Terminal.SendResponse(&terminal_response);
}

static void GetAxAngle(uint8_t* data, uint8_t data_length)
{
  uint16_t angle;
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_GET_AX_ANGLE;
  terminal_response.data[0] = Dynamixel_AX.GetAngle(USART3, data[0], &angle);
  Word16_ToByte(&angle, &terminal_response.data[1]);
  terminal_response.data_length = 3;

  Terminal.SendResponse(&terminal_response);
}

static void SetAxAngle(uint8_t* data, uint8_t data_length)
{
  uint8_t id = data[0];
  uint16_t angle;
  ByteTo_Word16(data+1,&angle);
	uint16_t speed;
  ByteTo_Word16(data+3,&speed);

	int16_t curr_speed = -1;
	while(curr_speed != speed){
		Dynamixel_AX.SetMovingSpeed(USART3, id, speed);
		Dynamixel_AX.GetMovingSpeed(USART3, id, &curr_speed);
	}
	
  Dynamixel_AX.TorqueEnable(USART3,id);
  Dynamixel_AX.SetAngle(USART3,id,angle);

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_SET_AX_ANGLE;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void SetStepperZero(uint8_t* data, uint8_t data_length)
{
	uint8_t id = data[0];
  
	switch(id){
		case 0:
			Stepper.SetZeroPosisition(STEPPER_MOTOR_1);
			stepper1_goal_position = 0;
			break;
		case 1:
			Stepper.SetZeroPosisition(STEPPER_MOTOR_2);
		stepper2_goal_position = 0;
			break;
		case 2:
			Stepper.SetZeroPosisition(STEPPER_MOTOR_3);
		stepper3_goal_position = 0;
			break;
		case 3:
			Stepper.SetZeroPosisition(STEPPER_MOTOR_4);
		stepper4_goal_position = 0;
			break;
		default:
			break;
	}
	
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_SET_STEPPER_ZERO;
	
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void GetStepperPosition(uint8_t* data, uint8_t data_length)
{
  int16_t position;
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_GET_STEPPER_POSITION;
	
	switch(data[0]){
		case 0:
			position = Stepper.GetPresentPosisition(STEPPER_MOTOR_1);
			break;
		case 1:
			position = Stepper.GetPresentPosisition(STEPPER_MOTOR_2);
			break;
		case 2:
			position = Stepper.GetPresentPosisition(STEPPER_MOTOR_3);
			break;
		case 3:
			position = Stepper.GetPresentPosisition(STEPPER_MOTOR_4);
			break;
		default:
			break;
	}
	
  Word16_ToByte(&position, &terminal_response.data[0]);
  terminal_response.data_length = 2;

  Terminal.SendResponse(&terminal_response);
}

static void SetStepperPosition(uint8_t* data, uint8_t data_length)
{
  uint8_t id = data[0];
  int16_t position;
  
  ByteTo_Word16(data+1,&position);
	
	switch(id){
		case 0:
			stepper1_goal_position = position;
			Stepper.StepToPosisition(STEPPER_MOTOR_1, position, 32, 16, 800);
			break;
		case 1:
			stepper2_goal_position = position;
			Stepper.StepToPosisition(STEPPER_MOTOR_2, position, 32, 16, 800);
			break;
		case 2:
			stepper3_goal_position = position;
		  Stepper.StepToPosisition(STEPPER_MOTOR_3, position, 32, 16, 800);
			break;
		case 3:
			stepper4_goal_position = position;
		  Stepper.StepToPosisition(STEPPER_MOTOR_4, position, 32, 16, 800);
			break;
		default:
			break;
	}

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_SET_STEPPER_POSITION;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void TogglePump(uint8_t* data, uint8_t data_length)
{
  uint8_t pump_state;
  struct terminal_response_s terminal_response;

  pump_state = 2*data[0] + data[1];
  switch (pump_state)
    {
      case 0: //open grab plank 1
				SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_1);
        break;
      case 1: //close grab plank 1
				SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_1);
				break;
			case 2:
				SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_2);
				break;
			case 3:
				SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_2);
				break;
			default:
				break;
    }

  
  terminal_response.cmd = BOARD_TOGGLE_PUMP;
  terminal_response.data_length = 0;
  
  Terminal.SendResponse(&terminal_response);
}

static void SetMagnetAngle(uint8_t* data, uint8_t data_length)
{
  uint8_t magnet_id = data[0];
	int16_t position;
  ByteTo_Word16(data+1,&position);
	
  struct terminal_response_s terminal_response;
 
  switch (magnet_id)
    {
      case 0:
				RC_Servo.Enable(RC_SERVO_1);
				RC_Servo.SetServoAngle(RC_SERVO_1, 525, 2500, 180, position);
        break;
      case 1:
				RC_Servo.Enable(RC_SERVO_2);
				RC_Servo.SetServoAngle(RC_SERVO_2, 525, 2500, 180, position);
				break;
			case 2:
				RC_Servo.Enable(RC_SERVO_3);
				RC_Servo.SetServoAngle(RC_SERVO_3, 525, 2500, 180, position);
        break;
      case 3:
				RC_Servo.Enable(RC_SERVO_4);
				RC_Servo.SetServoAngle(RC_SERVO_4, 525, 2500, 180, position);
        break;
			default:
				break;
    }

  
  terminal_response.cmd = BOARD_SET_MAGNET_ANGLE;
  terminal_response.data_length = 0;
  
  Terminal.SendResponse(&terminal_response);
}

static void GetKey(uint8_t* data, uint8_t data_length)
{
 
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_GET_KEY;
  terminal_response.data[0] = (robot_ctrl.starting_cord == STARTING_CORD_RELEASED)?1:0;
  terminal_response.data_length = 1;

  Terminal.SendResponse(&terminal_response);
}

static void GetToggle(uint8_t* data, uint8_t data_length)
{
  
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_GET_TOGGLE;
  terminal_response.data[0] = (robot_ctrl.maxons_state == MAXONS_ENABLED)?1:0;
  terminal_response.data_length = 1;

  Terminal.SendResponse(&terminal_response);
}

void (*terminal_cmd_handlers[TERMINAL_CMD_TABLE_SIZE])(uint8_t* data, uint8_t data_length) = {[BOARD_Ping] = Ping,
                                                                                              [BOARD_SetSpeed] = SetRobotSpeed,
                                                                                              [BOARD_GetSpeed] = GetRobotSpeed,
                                                                                              [BOARD_SetCoord] = SetRobotCoord,
                                                                                              [BOARD_GetCoord] = GetRobotCoord,
                                                                                              [BOARD_GetBatteryState] = GetBatteryState,
                                                                                              [BOARD_Set_BMS_CalibrationValue] = Set_BMS_CalibrationValue,
																																															[BOARD_GET_AX_ANGLE] = GetAxAngle,
                                                                                              [BOARD_SET_AX_ANGLE] = SetAxAngle,
																																															[BOARD_SET_STEPPER_ZERO] = SetStepperZero,
																																															[BOARD_SET_STEPPER_POSITION] = SetStepperPosition,
																																															[BOARD_GET_STEPPER_POSITION] = GetStepperPosition,
																																															[BOARD_TOGGLE_PUMP] = TogglePump,
																																															[BOARD_SET_MAGNET_ANGLE] = SetMagnetAngle,
																																															[BOARD_GET_KEY] = GetKey,
																																															[BOARD_GET_TOGGLE] = GetToggle
};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
