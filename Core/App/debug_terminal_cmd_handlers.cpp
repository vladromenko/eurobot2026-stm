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

#include "debug_terminal_cmd_handlers.hpp" 
#include "dynamixel_master.hpp"
#include "byte2word.hpp"

#include "board_gpio_defines.hpp"
#include "stm32f4xx_ll_gpio.h"

#include "dynamixel_ax.hpp" 
#include "dynamixel_mx.hpp" 
#include "stm32f4xx_ll_usart.h"
#include "os_tasks.hpp"
#include "kinematics.hpp"
#include "odometry.hpp" 
#include "bms_board.hpp"
#include "bus_device.hpp"
#include "robot_behavior_routine.hpp"
#include "spi_gpo.hpp" 
#include "spi_gpo_defines.hpp" 
#include "rc_servo.hpp" 
#include "stepper.hpp" 
#include "serial.hpp" 
#include "pca9685.hpp" 

extern int32_t stepper1_goal_position;
extern int16_t stepper2_goal_position;
extern int16_t stepper3_goal_position;

extern uint8_t stepper1_calibrate;
extern uint8_t stepper2_calibrate;
extern uint8_t stepper3_calibrate;
constinit DebugTerminalCmdHandler DebugTerminalCmd;

void DebugTerminalCmdHandler::Handle(debug_terminal_request_s* request)
{
  if((request->cmd < TERMINAL_CMD_TABLE_SIZE) && (handlers[request->cmd] != nullptr))
    (this->*handlers[request->cmd])(request->data, request->data_length);
}
void DebugTerminalCmdHandler::DebugPing([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
  LL_GPIO_TogglePin(GPIOD, PWM2_PD14);

  struct debug_terminal_response_s debug_terminal_response = {};

  debug_terminal_response.cmd = BOARD_Ping;
	debug_terminal_response.data_length = 0;
  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugSetRobotSpeed([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
	
	if(comm_enable){
		struct speed_s speed;
  
  ByteTo_Word32(data,&speed.dx);
  ByteTo_Word32(data+4,&speed.dy);
  ByteTo_Word32(data+8,&speed.dphi);

  Odometry.SetSpeed(&speed);

	}
  struct debug_terminal_response_s debug_terminal_response = {};

  
  debug_terminal_response.cmd = BOARD_SetSpeed;
  debug_terminal_response.data_length = 0;

  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugGetRobotSpeed([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
	
  struct debug_terminal_response_s debug_terminal_response = {};

  struct speed_s speed;

  Odometry.GetSpeed(&speed);

  Word32_ToByte(&speed.dx,debug_terminal_response.data);
  Word32_ToByte(&speed.dy,debug_terminal_response.data+4);
  Word32_ToByte(&speed.dphi,debug_terminal_response.data+8);

  debug_terminal_response.cmd = BOARD_GetSpeed;
  debug_terminal_response.data_length = 12;

  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugSetRobotCoord([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
	if(comm_enable){
		struct coord_s coord;
  
  ByteTo_Word32(data,&coord.x);
  ByteTo_Word32(data+4,&coord.y);
  ByteTo_Word32(data+8,&coord.phi);

  Odometry.SetCoord(&coord);
	}
  struct debug_terminal_response_s debug_terminal_response = {};

  

  debug_terminal_response.cmd = BOARD_SetCoord;
  debug_terminal_response.data_length = 0;

  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugGetRobotCoord([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
  struct debug_terminal_response_s debug_terminal_response = {};

  struct coord_s coord;

  Odometry.GetCoord(&coord);

  Word32_ToByte(&coord.x,debug_terminal_response.data);
  Word32_ToByte(&coord.y,debug_terminal_response.data+4);
  Word32_ToByte(&coord.phi,debug_terminal_response.data+8);

  debug_terminal_response.cmd = BOARD_GetCoord;
  debug_terminal_response.data_length = 12;
  
  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugGetBatteryState([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
  struct debug_terminal_response_s debug_terminal_response = {};

  struct bms_battery_state_s battery_state;
  BMS.GetBatteryState(bus_devices[BMS_BOARD].id, &battery_state);
  memcpy(debug_terminal_response.data, &battery_state, 18);

  debug_terminal_response.cmd = BOARD_GetBatteryState;
  debug_terminal_response.data_length = 18;
  
  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugSet_BMS_CalibrationValue([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
  struct debug_terminal_response_s debug_terminal_response = {};

  uint16_t calibration_value;
  ByteTo_Word16(data+1, &calibration_value);

  BMS.SetCalibrationValue(bus_devices[BMS_BOARD].id, data[0], calibration_value);
  
  debug_terminal_response.cmd = BOARD_Set_BMS_CalibrationValue;
  debug_terminal_response.data_length = 0;
  
  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugGetAxAngle([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
  uint16_t angle;
  struct debug_terminal_response_s debug_terminal_response = {};

  debug_terminal_response.cmd = BOARD_GET_AX_ANGLE;
  debug_terminal_response.data[0] = Dynamixel_AX.GetAngle(data[0], &angle);
  Word16_ToByte(&angle, &debug_terminal_response.data[1]);
  debug_terminal_response.data_length = 3;

  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugSetAxAngle([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
	if(comm_enable){
  uint8_t id = data[0];
  uint16_t angle;
  ByteTo_Word16(data+1,&angle);
	uint16_t speed;
  ByteTo_Word16(data+3,&speed);

	int16_t curr_speed = -1;
	while(curr_speed != speed){
		Dynamixel_AX.SetMovingSpeed(id, speed);
		Dynamixel_AX.GetMovingSpeed(id, &curr_speed);
	}
	
  Dynamixel_AX.TorqueEnable(id);
  Dynamixel_AX.SetAngle(id,angle);
}
  struct debug_terminal_response_s debug_terminal_response = {};

  debug_terminal_response.cmd = BOARD_SET_AX_ANGLE;
  debug_terminal_response.data_length = 0;

  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugTogglePump([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
  uint8_t pump_state;
  
	if(comm_enable){
  pump_state = 2*data[0] + data[1];
  switch (pump_state)
    {
      case 0: //open grab plank 1
				SPI_GPO.ResetOutputPin(SPI_POWER_OUTPUT_1);
        break;
      case 1: //close grab plank 1
				SPI_GPO.SetOutputPin(SPI_POWER_OUTPUT_1);
				break;
			case 2:
				SPI_GPO.ResetOutputPin(SPI_POWER_OUTPUT_2);
				break;
			case 3:
				SPI_GPO.SetOutputPin(SPI_POWER_OUTPUT_2);
				break;
			default:
				break;
    }
	}
  struct debug_terminal_response_s debug_terminal_response = {};
  debug_terminal_response.cmd = BOARD_TOGGLE_PUMP;
  debug_terminal_response.data_length = 0;
  
  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugGetKey([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
	if(comm_enable){
	}
	
 
  struct debug_terminal_response_s debug_terminal_response = {};

  debug_terminal_response.cmd = BOARD_GET_KEY;
  debug_terminal_response.data[0] = (robot_ctrl.starting_cord == STARTING_CORD_RELEASED)?1:0;
  debug_terminal_response.data_length = 1;

  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugGetToggle([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
  if(comm_enable){
	}
  struct debug_terminal_response_s debug_terminal_response = {};

  debug_terminal_response.cmd = BOARD_GET_TOGGLE;
  debug_terminal_response.data[0] = (robot_ctrl.playground_side == LEFT_SIDE)?1:0;
  debug_terminal_response.data_length = 1;

  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugGetDebugMessage([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{	
	LL_GPIO_TogglePin(GPIOD, PWM2_PD14);

	if(comm_enable){
	}
  struct debug_terminal_response_s debug_terminal_response = {};

  debug_terminal_response.cmd = BOARD_GET_DEBUG_MESSAGE;

  // Example message (must be <= 32 characters)
  const char* debug_msg = "Robot systems: all OK";

  // Copy to response buffer (pad with 0 if shorter)
  memset(debug_terminal_response.data, 0, 32);
  strncpy((char*)debug_terminal_response.data, debug_msg, 32);
  debug_terminal_response.data_length = 32;
  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugGetSwitchesState([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
	if(comm_enable){
	}
	
  struct debug_terminal_response_s debug_terminal_response = {};
  debug_terminal_response.cmd = BOARD_GET_LINEAR_SWITCHES_STATE;
  debug_terminal_response.data[0] = (robot_ctrl.linear_switch_1 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
  debug_terminal_response.data[1] = (robot_ctrl.linear_switch_2 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
	debug_terminal_response.data[2] = (robot_ctrl.linear_switch_3 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
	debug_terminal_response.data[3] = (robot_ctrl.linear_switch_4 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
  debug_terminal_response.data[4] = (robot_ctrl.linear_switch_5 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
	debug_terminal_response.data[5] = (robot_ctrl.linear_switch_6 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
	debug_terminal_response.data[6] = (robot_ctrl.linear_switch_7 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
	debug_terminal_response.data[7] = (robot_ctrl.linear_switch_8 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
	debug_terminal_response.data[8] = (robot_ctrl.linear_switch_9 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
  debug_terminal_response.data_length = 9;
  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugSetMaxonsState([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length){
	if(comm_enable){
	uint8_t state = data[0];
	
	if(state){
		Robot.MaxonsEnable();
	} else {
		Robot.MaxonsDisable();
	}
}
	struct debug_terminal_response_s debug_terminal_response = {};
	debug_terminal_response.cmd = BOARD_SET_MAXONS_STATE;
  debug_terminal_response.data_length = 0;
  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugSetCommState([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length){
	uint8_t state = data[0];
	
	if(state){
		comm_enable = 1;
	} else {
		comm_enable = 0;
	}
	
	struct debug_terminal_response_s debug_terminal_response = {};
	debug_terminal_response.cmd = BOARD_SET_COMM_STATE;
  debug_terminal_response.data_length = 0;
  Debug_Terminal.SendResponse(&debug_terminal_response);
	
}

void DebugTerminalCmdHandler::DebugBoardReset([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length){
	struct debug_terminal_response_s debug_terminal_response = {};
	debug_terminal_response.cmd = BOARD_RESET;
  debug_terminal_response.data_length = 0;
  Debug_Terminal.SendResponse(&debug_terminal_response);
	SCB->AIRCR = 0x05FA0004;
}

void DebugTerminalCmdHandler::DebugBoardGetReset([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
	if(comm_enable){
	}
	
  struct debug_terminal_response_s debug_terminal_response = {};

  debug_terminal_response.cmd = BOARD_GET_RESET;
  debug_terminal_response.data[0] = (robot_ctrl.reset_state == RESETED)?1:0;
  debug_terminal_response.data_length = 1;

  Debug_Terminal.SendResponse(&debug_terminal_response);
}

void DebugTerminalCmdHandler::DebugBoardClearReset([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
	if(comm_enable){
	}
	
  struct debug_terminal_response_s debug_terminal_response = {};

  debug_terminal_response.cmd = BOARD_CLEAR_RESET;
  robot_ctrl.reset_state = NOT_RESETED;
  debug_terminal_response.data_length = 0;

  Debug_Terminal.SendResponse(&debug_terminal_response);
}

/* BOARD_PCA9685_SET_SERVO_ANGLE
 * data[0] : i2c_addr_7bit (адрес платы PCA9685, например 0x40, 0x41, ...)
 * data[1] : channel (0..15)
 * data[2..3]: uint16_t angle_deg
 */
void DebugTerminalCmdHandler::DebugPCA9685_SetServoAngle([[maybe_unused]] uint8_t* data, [[maybe_unused]] uint8_t data_length)
{
  struct debug_terminal_response_s debug_terminal_response = {};
  int status = 0;

  if(comm_enable){
    uint8_t i2c_addr = data[0];
    uint8_t ch_raw = data[1];
    
    /* Проверка корректности канала */
    if (ch_raw > 15)
    {
      status = -1;
    }
    else
    {
      enum pca9685_channel_e ch = (enum pca9685_channel_e)ch_raw;
      uint16_t angle_deg;
      ByteTo_Word16(data + 2, &angle_deg);

      status = PCA9685.SetServoAngleWithAddr(i2c_addr, ch, angle_deg);
    }
  }

  debug_terminal_response.cmd = BOARD_PCA9685_SET_SERVO_ANGLE;
  debug_terminal_response.data[0] = (uint8_t)(status == 0 ? 0 : 1);  /* 0 = OK, 1 = Error */
  debug_terminal_response.data_length = 1;

  Debug_Terminal.SendResponse(&debug_terminal_response);
}

// C++ has no out-of-order array designators: the table is filled at compile time,
// so it still lands in .data without any startup code.
constinit const std::array<DebugTerminalCmdHandler::Handler, TERMINAL_CMD_TABLE_SIZE> DebugTerminalCmdHandler::handlers = [] {
  std::array<Handler, TERMINAL_CMD_TABLE_SIZE> t{};
  t[BOARD_Ping] = &DebugTerminalCmdHandler::DebugPing;
  t[BOARD_SetSpeed] = &DebugTerminalCmdHandler::DebugSetRobotSpeed;
  t[BOARD_GetSpeed] = &DebugTerminalCmdHandler::DebugGetRobotSpeed;
  t[BOARD_SetCoord] = &DebugTerminalCmdHandler::DebugSetRobotCoord;
  t[BOARD_GetCoord] = &DebugTerminalCmdHandler::DebugGetRobotCoord;
  t[BOARD_GetBatteryState] = &DebugTerminalCmdHandler::DebugGetBatteryState;
  t[BOARD_Set_BMS_CalibrationValue] = &DebugTerminalCmdHandler::DebugSet_BMS_CalibrationValue;
  t[BOARD_GET_AX_ANGLE] = &DebugTerminalCmdHandler::DebugGetAxAngle;
  t[BOARD_SET_AX_ANGLE] = &DebugTerminalCmdHandler::DebugSetAxAngle;
  t[BOARD_TOGGLE_PUMP] = &DebugTerminalCmdHandler::DebugTogglePump;
  t[BOARD_GET_KEY] = &DebugTerminalCmdHandler::DebugGetKey;
  t[BOARD_GET_TOGGLE] = &DebugTerminalCmdHandler::DebugGetToggle;
  t[BOARD_GET_DEBUG_MESSAGE] = &DebugTerminalCmdHandler::DebugGetDebugMessage;
  t[BOARD_GET_LINEAR_SWITCHES_STATE] = &DebugTerminalCmdHandler::DebugGetSwitchesState;
  t[BOARD_SET_MAXONS_STATE] = &DebugTerminalCmdHandler::DebugSetMaxonsState;
  t[BOARD_SET_COMM_STATE] = &DebugTerminalCmdHandler::DebugSetCommState;
  t[BOARD_RESET] = &DebugTerminalCmdHandler::DebugBoardReset;
  t[BOARD_GET_RESET] = &DebugTerminalCmdHandler::DebugBoardGetReset;
  t[BOARD_CLEAR_RESET] = &DebugTerminalCmdHandler::DebugBoardClearReset;
  t[BOARD_PCA9685_SET_SERVO_ANGLE] = &DebugTerminalCmdHandler::DebugPCA9685_SetServoAngle;
  return t;
}();

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
