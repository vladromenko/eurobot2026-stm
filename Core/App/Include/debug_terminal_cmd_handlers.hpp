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
#ifndef __DEBUG_TERMINAL_CMD_HANDLER_H
#define __DEBUG_TERMINAL_CMD_HANDLER_H

#include <array>
#include "std_def.hpp"
#include "terminal_cmd_table.hpp"
#include "debug_terminal.hpp"

/*  Commands of the debug terminal (Serial, UART5); codes from terminal_cmd_table.hpp  */
class DebugTerminalCmdHandler{
public:
  /*  Run the handler of request->cmd; unknown commands are ignored  */
  void Handle(debug_terminal_request_s* request);

  /*  0 blocks the commands that move the robot (end of the match)  */
  void SetCommEnable(uint8_t enable) { comm_enable = enable; }

private:
  using Handler = void (DebugTerminalCmdHandler::*)(uint8_t* data, uint8_t data_length);
  static const std::array<Handler, TERMINAL_CMD_TABLE_SIZE> handlers;

  void DebugPing(uint8_t* data, uint8_t data_length);
  void DebugSetRobotSpeed(uint8_t* data, uint8_t data_length);
  void DebugGetRobotSpeed(uint8_t* data, uint8_t data_length);
  void DebugSetRobotCoord(uint8_t* data, uint8_t data_length);
  void DebugGetRobotCoord(uint8_t* data, uint8_t data_length);
  void DebugGetBatteryState(uint8_t* data, uint8_t data_length);
  void DebugSet_BMS_CalibrationValue(uint8_t* data, uint8_t data_length);
  void DebugGetAxAngle(uint8_t* data, uint8_t data_length);
  void DebugSetAxAngle(uint8_t* data, uint8_t data_length);
  void DebugTogglePump(uint8_t* data, uint8_t data_length);
  void DebugGetKey(uint8_t* data, uint8_t data_length);
  void DebugGetToggle(uint8_t* data, uint8_t data_length);
  void DebugGetDebugMessage(uint8_t* data, uint8_t data_length);
  void DebugGetSwitchesState(uint8_t* data, uint8_t data_length);
  void DebugSetMaxonsState(uint8_t* data, uint8_t data_length);
  void DebugSetCommState(uint8_t* data, uint8_t data_length);
  void DebugBoardReset(uint8_t* data, uint8_t data_length);
  void DebugBoardGetReset(uint8_t* data, uint8_t data_length);
  void DebugBoardClearReset(uint8_t* data, uint8_t data_length);
  void DebugPCA9685_SetServoAngle(uint8_t* data, uint8_t data_length);

  volatile uint8_t comm_enable = 1;
};

extern DebugTerminalCmdHandler DebugTerminalCmd;

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
