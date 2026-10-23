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
#ifndef __TERMINAL_CMD_HANDLER_H
#define __TERMINAL_CMD_HANDLER_H

#include <array>
#include "std_def.hpp"
#include "terminal_cmd_table.hpp"
#include "terminal.hpp"

/*  Commands of the main terminal (HL_RTU, USART1), see terminal_cmd_table.hpp  */
class TerminalCmdHandler{
public:
  /*  Run the handler of request->cmd; unknown commands are ignored  */
  void Handle(terminal_request_s* request);

  /*  0 blocks the commands that move the robot (end of the match)  */
  void SetCommEnable(uint8_t enable) { comm_enable = enable; }

private:
  using Handler = void (TerminalCmdHandler::*)(uint8_t* data, uint8_t data_length);
  static const std::array<Handler, TERMINAL_CMD_TABLE_SIZE> handlers;

  void Ping(uint8_t* data, uint8_t data_length);
  void SetRobotSpeed(uint8_t* data, uint8_t data_length);
  void GetRobotSpeed(uint8_t* data, uint8_t data_length);
  void SetRobotCoord(uint8_t* data, uint8_t data_length);
  void GetRobotCoord(uint8_t* data, uint8_t data_length);
  void GetBatteryState(uint8_t* data, uint8_t data_length);
  void Set_BMS_CalibrationValue(uint8_t* data, uint8_t data_length);
  void GetAxAngle(uint8_t* data, uint8_t data_length);
  void SetAxAngle(uint8_t* data, uint8_t data_length);
  void ResetWheelTicks(uint8_t* data, uint8_t data_length);
  void GetWheelTicks(uint8_t* data, uint8_t data_length);
  void GetToggle(uint8_t* data, uint8_t data_length);
  void GetDebugMessage(uint8_t* data, uint8_t data_length);
  void GetSwitchesState(uint8_t* data, uint8_t data_length);
  void SetMaxonsState(uint8_t* data, uint8_t data_length);
  void SetCommState(uint8_t* data, uint8_t data_length);
  void FreeWheels(uint8_t* data, uint8_t data_length);
  void BoardReset(uint8_t* data, uint8_t data_length);
  void BoardGetReset(uint8_t* data, uint8_t data_length);
  void BoardClearReset(uint8_t* data, uint8_t data_length);
  void grab_rotate(uint8_t* data, uint8_t data_length);
  void BlocksShift(uint8_t* data, uint8_t data_length);
  void BlocksRotateMask(uint8_t* data, uint8_t data_length);
  void RelativeMovePoll(uint8_t* data, uint8_t data_length);
  void GetMoveStatus(uint8_t* data, uint8_t data_length);
  void ToggleCursor(uint8_t* data, uint8_t data_length);
  void SideGrips(uint8_t* data, uint8_t data_length);
  void GripperLift(uint8_t* data, uint8_t data_length);
  void TogglePumps(uint8_t* data, uint8_t data_length);
  void ToggleValves(uint8_t* data, uint8_t data_length);
  void GetWheelPath(uint8_t* data, uint8_t data_length);
  void SimpleMove(uint8_t* data, uint8_t data_length);
  void RelativeMove(uint8_t* data, uint8_t data_length);
  void GetKey(uint8_t* data, uint8_t data_length);
  void GripperVertical(uint8_t* data, uint8_t data_length);
  void RotateWheelOnce(uint8_t* data, uint8_t data_length);
  void SlideGripper(uint8_t* data, uint8_t data_length);

  volatile uint8_t comm_enable = 1;
};

extern TerminalCmdHandler TerminalCmd;

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
