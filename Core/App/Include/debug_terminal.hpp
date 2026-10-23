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
#ifndef __DEBUG_TERMINAL_H
#define __DEBUG_TERMINAL_H

#include "std_def.hpp"
#include "terminal.hpp"
#include "serial.hpp"

struct debug_terminal_request_s{
    uint8_t cmd;
    uint8_t data_length;
    uint8_t data[TERMINAL_MAX_DATA_LENGTH];
};

struct debug_terminal_response_s{
    uint8_t cmd;
    uint8_t data_length;
    uint8_t data[TERMINAL_MAX_DATA_LENGTH];
};

/*  Debug terminal: fixed-size packets over Serial on UART5  */
class DebugTerminalModule{
public:
    constexpr explicit DebugTerminalModule(SerialDriver& serial) : serial(serial) {}

    void SendResponse(debug_terminal_response_s* debug_terminal_response);
    /*  Waits up to 100 ms; returns 1 if a full request was received  */
    uint8_t GetRequest(debug_terminal_request_s* debug_terminal_request);

private:
    SerialDriver& serial;
};

extern DebugTerminalModule Debug_Terminal;

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
