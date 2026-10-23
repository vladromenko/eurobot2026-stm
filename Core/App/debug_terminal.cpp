/**
  ******************************************************************************
  * @file    debug_terminal.cpp
  * @brief   Debug terminal: fixed-size packets over Serial on UART5.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "debug_terminal.hpp" 
#include "serial.hpp" 

constinit DebugTerminalModule Debug_Terminal;


void DebugTerminalModule::SendResponse(debug_terminal_response_s* debug_terminal_response)
{
    Serial.WriteBytes((uint8_t*)debug_terminal_response,
                      (uint32_t)sizeof(debug_terminal_response_s));
}

uint8_t DebugTerminalModule::GetRequest(debug_terminal_request_s* debug_terminal_request)
{
    uint32_t n = Serial.ReadBytes((uint8_t*)debug_terminal_request,
                                  100);
    return (n == sizeof(debug_terminal_request_s)) ? 1 : 0;
}
