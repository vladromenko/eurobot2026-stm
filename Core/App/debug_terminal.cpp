#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.hpp"
#include "robot_gpio_defines.hpp"
#include "byte2word.hpp" 
#include "maxon.hpp" 
#include "dynamixel_ax.hpp" 
#include "serial.hpp" 
#include "terminal.hpp" 


static void DebugSendResponse(struct debug_terminal_response_s* debug_terminal_response)
{
    Serial.WriteBytes((uint8_t*)debug_terminal_response,
                      (uint32_t)sizeof(struct debug_terminal_response_s));
}
static uint8_t DebugGetRequest(struct debug_terminal_request_s* debug_terminal_request)
{
    uint32_t n = Serial.ReadBytes((uint8_t*)debug_terminal_request,
                                  100);
    return (n == sizeof(struct debug_terminal_request_s)) ? 1 : 0;
}



const struct debug_terminal_s Debug_Terminal = {.DebugSendResponse = DebugSendResponse,
                                    .DebugGetRequest = DebugGetRequest};
