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
#ifndef __TERMINAL_H
#define __TERMINAL_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.hpp"


#define TERMINAL_MAX_DATA_LENGTH    32


struct terminal_request_s{
    uint8_t cmd;
    uint8_t data_length;
    uint8_t data[TERMINAL_MAX_DATA_LENGTH];
};

struct terminal_response_s{
    uint8_t cmd;
    uint8_t data_length;
    uint8_t data[TERMINAL_MAX_DATA_LENGTH];
};

struct terminal_s{
    void (*SendResponse)(struct terminal_response_s* terminal_response);
    uint8_t (*GetRequest)(struct terminal_request_s* terminal_request);
};

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

struct debug_terminal_s{
    void (*DebugSendResponse)(struct debug_terminal_response_s* debug_terminal_response);
    uint8_t (*DebugGetRequest)(struct debug_terminal_request_s* debug_terminal_request);
};

extern const struct terminal_s Terminal;
extern const struct debug_terminal_s Debug_Terminal;



#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
