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

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.hpp"
#include "terminal_cmd_table.hpp" 

typedef void (*terminal_cmd_handler_t)(uint8_t* data, uint8_t data_length);



extern std::array<terminal_cmd_handler_t, TERMINAL_CMD_TABLE_SIZE> terminal_cmd_handlers;
extern std::array<terminal_cmd_handler_t, TERMINAL_CMD_TABLE_SIZE> debug_terminal_cmd_handlers;



#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
