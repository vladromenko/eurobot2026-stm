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

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.h"
#include "terminal_cmd_table.h" 



extern void (*terminal_cmd_handlers[TERMINAL_CMD_TABLE_SIZE])(uint8_t* data, uint8_t data_length);




#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
