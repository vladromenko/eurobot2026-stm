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
#ifndef __TERMINAL_CMD_TABLE_H
#define __TERMINAL_CMD_TABLE_H

#ifdef __cplusplus
 extern "C" {
#endif
	 



#define BOARD_Ping            	            0X00    /* Command Description  */

#define BOARD_SetSpeed            	        0X04    /* Command Description  */
#define BOARD_GetSpeed                      0X05    /* Command Description  */
#define BOARD_SetCoord            	        0X06    /* Command Description  */
#define BOARD_GetCoord                      0X07    /* Command Description  */
#define BOARD_GetBatteryState               0X08    /* Command Description  */
#define BOARD_Set_BMS_CalibrationValue      0X09    /* Command Description  */

#define BOARD_GET_AX_ANGLE                	0X0A    /* Command Description  */
#define BOARD_SET_AX_ANGLE                	0X0B    /* Command Description  */
#define BOARD_SET_STEPPER_ZERO              0X0C    /* Command Description  */
#define BOARD_SET_STEPPER_POSITION          0X0D    /* Command Description  */
#define BOARD_GET_STEPPER_POSITION 					0X0E 		/* Command Description  */
#define BOARD_TOGGLE_PUMP										0X0F 		/* Command Description  */
#define BOARD_SET_MAGNET_ANGLE							0X10		/* Command Description  */
#define BOARD_GET_KEY                       0X11		/* Command Description  */
#define BOARD_GET_TOGGLE                    0X12		/* Command Description  */
#define BOARD_GET_DEBUG_MESSAGE             0X13		/* Command Description  */
#define BOARD_GET_LINEAR_SWITCHES_STATE     0X14		/* Command Description  */
#define BOARD_SET_MAXONS_STATE							0X15		/* Command Description  */
#define BOARD_SET_COMM_STATE								0X16		/* Command Description  */
#define BOARD_RESET													0X17		/* Command Description  */
#define BOARD_GET_RESET											0X18		/* Command Description  */
#define BOARD_CLEAR_RESET										0X19		/* Command Description  */

#define BOARD_GRAB_ON										    0X1A		/* Command Description  */
#define BOARD_GRAB_OFF										  0X1B		/* Command Description  */
#define BOARD_GRAB_ROTATE									  0X1C		/* Command Description  */
/* PCA9685 (I2C2, 0x40) debug commands */
#define BOARD_PCA9685_SET_SERVO_ANGLE       0x1D    /* Set servo angle: i2c_addr, ch, angle_deg */



/*  Do not forget to update the table size with the last one command! */
#define TERMINAL_CMD_TABLE_SIZE             (BOARD_PCA9685_SET_SERVO_ANGLE + 1)


#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
