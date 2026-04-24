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
//#define BOARD_SET_STEPPER_POSITION          0X0C    /* [12] set the position of the stepper  */
#define BOARD_RESET_WHEEL_TICKS             0X0D    /* [13] reset wheel ticks */
#define BOARD_GET_WHEEL_TICKS               0X0E    /* [14] reset wheel ticks */
#define BOARD_TOGGLE_PUMP										0X0F 		/* Command Description  */
//#define BOARD_GET_KEY                       0X11		/* Command Description  */
#define BOARD_GET_TOGGLE                    0X12		/* Command Description  */
#define BOARD_GET_DEBUG_MESSAGE             0X13		/* Command Description  */
#define BOARD_GET_LINEAR_SWITCHES_STATE     0X14		/* Command Description  */
#define BOARD_SET_MAXONS_STATE							0X15		/* Command Description  */
#define BOARD_SET_COMM_STATE								0X16		/* Command Description  */
#define BOARD_RESET													0X17		/* Command Description  */
#define BOARD_GET_RESET											0X18		/* Command Description  */
#define BOARD_CLEAR_RESET										0X19		/* Command Description  */

#define BOARD_GRAB_ROTATE									  0X1C		/* [28] Command Description  */
/* PCA9685 (I2C2, 0x40) debug commands */
#define BOARD_PCA9685_SET_SERVO_ANGLE       0x1D    /* [29]Set servo angle: i2c_addr, ch, angle_deg */
#define BOARD_TOGGLE_CURSOR                 0x1E    /* [30] Change the state of the cursor dynamixel */

/* Microcommands for BT */
#define BOARD_SIDE_GRIPS                    0x1F    /* [31] Control the angle of the side grips */
#define BOARD_GRIPPER_LIFT                  0x20    /* [32] Control the angle of the gripper's dynamixels */
#define BOARD_TOGGLE_PUMPS									0X21 		/* [33] Turn on/off pumpes  */
#define BOARD_TOGGLE_VALVES                 0x22		/* [34] Turn on/off valves */
#define BOARD_GET_WHEEL_PATH                0x23		/* [35] get linear dist of wheel */
#define BOARD_MOVE                          0x25		/* [37] Move the robot to grab blocks */
#define BOARD_GET_KEY                       0X26		/* [38] Check the state of the starting cord  */
#define BOARD_GRIPPER_VERTICAL              0X27		/* [39] Control the gripper's height  */
#define BOARD_ROTATE_WHEEL_ONCE             0x28    /* [40] Rotate a wheel by one full revolution for calibration */
#define BOARD_MOVE_RELATIVE                 0x29    /* [41] Relative move in robot frame by dx/dy */
#define BOARD_FREE_WHEELS                   0x2A    /* [42] Disable wheel holding for manual odometry tests */
#define BOARD_BLOCKS_SHIFT                  0x2B    /* [43] Spread or close block holders */
#define BOARD_BLOCKS_ROTATE_MASK            0x2C    /* [44] Rotate blocks according to mask */

/*  Do not forget to update the table size with the last one command! */
#define TERMINAL_CMD_TABLE_SIZE             (BOARD_BLOCKS_ROTATE_MASK + 1)


#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
