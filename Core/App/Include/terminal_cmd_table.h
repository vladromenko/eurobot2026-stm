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
#define BOARD_AX_GetAngle            	      0X02    /* Command Description  */
#define BOARD_AX_SetAngle                   0X03    /* Command Description  */

#define BOARD_SetSpeed            	        0X04    /* Command Description  */
#define BOARD_GetSpeed                      0X05    /* Command Description  */
#define BOARD_SetCoord            	        0X06    /* Command Description  */
#define BOARD_GetCoord                      0X07    /* Command Description  */
#define BOARD_GetBatteryState               0X08    /* Command Description  */
#define BOARD_Set_BMS_CalibrationValue      0X09    /* Command Description  */

#define BOARD_TOGGLE_GRIPPER                0X0A    /* Command Description  */
#define BOARD_POWER_ON_PUMP                 0X0B    /* Command Description  */
#define BOARD_POWER_OFF_PUMP                0X0C    /* Command Description  */
#define BOARD_STEPPER_TEST                  0x0D

/* Odometry calibration, used by tools/odom_calibrate.py */
#define BOARD_SET_MAXONS_STATE              0x0E    /* [state]: 0 - stop and free wheels, 1 - enable -> [status] */
#define BOARD_GET_WHEEL_TICKS               0x0F    /* -> [status, int32 ticks1, ticks2, ticks3] */
#define BOARD_ROTATE_WHEEL                  0x10    /* [wheel 1..3, int32 ticks, int32 tpm] -> [status] */
#define BOARD_GET_WHEEL_ROTATION            0x11    /* -> [enum wheel_rotation_state_e, int32 ticks done] */
#define BOARD_GET_ODOMETRY_GEOMETRY         0x12    /* -> [float radius, center_to_wheel, gear_ratio, ticks_per_turn] */
#define BOARD_SET_ODOMETRY_CALIBRATION      0x13    /* [float wheel1, wheel2, wheel3, x, y, phi] -> [status] */
#define BOARD_GET_ODOMETRY_CALIBRATION      0x14    /* -> [float wheel1, wheel2, wheel3, x, y, phi] */


/*  Do not forget to update the table size with the last one command! */
#define TERMINAL_CMD_TABLE_SIZE             (BOARD_GET_ODOMETRY_CALIBRATION + 1)


#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
