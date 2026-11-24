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

#define BOARD_IMPELLER_ON                   0x0D    /* [13] Turn the impeller on (if off) and set its PWM duty, % */
#define BOARD_IMPELLER_OFF                  0x0E    /* [14] Duty 0, then turn the impeller off */
#define BOARD_IMPELLER_GET_STATE            0x0F    /* [15] Read the impeller state and duty */
#define BOARD_IMPELLER_SET_DUTY             0x10    /* [16] Set the impeller PWM duty, %, the state is unchanged */

#define BOARD_TOGGLE_GRAIL_PICK_GRIPPER     0x11    /* [17] Toggle the PCA9685 CH13 servo between its closed and open angles */
#define BOARD_TOGGLE_GRAIL_SHELF_GRIPPER    0x12    /* [18] Toggle the PCA9685 CH14 servo between its closed and open angles */
#define BOARD_GRAIL_LIFT_UP                 0x13    /* [19] Raise the grail lift (Stepper 2) by N mm */
#define BOARD_GRAIL_LIFT_DOWN               0x14    /* [20] Lower the grail lift (Stepper 2) by N mm */

#define BOARD_GRIPPER_VERTICAL              0x15    /* [21] Gripper height (Stepper 1), mm from the power-on position */
#define BOARD_TOGGLE_CURSOR                 0x16    /* [22] Cursor dynamixel: 1 horizontal, 0 vertical */
#define BOARD_GRIPPER_SLIDE                 0x17    /* [23] Slide the gripper along the rack (pinion dynamixel), mm */

/*  Do not forget to update the table size with the last one command! */
#define TERMINAL_CMD_TABLE_SIZE             (BOARD_GRIPPER_SLIDE + 1)


#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
