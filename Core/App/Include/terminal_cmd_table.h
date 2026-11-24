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
	 



#define BOARD_PING                          0x00    /* [0]  Link check, empty response */
#define BOARD_AX_GET_ANGLE                  0x02    /* [2]  AX dynamixel on USART3: id -> bus error, angle deg (uint16) */
#define BOARD_AX_SET_ANGLE                  0x03    /* [3]  AX dynamixel on USART3: id, angle deg (uint16); enables torque */

#define BOARD_SET_SPEED                     0x04    /* [4]  Robot speed: dx, dy m/s, dphi rad/s (3 x float) */
#define BOARD_GET_SPEED                     0x05    /* [5]  Robot speed from the wheel encoders: dx, dy, dphi (3 x float) */
#define BOARD_SET_COORD                     0x06    /* [6]  Odometry coordinates: x, y m, phi rad (3 x float) */
#define BOARD_GET_COORD                     0x07    /* [7]  Odometry coordinates: x, y, phi (3 x float) */
#define BOARD_GET_BATTERY_STATE             0x08    /* [8]  BMS state, 18 bytes: power, cell voltages, voltage, current, charge, cells */
#define BOARD_SET_BMS_CALIBRATION_VALUE     0x09    /* [9]  BMS calibration: index, value (uint16) */

#define BOARD_TOGGLE_GRIPPER                0x0A    /* [10] Top and bottom gripper dynamixels: 0 close, 1 open-close cycle */
#define BOARD_POWER_ON_PUMP                 0x0B    /* [11] Turn on the pumps on power outputs 1-4 */
#define BOARD_POWER_OFF_PUMP                0x0C    /* [12] Turn off the pumps on power outputs 1-4 */

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

#define BOARD_GET_IMU                       0x18    /* [24] Last MPU-6050 sample: accel g, gyro deg/s, temperature */

/*  Do not forget to update the table size with the last one command! */
#define TERMINAL_CMD_TABLE_SIZE             (BOARD_GET_IMU + 1)


#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
