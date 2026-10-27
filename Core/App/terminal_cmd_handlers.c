/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         functions.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "terminal_cmd_handlers.h" 
#include "dynamixel_master.h"
#include "byte2word.h"

#include "board_gpio_defines.h"
#include "stm32f4xx_ll_gpio.h"
#include "terminal.h"

#include "dynamixel_ax.h" 
#include "dynamixel_mx.h" 
#include "stm32f4xx_ll_usart.h"
#include "os_tasks.h"
#include "kinematics.h"
#include "odometry.h" 
#include "bms_board.h"
#include "bus_device.h"
#include "spi_gpo.h"
#include "spi_gpo_defines.h"


static void Ping(uint8_t* data, uint8_t data_length)
{
  LL_GPIO_TogglePin(GPIOD, PWM2_PD14);

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_Ping;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void AX_GetAngle(uint8_t* data, uint8_t data_length)
{
  uint16_t angle;
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_AX_GetAngle;
  terminal_response.data[0] = Dynamixel_AX.GetAngle(USART3, data[0], &angle);
  Word16_ToByte(&angle, &terminal_response.data[1]);
  terminal_response.data_length = 3;

  Terminal.SendResponse(&terminal_response);
}

static void AX_SetAngle(uint8_t* data, uint8_t data_length)
{
//  LL_GPIO_TogglePin(GPIOD, PWM1_PD15);
  uint8_t id = data[0];
  uint16_t angle;
  
  ByteTo_Word16(data+1,&angle);

  Dynamixel_AX.TorqueEnable(USART3,id);
  Dynamixel_AX.SetAngle(USART3,id,angle);

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_AX_SetAngle;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void SetRobotSpeed(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct speed_s speed;
  
  ByteTo_Word32(data,&speed.dx);
  ByteTo_Word32(data+4,&speed.dy);
  ByteTo_Word32(data+8,&speed.dphi);

  Odometry.SetSpeed(&speed);

  terminal_response.cmd = BOARD_SetSpeed;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void GetRobotSpeed(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct speed_s speed;

  Odometry.GetSpeed(&speed);

  Word32_ToByte(&speed.dx,terminal_response.data);
  Word32_ToByte(&speed.dy,terminal_response.data+4);
  Word32_ToByte(&speed.dphi,terminal_response.data+8);

  terminal_response.cmd = BOARD_GetSpeed;
  terminal_response.data_length = 12;

  Terminal.SendResponse(&terminal_response);
}

static void SetRobotCoord(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct coord_s coord;
  
  ByteTo_Word32(data,&coord.x);
  ByteTo_Word32(data+4,&coord.y);
  ByteTo_Word32(data+8,&coord.phi);

  Odometry.SetCoord(&coord);

  terminal_response.cmd = BOARD_SetCoord;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void GetRobotCoord(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct coord_s coord;

  Odometry.GetCoord(&coord);

  Word32_ToByte(&coord.x,terminal_response.data);
  Word32_ToByte(&coord.y,terminal_response.data+4);
  Word32_ToByte(&coord.phi,terminal_response.data+8);

  terminal_response.cmd = BOARD_GetCoord;
  terminal_response.data_length = 12;
  
  Terminal.SendResponse(&terminal_response);
}

static void GetBatteryState(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct bms_battery_state_s battery_state;
  BMS.GetBatteryState(bus_devices[BMS_BOARD].usart, bus_devices[BMS_BOARD].id, &battery_state);
  memcpy(terminal_response.data, &battery_state, 18);

  terminal_response.cmd = BOARD_GetBatteryState;
  terminal_response.data_length = 18;
  
  Terminal.SendResponse(&terminal_response);
}

static void Set_BMS_CalibrationValue(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  uint16_t calibration_value;
  ByteTo_Word16(data+1, &calibration_value);

  BMS.SetCalibrationValue(bus_devices[BMS_BOARD].usart, bus_devices[BMS_BOARD].id, data[0], calibration_value);
  
  terminal_response.cmd = BOARD_Set_BMS_CalibrationValue;
  terminal_response.data_length = 0;
  
  Terminal.SendResponse(&terminal_response);
}

static void ToggleGripper(uint8_t* data, uint8_t data_length)
{
  extern uint8_t gripper_state;
  struct terminal_response_s terminal_response;

  gripper_state = data[0];
  vTaskResume(GripperTask.GetTaskHandle());

  // switch (data[0])
  // {
  //   case 0:
  //     Dynamixel_AX.SetAngle(bus_devices[GRIPPER_TOP].usart, bus_devices[GRIPPER_TOP].id, 0);
  //     Dynamixel_AX.SetAngle(bus_devices[GRIPPER_BOTTOM].usart, bus_devices[GRIPPER_BOTTOM].id, 0);
  //     break;
  //   case 1:
  //     Dynamixel_AX.SetAngle(bus_devices[GRIPPER_TOP].usart, bus_devices[GRIPPER_TOP].id, 100);
  //     Dynamixel_AX.SetAngle(bus_devices[GRIPPER_BOTTOM].usart, bus_devices[GRIPPER_BOTTOM].id, 100);
  //     break;
  // }
  
  terminal_response.cmd = BOARD_TOGGLE_GRIPPER;
  terminal_response.data_length = 0;
  
  Terminal.SendResponse(&terminal_response);
}

static void PumpOn(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_1|SPI_POWER_OUTPUT_2|SPI_POWER_OUTPUT_3|SPI_POWER_OUTPUT_4);

  terminal_response.cmd = BOARD_POWER_ON_PUMP;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void PumpOff(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_1|SPI_POWER_OUTPUT_2|SPI_POWER_OUTPUT_3|SPI_POWER_OUTPUT_4);

  terminal_response.cmd = BOARD_POWER_OFF_PUMP;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}





void (*terminal_cmd_handlers[TERMINAL_CMD_TABLE_SIZE])(uint8_t* data, uint8_t data_length) = {[BOARD_Ping] = Ping,
                                                                                              [BOARD_AX_GetAngle] = AX_GetAngle,
                                                                                              [BOARD_AX_SetAngle] = AX_SetAngle,
                                                                                              [BOARD_SetSpeed] = SetRobotSpeed,
                                                                                              [BOARD_GetSpeed] = GetRobotSpeed,
                                                                                              [BOARD_SetCoord] = SetRobotCoord,
                                                                                              [BOARD_GetCoord] = GetRobotCoord,
                                                                                              [BOARD_GetBatteryState] = GetBatteryState,
                                                                                              [BOARD_Set_BMS_CalibrationValue] = Set_BMS_CalibrationValue,
                                                                                              [BOARD_TOGGLE_GRIPPER] = ToggleGripper,
                                                                                              [BOARD_POWER_ON_PUMP] = PumpOn,
                                                                                              [BOARD_POWER_OFF_PUMP] = PumpOff
                                                                                            };

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
