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
#include "hw_instance.h"
#include "stepper.h"
#include "impeller.h"

extern uint8_t gripper_state;
extern volatile uint8_t gripper_cmd;
extern volatile uint8_t gripper_cmd_status;
extern uint8_t gripper_height;
extern uint8_t cursor_pos;
extern int32_t rack_distance_mm;
extern volatile uint8_t grail_pick_toggle;
extern volatile uint8_t grail_shelf_toggle;
extern volatile uint8_t grail_lift_cmd;
extern volatile uint8_t grail_lift_step;
extern volatile uint8_t grail_lift_status;
extern volatile uint8_t grail_lift_ready;


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
  const struct hw_instance_s* hw = GetHwInstance();
  uint16_t angle;
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_AX_GetAngle;
  terminal_response.data[0] = hw->dynamixel_ax->GetAngle(USART3, data[0], &angle);
  Word16_ToByte(&angle, &terminal_response.data[1]);
  terminal_response.data_length = 3;

  Terminal.SendResponse(&terminal_response);
}

static void AX_SetAngle(uint8_t* data, uint8_t data_length)
{
  const struct hw_instance_s* hw = GetHwInstance();
//  LL_GPIO_TogglePin(GPIOD, PWM1_PD15);
  uint8_t id = data[0];
  uint16_t angle;
  
  ByteTo_Word16(data+1,&angle);

  hw->dynamixel_ax->TorqueEnable(USART3,id);
  hw->dynamixel_ax->SetAngle(USART3,id,angle);

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
  const struct hw_instance_s* hw = GetHwInstance();
  struct terminal_response_s terminal_response;

  struct bms_battery_state_s battery_state;
  hw->bms->GetBatteryState(hw->bus_devices[BMS_BOARD].usart, hw->bus_devices[BMS_BOARD].id, &battery_state);
  memcpy(terminal_response.data, &battery_state, 18);

  terminal_response.cmd = BOARD_GetBatteryState;
  terminal_response.data_length = 18;
  
  Terminal.SendResponse(&terminal_response);
}

static void Set_BMS_CalibrationValue(uint8_t* data, uint8_t data_length)
{
  const struct hw_instance_s* hw = GetHwInstance();
  struct terminal_response_s terminal_response;

  uint16_t calibration_value;
  ByteTo_Word16(data+1, &calibration_value);

  hw->bms->SetCalibrationValue(hw->bus_devices[BMS_BOARD].usart, hw->bus_devices[BMS_BOARD].id, data[0], calibration_value);
  
  terminal_response.cmd = BOARD_Set_BMS_CalibrationValue;
  terminal_response.data_length = 0;
  
  Terminal.SendResponse(&terminal_response);
}

static void ToggleGripper(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  if(data_length >= 1 && gripper_cmd == GRIPPER_CMD_NONE){
    gripper_state = data[0];
    gripper_cmd = GRIPPER_CMD_TOGGLE;
  }

  terminal_response.cmd = BOARD_TOGGLE_GRIPPER;
  terminal_response.data_length = 0;
  
  Terminal.SendResponse(&terminal_response);
}

static void PumpOn(uint8_t* data, uint8_t data_length)
{
  const struct hw_instance_s* hw = GetHwInstance();
  struct terminal_response_s terminal_response;

  hw->spi_gpo->SetOutputPin(SPI_POWER_OUTPUT_1|SPI_POWER_OUTPUT_2|SPI_POWER_OUTPUT_3|SPI_POWER_OUTPUT_4);

  terminal_response.cmd = BOARD_POWER_ON_PUMP;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void PumpOff(uint8_t* data, uint8_t data_length)
{
  const struct hw_instance_s* hw = GetHwInstance();
  struct terminal_response_s terminal_response;

  hw->spi_gpo->ResetOutputPin(SPI_POWER_OUTPUT_1|SPI_POWER_OUTPUT_2|SPI_POWER_OUTPUT_3|SPI_POWER_OUTPUT_4);

  terminal_response.cmd = BOARD_POWER_OFF_PUMP;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}


static void SendImpellerResponse(uint8_t cmd, uint8_t status)
{
  struct terminal_response_s terminal_response;

  terminal_response.cmd = cmd;
  terminal_response.data[0] = status;
  terminal_response.data[1] = (uint8_t)Impeller.GetState();
  terminal_response.data[2] = Impeller.GetDuty();
  terminal_response.data[3] = Impeller.GetTargetDuty();
  terminal_response.data_length = 4;
  Terminal.SendResponse(&terminal_response);
}

static void ImpellerOn(uint8_t* data, uint8_t data_length)
{
  /*
   *  data[0] - PWM duty, 0..100 % (limited by IMPELLER_MAX_DUTY_PCT); no data - keep the set duty.
   *  Turns the impeller on if it is off, otherwise only changes the duty.
   *  Response: [status, state, duty on the pin %, set duty %].
   */
  uint8_t status = 0;

  if((data_length == 0) || (data[0] <= 100)){
    if(data_length >= 1)
      Impeller.SetDuty(data[0]);
    Impeller.TurnOn();
    status = 1;
  }

  SendImpellerResponse(BOARD_IMPELLER_ON, status);
}

static void ImpellerOff(uint8_t* data, uint8_t data_length)
{
  (void)data;
  (void)data_length;

  Impeller.TurnOff();
  SendImpellerResponse(BOARD_IMPELLER_OFF, 1);
}

static void ImpellerGetState(uint8_t* data, uint8_t data_length)
{
  /*  Response: [1, state, duty on the pin %, set duty %]  */
  (void)data;
  (void)data_length;

  SendImpellerResponse(BOARD_IMPELLER_GET_STATE, 1);
}

static void ImpellerSetDuty(uint8_t* data, uint8_t data_length)
{
  /*
   *  data[0] - PWM duty, 0..100 % (limited by IMPELLER_MAX_DUTY_PCT).
   *  Applied at once while starting or on, otherwise used by the next turn on. No press.
   *  Response: [status, state, duty on the pin %, set duty %].
   */
  uint8_t status = 0;

  if((data_length >= 1) && (data[0] <= 100)){
    Impeller.SetDuty(data[0]);
    status = 1;
  }

  SendImpellerResponse(BOARD_IMPELLER_SET_DUTY, status);
}

/* data[0] - travel, mm. Response after the move: 1 - moved, 0 - fault, timeout, bottom switch or busy */
static void GrailLiftStep(uint8_t cmd, uint8_t direction, uint8_t* data, uint8_t data_length)
{
  uint8_t status = 0;
  // Rejected while the lift is homing at startup
  if(data_length == 1 && grail_lift_ready && !grail_lift_cmd){
    grail_lift_step = data[0];
    grail_lift_cmd = direction;

    // grail_gripper_task gives up after 10 s, so the loop ends before this limit
    TickType_t start_time = xTaskGetTickCount();
    while(grail_lift_cmd && (xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(11000)){
      vTaskDelay(10);
    }
    if(!grail_lift_cmd){
      status = grail_lift_status;
    }
  }

  struct terminal_response_s terminal_response;
  terminal_response.cmd = cmd;
  terminal_response.data[0] = status;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}

static void GrailLiftUp(uint8_t* data, uint8_t data_length)
{
  GrailLiftStep(BOARD_GRAIL_LIFT_UP, GRAIL_LIFT_UP, data, data_length);
}

static void GrailLiftDown(uint8_t* data, uint8_t data_length)
{
  GrailLiftStep(BOARD_GRAIL_LIFT_DOWN, GRAIL_LIFT_DOWN, data, data_length);
}

static void ToggleGrailPickGripper(uint8_t* data, uint8_t data_length)
{
  grail_pick_toggle = 1;

  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_TOGGLE_GRAIL_PICK_GRIPPER;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}

static void ToggleGrailShelfGripper(uint8_t* data, uint8_t data_length)
{
  grail_shelf_toggle = 1;

  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_TOGGLE_GRAIL_SHELF_GRIPPER;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}

/* Runs one gripper_task request and waits for it. Response: 1 - done, 0 - failed, timeout or busy */
static void RunGripperCmd(uint8_t terminal_cmd, uint8_t cmd, uint32_t timeout_ms)
{
  uint8_t status = 0;

  if(gripper_cmd == GRIPPER_CMD_NONE){
    gripper_cmd = cmd;

    TickType_t start_time = xTaskGetTickCount();
    while(gripper_cmd != GRIPPER_CMD_NONE && (xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(timeout_ms)){
      vTaskDelay(10);
    }
    if(gripper_cmd == GRIPPER_CMD_NONE){
      status = gripper_cmd_status;
    }
  }

  struct terminal_response_s terminal_response;
  terminal_response.cmd = terminal_cmd;
  terminal_response.data[0] = status;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}

static void GripperVertical(uint8_t* data, uint8_t data_length)
{
  /*  data[0] - height, mm from the power-on position (Stepper 1)  */
  if(data_length >= 1 && gripper_cmd == GRIPPER_CMD_NONE)
    gripper_height = data[0];
  RunGripperCmd(BOARD_GRIPPER_VERTICAL, (data_length >= 1) ? GRIPPER_CMD_VERTICAL : GRIPPER_CMD_NONE, 5000);
}

static void ToggleCursor(uint8_t* data, uint8_t data_length)
{
  /*  data[0] - 1 horizontal, 0 vertical  */
  if(data_length >= 1 && gripper_cmd == GRIPPER_CMD_NONE)
    cursor_pos = data[0];
  RunGripperCmd(BOARD_TOGGLE_CURSOR, (data_length >= 1) ? GRIPPER_CMD_CURSOR : GRIPPER_CMD_NONE, 2000);
}

static void SlideGripper(uint8_t* data, uint8_t data_length)
{
  /*  data[0..3] - distance along the rack, mm (int32) from the start position  */
  if(data_length >= 4 && gripper_cmd == GRIPPER_CMD_NONE)
    ByteTo_Word32(data, &rack_distance_mm);
  RunGripperCmd(BOARD_GRIPPER_SLIDE, (data_length >= 4) ? GRIPPER_CMD_SLIDE : GRIPPER_CMD_NONE, 2000);
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
                                                                                              [BOARD_POWER_OFF_PUMP] = PumpOff,
                                                                                              [BOARD_IMPELLER_ON] = ImpellerOn,
                                                                                              [BOARD_IMPELLER_OFF] = ImpellerOff,
                                                                                              [BOARD_IMPELLER_GET_STATE] = ImpellerGetState,
                                                                                              [BOARD_IMPELLER_SET_DUTY] = ImpellerSetDuty,
                                                                                              [BOARD_TOGGLE_GRAIL_PICK_GRIPPER] = ToggleGrailPickGripper,
                                                                                              [BOARD_TOGGLE_GRAIL_SHELF_GRIPPER] = ToggleGrailShelfGripper,
                                                                                              [BOARD_GRAIL_LIFT_UP] = GrailLiftUp,
                                                                                              [BOARD_GRAIL_LIFT_DOWN] = GrailLiftDown,
                                                                                              [BOARD_GRIPPER_VERTICAL] = GripperVertical,
                                                                                              [BOARD_TOGGLE_CURSOR] = ToggleCursor,
                                                                                              [BOARD_GRIPPER_SLIDE] = SlideGripper
                                                                                            };

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
