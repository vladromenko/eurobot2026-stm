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
#include "debug_terminal.h"

#include "dynamixel_ax.h" 
#include "dynamixel_mx.h" 
#include "stm32f4xx_ll_usart.h"
#include "os_tasks.h"
#include "kinematics.h"
#include "odometry.h" 
#include "bms_board.h"
#include "bus_device.h"
#include "robot_behavior_routine.h"
#include "spi_gpo.h" 
#include "spi_gpo_defines.h" 
#include "rc_servo.h" 
#include "stepper.h"
#include "pca9685.h"

#include "FreeRTOS.h"
#include "task.h"

// Глобальная переменная, чтобы gripper_task мог отправить сюда сигнал
volatile TaskHandle_t xTerminalTaskHandle = NULL; 

extern uint16_t dynamixel_speed;

extern uint8_t i2c_addr_1;

// cursor params
extern uint8_t dynamixel_cursor_id;
extern uint16_t cursor_speed;
extern uint8_t cursor_hor_angle;
extern uint8_t cursor_vert_angle;
extern uint8_t cursor_target_angle;
extern uint8_t cursor_cmd;

extern uint8_t grips_cmd;

extern uint8_t cmd_isFinished;
extern uint8_t gripper_cmd;
extern uint8_t lift_cmd;
extern uint8_t gripper_id;

extern uint8_t pump_mask;
extern uint8_t valve_mask;
extern uint8_t block_mask;

extern uint16_t current_angle;
extern uint8_t dir;
extern uint8_t move_cmd_speed;
extern uint8_t move_cmd_time;

extern uint8_t last_cmd_status;

uint8_t COMM_ENABLE = 1;

static void WaitForGripperCompletion(void)
{
  // 1. Сохраняем handle текущей задачи (TerminalTask), чтобы Gripper знал, кому отвечать
  xTerminalTaskHandle = xTaskGetCurrentTaskHandle();

  // 3. Ждем сигнала от gripper_task.
  // Задача переходит в состояние BLOCKED и не тратит CPU.
  // portMAX_DELAY - ждать вечно. Можно поставить таймаут (напр. 5000), если боишься зависания.
  ulTaskNotifyTake(pdTRUE, 5000);
  
  // Как только мы здесь — значит задача гриппера выполнила работу.
}


static void Ping(uint8_t* data, uint8_t data_length)
{
  LL_GPIO_TogglePin(GPIOD, PWM2_PD14);

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_Ping;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void SetRobotSpeed(uint8_t* data, uint8_t data_length)
{
	
	if(COMM_ENABLE){
		struct speed_s speed;
  
  ByteTo_Word32(data,&speed.dx);
  ByteTo_Word32(data+4,&speed.dy);
  ByteTo_Word32(data+8,&speed.dphi);

  Odometry.SetSpeed(&speed);

	}
  struct terminal_response_s terminal_response;

  
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
	if(COMM_ENABLE){
		struct coord_s coord;
  
  ByteTo_Word32(data,&coord.x);
  ByteTo_Word32(data+4,&coord.y);
  ByteTo_Word32(data+8,&coord.phi);

  Odometry.SetCoord(&coord);
	}
  struct terminal_response_s terminal_response;

  

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

static void GetAxAngle(uint8_t* data, uint8_t data_length)
{
  uint16_t angle;
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_GET_AX_ANGLE;
  terminal_response.data[0] = Dynamixel_AX.GetAngle(USART3, data[0], &angle);
  Word16_ToByte(&angle, &terminal_response.data[1]);
  terminal_response.data_length = 3;

  Terminal.SendResponse(&terminal_response);
}

static void SetAxAngle(uint8_t* data, uint8_t data_length)
{
	if(COMM_ENABLE){
  uint8_t id = data[0];
  uint16_t angle;
  ByteTo_Word16(data+1,&angle);
	uint16_t speed;
  ByteTo_Word16(data+3,&speed);

	int16_t curr_speed = -1;
	while(curr_speed != speed){
		Dynamixel_AX.SetMovingSpeed(USART3, id, speed);
		Dynamixel_AX.GetMovingSpeed(USART3, id, &curr_speed);
	}
	
  Dynamixel_AX.TorqueEnable(USART3,id);
  Dynamixel_AX.SetAngle(USART3,id,angle);
}
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_SET_AX_ANGLE;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}
/*
static void GetKey(uint8_t* data, uint8_t data_length)
{
  uint8_t key_request = data[0];
  uint8_t key_response = (robot_ctrl.starting_cord == STARTING_CORD_ATTACHED) ? 1 : 0;

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_GET_KEY;
  terminal_response.data[0] = key_response;
  terminal_response.data_length = 1;

  Terminal.SendResponse(&terminal_response);
}
  */

static void GetToggle(uint8_t* data, uint8_t data_length)
{
  if(COMM_ENABLE){
	}
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_GET_TOGGLE;
  terminal_response.data[0] = (robot_ctrl.playground_side == LEFT_SIDE)?1:0;
  terminal_response.data_length = 1;

  Terminal.SendResponse(&terminal_response);
}

static void GetDebugMessage(uint8_t* data, uint8_t data_length)
{
	if(COMM_ENABLE){
	}
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_GET_DEBUG_MESSAGE;

  // Example message (must be <= 32 characters)
  const char* debug_msg = "Robot systems: all OK";

  // Copy to response buffer (pad with 0 if shorter)
  memset(terminal_response.data, 0, 32);
  strncpy((char*)terminal_response.data, debug_msg, 32);
  terminal_response.data_length = 32;
  Terminal.SendResponse(&terminal_response);
}

static void GetSwitchesState(uint8_t* data, uint8_t data_length)
{
	if(COMM_ENABLE){
	}
	
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_GET_LINEAR_SWITCHES_STATE;
  terminal_response.data[0] = (robot_ctrl.linear_switch_1 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
  terminal_response.data[1] = (robot_ctrl.linear_switch_2 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
	terminal_response.data[2] = (robot_ctrl.linear_switch_3 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
	terminal_response.data[3] = (robot_ctrl.linear_switch_4 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
  terminal_response.data[4] = (robot_ctrl.linear_switch_5 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
	terminal_response.data[5] = (robot_ctrl.linear_switch_6 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
	terminal_response.data[6] = (robot_ctrl.linear_switch_7 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
	terminal_response.data[7] = (robot_ctrl.linear_switch_8 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
	terminal_response.data[8] = (robot_ctrl.linear_switch_9 == LINEAR_SWITCH_PRESSED) ? 1 : 0;
  terminal_response.data_length = 9;
  Terminal.SendResponse(&terminal_response);
}

static void SetMaxonsState(uint8_t* data, uint8_t data_length){
	if(COMM_ENABLE){
	uint8_t state = data[0];
	
	if(state){
		MaxonsEnable();
	} else {
		MaxonsDisable();
	}
}
	struct terminal_response_s terminal_response;
	terminal_response.cmd = BOARD_SET_MAXONS_STATE;
  terminal_response.data_length = 0;
  Terminal.SendResponse(&terminal_response);
}

static void SetCommState(uint8_t* data, uint8_t data_length){
	uint8_t state = data[0];
	
	if(state){
		COMM_ENABLE = 1;
	} else {
		COMM_ENABLE = 0;
	}
	
	struct terminal_response_s terminal_response;
	terminal_response.cmd = BOARD_SET_COMM_STATE;
  terminal_response.data_length = 0;
  Terminal.SendResponse(&terminal_response);
	
}

static void BoardReset(uint8_t* data, uint8_t data_length){
	struct terminal_response_s terminal_response;
	terminal_response.cmd = BOARD_RESET;
  terminal_response.data_length = 0;
  Terminal.SendResponse(&terminal_response);
	SCB->AIRCR = 0x05FA0004;
}

static void BoardGetReset(uint8_t* data, uint8_t data_length)
{
	if(COMM_ENABLE){
	}
	
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_GET_RESET;
  terminal_response.data[0] = (robot_ctrl.reset_state == RESETED)?1:0;
  terminal_response.data_length = 1;

  Terminal.SendResponse(&terminal_response);
}

static void BoardClearReset(uint8_t* data, uint8_t data_length)
{
	if(COMM_ENABLE){
	}
	
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_CLEAR_RESET;
  robot_ctrl.reset_state = NOT_RESETED;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void grab_rotate(uint8_t* data, uint8_t data_length)
{
  if(COMM_ENABLE){
    ulTaskNotifyTake(pdTRUE, 0); 
  
    gripper_id = data[0];
    block_mask = data[1];
    cmd_isFinished = 0; 
    gripper_cmd = 5;
	}

  struct terminal_response_s terminal_response = {0};
  terminal_response.cmd = BOARD_GRAB_ROTATE;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}

static void ToggleCursor(uint8_t* data, uint8_t data_length)
{
  if(COMM_ENABLE){
    // Clear old notify flags to prevent false execution
    ulTaskNotifyTake(pdTRUE, 0); 
    // Point this FreeRTOS task as a reciever
    xTerminalTaskHandle = xTaskGetCurrentTaskHandle();
    
    cursor_cmd = data[0];
    gripper_cmd = 0;

    WaitForGripperCompletion();
	}

  // terminal response
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_TOGGLE_CURSOR;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
  
}

static void SideGrips(uint8_t* data, uint8_t data_length)
{
  if(COMM_ENABLE){
    // Clear old notify flags to prevent false execution
    ulTaskNotifyTake(pdTRUE, 0); 
    // Point this FreeRTOS task as a reciever
    xTerminalTaskHandle = xTaskGetCurrentTaskHandle();
    
    gripper_id = data[0];
    grips_cmd = data[1];
    gripper_cmd = 1;

    WaitForGripperCompletion();
	}

  // terminal response
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_SIDE_GRIPS;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}

static void GripperLift(uint8_t* data, uint8_t data_length)
{
  if(COMM_ENABLE){
    // Clear old notify flags to prevent false execution
    ulTaskNotifyTake(pdTRUE, 0); 
    // Point this FreeRTOS task as a reciever
    xTerminalTaskHandle = xTaskGetCurrentTaskHandle();
    
    gripper_id = data[0];
    lift_cmd = data[1];
    gripper_cmd = 2;

    WaitForGripperCompletion();
	}

  // terminal response
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_GRIPPER_LIFT;
  //terminal_response.data[0] = last_cmd_status;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}

static void TogglePumps(uint8_t* data, uint8_t data_length)
{
  if(COMM_ENABLE){
    // Clear old notify flags to prevent false execution
    ulTaskNotifyTake(pdTRUE, 0); 
    // Point this FreeRTOS task as a reciever
    xTerminalTaskHandle = xTaskGetCurrentTaskHandle();
    
    gripper_id = data[0];
    pump_mask = data[1];
    gripper_cmd = 3;

    WaitForGripperCompletion();
	}
  
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_TOGGLE_PUMPS;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}

static void ToggleValves(uint8_t* data, uint8_t data_length)
{
  if(COMM_ENABLE){
    // Clear old notify flags to prevent false execution
    ulTaskNotifyTake(pdTRUE, 0); 
    // Point this FreeRTOS task as a reciever
    xTerminalTaskHandle = xTaskGetCurrentTaskHandle();
    
    gripper_id = data[0];
    valve_mask = data[1];
    gripper_cmd = 4;

    WaitForGripperCompletion();
  }

  // terminal response
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_TOGGLE_VALVES;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}

/* doesn't work response (need to check odometry or use FreeRTOS timer)
static void SimpleMove(uint8_t* data, uint8_t data_length)
{
  // Clear old notify flags to prevent false execution
  ulTaskNotifyTake(pdTRUE, 0); 
  // Point this FreeRTOS task as a reciever
  xTerminalTaskHandle = xTaskGetCurrentTaskHandle();

  dir = data[0];
  gripper_cmd = 6;

  WaitForGripperCompletion();

  // terminal response
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_MOVE;
  terminal_response.data[0] = (uint8_t)0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}
*/

/*
static void SimpleMove(uint8_t* data, uint8_t data_length)
{
  ulTaskNotifyTake(pdTRUE, 0); 
  
  dir = data[0];
  cmd_isFinished = 0; 
  gripper_cmd = 6;

  struct terminal_response_s terminal_response = {0};
  terminal_response.cmd = BOARD_MOVE;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}
*/

static void SimpleMove(uint8_t* data, uint8_t data_length)
{
  if(COMM_ENABLE){
    ulTaskNotifyTake(pdTRUE, 0);
    
    gripper_id = data[0];
    dir = data[1];
    move_cmd_speed = data[2];
    move_cmd_time = data[3];
    
    cmd_isFinished = 0; 
    gripper_cmd = 6;
  }

  struct terminal_response_s terminal_response = {0};
  terminal_response.cmd = BOARD_MOVE;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}

static void GetKey(uint8_t* data, uint8_t data_length)
{
  uint8_t key_request = data[0];
  uint8_t key_response = (robot_ctrl.starting_cord == STARTING_CORD_ATTACHED) ? 1 : 0;

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_GET_KEY;
  terminal_response.data[0] = key_response;
  terminal_response.data_length = 1;

  Terminal.SendResponse(&terminal_response);
}




void (*terminal_cmd_handlers[TERMINAL_CMD_TABLE_SIZE])(uint8_t* data, uint8_t data_length) = {[BOARD_Ping] = Ping,
                                                                                              [BOARD_SetSpeed] = SetRobotSpeed,
                                                                                              [BOARD_GetSpeed] = GetRobotSpeed,
                                                                                              [BOARD_SetCoord] = SetRobotCoord,
                                                                                              [BOARD_GetCoord] = GetRobotCoord,
                                                                                              [BOARD_GetBatteryState] = GetBatteryState,
                                                                                              [BOARD_Set_BMS_CalibrationValue] = Set_BMS_CalibrationValue,
																																															[BOARD_GET_AX_ANGLE] = GetAxAngle,
                                                                                              [BOARD_SET_AX_ANGLE] = SetAxAngle,
																																															//[BOARD_GET_KEY] = GetKey,
																																															[BOARD_GET_TOGGLE] = GetToggle,
                                                                                              [BOARD_GET_DEBUG_MESSAGE] = GetDebugMessage,
                                                                                              [BOARD_GET_LINEAR_SWITCHES_STATE] = GetSwitchesState,
																																															[BOARD_SET_MAXONS_STATE] = SetMaxonsState,
																																															[BOARD_SET_COMM_STATE] = SetCommState,
																																															[BOARD_RESET] = BoardReset,
																																															[BOARD_GET_RESET] = BoardGetReset,
																																															[BOARD_CLEAR_RESET] = BoardClearReset,
                                                                                              [BOARD_GRAB_ROTATE] = grab_rotate,
                                                                                              [BOARD_TOGGLE_CURSOR] = ToggleCursor,
                                                                                              [BOARD_SIDE_GRIPS] = SideGrips,
                                                                                              [BOARD_GRIPPER_LIFT] = GripperLift,
                                                                                              [BOARD_TOGGLE_PUMPS] = TogglePumps,
                                                                                              [BOARD_TOGGLE_VALVES] = ToggleValves,
                                                                                              [BOARD_MOVE] = SimpleMove,
                                                                                              [BOARD_GET_KEY] = GetKey
                                                                                              
                                                                                             

};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
