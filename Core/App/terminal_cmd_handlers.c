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

extern int16_t stepper1_goal_position;
extern int16_t stepper2_goal_position;
extern int16_t stepper3_goal_position;

extern uint8_t stepper1_calibrate;
extern uint8_t stepper2_calibrate;
extern uint8_t stepper3_calibrate;

// from gripper_task
/*
extern 	uint8_t dynamixel_id_1;
extern	uint8_t dynamixel_id_2;
extern	uint16_t dynamixel_speed;
extern	uint8_t start_angle;
extern	uint8_t end_angle;

extern	uint8_t servo5_start_angle;
extern	uint8_t servo5_end_angle;
extern	uint8_t servo6_start_angle;
extern	uint8_t servo6_end_angle;
*/

uint8_t dynamixel_id_1 = 98;
uint8_t dynamixel_id_2 = 112;
extern uint16_t dynamixel_speed;
extern uint8_t start_angle;
extern uint8_t end_angle;

extern uint8_t servo5_start_angle;
extern uint8_t servo5_end_angle;
extern uint8_t servo6_start_angle;
extern uint8_t servo6_end_angle;

extern uint8_t i2c_addr_1;

// cursor params
extern uint8_t dynamixel_cursor_id;
extern uint16_t cursor_speed;
extern uint8_t cursor_hor_angle;
extern uint8_t cursor_vert_angle;
extern uint8_t cursor_target_angle;
extern uint8_t cursor_cmd;

// Side dynamixels of the gripper
uint8_t side_dynamixel_id_1 = 13;
uint8_t side_dynamixel_id_2 = 14;
extern uint16_t side_dynamixel_speed;
extern uint8_t side_off_angle_1;
extern uint8_t side_on_angle_1;
extern uint8_t side_off_angle_2;
extern uint8_t side_on_angle_2;
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

/*
uint8_t dynamixel_id_1 = 98;
uint8_t dynamixel_id_2 = 112;
uint16_t dynamixel_speed = 150; //0-1024
uint8_t start_angle = 62; //0-300 60
uint8_t end_angle = 110; //0-300 150 125

uint8_t servo5_start_angle = 10; // left shift servo 90 170
uint8_t servo5_end_angle = 110;
uint8_t servo6_start_angle = 120; // right shift servo
uint8_t servo6_end_angle = 30;

uint8_t i2c_addr_1 = 0x40; // I2C address of the PWM driver

uint8_t dynamixel_cursor_id = 15;
uint16_t cursor_speed = 150; //0-1024
uint8_t cursor_hor_angle = 40; //0-300 60
uint8_t cursor_vert_angle = 130; //0-300 150 125

// Side dynamixels of the gripper
uint8_t side_dynamixel_id_1 = 13;
uint8_t side_dynamixel_id_2 = 14;
uint16_t side_dynamixel_speed = 300; //0-1024
uint8_t side_off_angle_1 = 69; //0-300 44 99
uint8_t side_on_angle_1 = 140; //0-300 132 140
uint8_t side_off_angle_2 = 127; //0-300 152 97
uint8_t side_on_angle_2 = 51; //0-300 59 51

uint16_t current_angle = 0;
*/

uint8_t COMM_ENABLE = 1;

/**
 * @brief Rotate two Dynamixels by a given angle synchronously.
 * @param usart USART bus of the Dynamixels
 * @param id_1 Dynamixel ID 1
 * @param id_2 Dynamixel ID 2
 * @param angle Goal angle in deg (0-300)
 */
static void RotateGripper(USART_TypeDef* usart, uint8_t id_1, uint8_t id_2, uint16_t angle)
{
    // Dynamixels ID
    uint8_t device_ids[] = {id_1, id_2};
    uint8_t num_devices = sizeof(device_ids) / sizeof(device_ids[0]); // universal counter of devices

    // Transform angle from deg to Goal Position Register (0-1023)
    if (angle > 300) angle = 300; // max angle limit
    uint16_t goal_position_value = (uint16_t)(((float)angle * 1023.0f) / 300.0f);

    // Prepare data for SyncWrite
    // 2 bytes for each servo (Low byte (LB), then High byte (HB))
    uint8_t data_to_write[4];

    // Data for 1st Dynamixel
    data_to_write[0] = (uint8_t)(goal_position_value & 0xFF); // LB
    data_to_write[1] = (uint8_t)(goal_position_value >> 8);  // HB

    // Data for 2nd Dynamixel
    data_to_write[2] = (uint8_t)(goal_position_value & 0xFF); // LB
    data_to_write[3] = (uint8_t)(goal_position_value >> 8);  // HB

    // Call the SyncWrite command
    DynamixelMaster.SyncWrite(usart, device_ids, num_devices, AX_GoalPosition, data_to_write, 2, NULL);
}

/**
 * @brief Rotate two Dynamixels by a given angle synchronously.
 * @param usart USART bus of the Dynamixels
 * @param id_1 Dynamixel ID 1
 * @param id_2 Dynamixel ID 2
 * @param angle_1 Goal angle in deg (0-300) for Dynamixel_1
 * @param angle_2 Goal angle in deg (0-300) for Dynamixel_2
 */
static void RotateTwoDynamixels(USART_TypeDef* usart, uint8_t id_1, uint8_t id_2, uint16_t angle_1, uint16_t angle_2)
{
    // Dynamixels ID
    uint8_t device_ids[] = {id_1, id_2};
    uint8_t num_devices = 2;
    //uint8_t num_devices = sizeof(device_ids) / sizeof(device_ids[0]); // universal counter of devices

    // Transform angle from deg to Goal Position Register (0-1023)
    if (angle_1 > 300) angle_1 = 300; // max angle limit
    if (angle_2 > 300) angle_2 = 300;
    uint16_t goal_position_value_1 = (uint16_t)(((float)angle_1 * 1023.0f) / 300.0f);
    uint16_t goal_position_value_2 = (uint16_t)(((float)angle_2 * 1023.0f) / 300.0f);

    // Prepare data for SyncWrite
    // 2 bytes for each servo (Low byte (LB), then High byte (HB))
    uint8_t data_to_write[4];

    // Data for 1st Dynamixel
    data_to_write[0] = (uint8_t)(goal_position_value_1 & 0xFF); // LB
    data_to_write[1] = (uint8_t)(goal_position_value_1 >> 8);  // HB

    // Data for 2nd Dynamixel
    data_to_write[2] = (uint8_t)(goal_position_value_2 & 0xFF); // LB
    data_to_write[3] = (uint8_t)(goal_position_value_2 >> 8);  // HB

    // Call the SyncWrite command
    DynamixelMaster.SyncWrite(usart, device_ids, num_devices, AX_GoalPosition, data_to_write, 2, NULL);
}

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

static void SetStepperZero(uint8_t* data, uint8_t data_length)
{
	
	uint8_t id = data[0];
  if(COMM_ENABLE){
	switch(id){
		case 0:
			stepper1_calibrate = 1;
			break;
		case 1:
			stepper2_calibrate = 1;
			break;
		case 2:
			stepper3_calibrate = 1;
			break;
		default:
			break;
	}
}
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_SET_STEPPER_ZERO;
	
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void GetStepperPosition(uint8_t* data, uint8_t data_length)
{
  int16_t position;
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_GET_STEPPER_POSITION;
	
	switch(data[0]){
		case 0:
			position = Stepper.GetPresentPosisition(STEPPER_MOTOR_1);
			break;
		case 1:
			position = Stepper.GetPresentPosisition(STEPPER_MOTOR_2);
			break;
		case 2:
			position = Stepper.GetPresentPosisition(STEPPER_MOTOR_3);
			break;
		default:
			break;
	}
	
  Word16_ToByte(&position, &terminal_response.data[0]);
  terminal_response.data_length = 2;

  Terminal.SendResponse(&terminal_response);
}

static void SetStepperPosition(uint8_t* data, uint8_t data_length)
{
	if(COMM_ENABLE){
  uint8_t id = data[0];
  int16_t position;
  
  ByteTo_Word16(data+1,&position);
	if(position >=0){
		switch(id){
			case 0:
				if(!stepper1_calibrate){
					stepper1_goal_position = position;
					Stepper.StepToPosisition(STEPPER_MOTOR_1, position, 32, 16, 800);
				}
				break;
			case 1:
				if(!stepper2_calibrate){
					stepper2_goal_position = position;
					Stepper.StepToPosisition(STEPPER_MOTOR_2, position, 32, 16, 800);
				}
				break;
			case 2:
				if(!stepper3_calibrate){
					stepper3_goal_position = position;
					Stepper.StepToPosisition(STEPPER_MOTOR_3, position, 32, 16, 800);
				}
				break;
			default:
				break;
		}
	}
}
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_SET_STEPPER_POSITION;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void TogglePump(uint8_t* data, uint8_t data_length)
{
  uint8_t pump_state;
  
	if(COMM_ENABLE){
  pump_state = 2*data[0] + data[1];
  switch (pump_state)
    {
      case 0: //open grab plank 1
				SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_1);
        break;
      case 1: //close grab plank 1
				SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_1);
				break;
			case 2:
				SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_2);
				break;
			case 3:
				SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_2);
				break;
			default:
				break;
    }
	}
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_TOGGLE_PUMP;
  terminal_response.data_length = 0;
  
  Terminal.SendResponse(&terminal_response);
}

static void SetMagnetAngle(uint8_t* data, uint8_t data_length)
{
	if(COMM_ENABLE){
  uint8_t magnet_id = data[0];
	int16_t position;
  ByteTo_Word16(data+1,&position);
	
  
 
  switch (magnet_id)
    {
      case 0:
				RC_Servo.Enable(RC_SERVO_1);
				RC_Servo.SetServoAngle(RC_SERVO_1, 525, 2500, 180, position);
        break;
      case 1:
				RC_Servo.Enable(RC_SERVO_2);
				RC_Servo.SetServoAngle(RC_SERVO_2, 525, 2500, 180, position);
				break;
			case 2:
				RC_Servo.Enable(RC_SERVO_3);
				RC_Servo.SetServoAngle(RC_SERVO_3, 525, 2500, 180, position);
        break;
      case 3:
				RC_Servo.Enable(RC_SERVO_4);
				RC_Servo.SetServoAngle(RC_SERVO_4, 525, 2500, 180, position);
        break;
			default:
				break;
    }
	}
	
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_SET_MAGNET_ANGLE;
  terminal_response.data_length = 0;
  
  Terminal.SendResponse(&terminal_response);
}

static void GetKey(uint8_t* data, uint8_t data_length)
{
	if(COMM_ENABLE){
	}
	
 
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_GET_KEY;
  terminal_response.data[0] = (robot_ctrl.starting_cord == STARTING_CORD_RELEASED)?1:0;
  terminal_response.data_length = 1;

  Terminal.SendResponse(&terminal_response);
}

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


static void grab_on(uint8_t* data, uint8_t data_length)
{
  uint8_t gripper_state = data[0];

  Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_1, dynamixel_speed);
	Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_2, dynamixel_speed);
  Dynamixel_AX.SetMovingSpeed(USART3, side_dynamixel_id_1, side_dynamixel_speed);
  Dynamixel_AX.SetMovingSpeed(USART3, side_dynamixel_id_2, side_dynamixel_speed);

  struct speed_s speed;

  if(gripper_state == 1) {

    Dynamixel_AX.TorqueEnable(USART3, dynamixel_id_1);
    Dynamixel_AX.TorqueEnable(USART3, dynamixel_id_2);
    Dynamixel_AX.TorqueEnable(USART3, side_dynamixel_id_1);
    Dynamixel_AX.TorqueEnable(USART3, side_dynamixel_id_2);

    speed.dphi = 0;
    speed.dx = 0;
    speed.dy = 0;
    Odometry.SetSpeed(&speed);
    vTaskDelay(500);

    RotateTwoDynamixels(USART3, side_dynamixel_id_1, side_dynamixel_id_2, side_off_angle_1, side_off_angle_2);
    vTaskDelay(1000);

    SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_3 | SPI_POWER_OUTPUT_4); // turn on valves
    SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_1 | SPI_POWER_OUTPUT_2); // turn on pumps
    vTaskDelay(500);


    PCA9685.SetServoAngleWithAddr(i2c_addr_1, PCA9685_CH1, 0);
    vTaskDelay(50);
    PCA9685.SetServoAngleWithAddr(i2c_addr_1, PCA9685_CH2, 0);
    vTaskDelay(50);
    PCA9685.SetServoAngleWithAddr(i2c_addr_1, PCA9685_CH3, 0);
    vTaskDelay(50);
    PCA9685.SetServoAngleWithAddr(i2c_addr_1, PCA9685_CH4, 0);
    vTaskDelay(500);

    PCA9685.SetServoAngleWithAddr(i2c_addr_1, PCA9685_CH5, servo5_start_angle);
    vTaskDelay(500);
    PCA9685.SetServoAngleWithAddr(i2c_addr_1, PCA9685_CH6, servo6_start_angle);
    vTaskDelay(500);
    
    /*
    RC_Servo.SetServoAngle(RC_SERVO_1, 500, 2500, 180, 0);
    RC_Servo.SetServoAngle(RC_SERVO_2, 500, 2500, 180, 0);
    RC_Servo.SetServoAngle(RC_SERVO_3, 500, 2500, 180, 0);
    RC_Servo.SetServoAngle(RC_SERVO_4, 500, 2500, 180, 0);
    vTaskDelay(500);
    RC_Servo.SetServoAngle(RC_SERVO_5, 500, 2500, 180, servo5_start_angle);
    RC_Servo.SetServoAngle(RC_SERVO_6, 500, 2500, 180, servo6_start_angle);
    vTaskDelay(500);
    */
  

    RotateGripper(USART3, dynamixel_id_1, dynamixel_id_2, start_angle);
    vTaskDelay(500);
    
    RotateTwoDynamixels(USART3, side_dynamixel_id_1, side_dynamixel_id_2, side_on_angle_1, side_on_angle_2);
    vTaskDelay(2000);
    RotateTwoDynamixels(USART3, side_dynamixel_id_1, side_dynamixel_id_2, side_off_angle_1, side_off_angle_2);
    vTaskDelay(1000);

    speed.dphi = 0;
    speed.dx = 0.1f;
    speed.dy = 0;
    Odometry.SetSpeed(&speed);
    vTaskDelay(2000);
/*
    speed.dphi = 0;
    speed.dx = 0;
    speed.dy = 0;
    Odometry.SetSpeed(&speed);
    vTaskDelay(500);

    RotateTwoDynamixels(USART3, side_dynamixel_id_1, side_dynamixel_id_2, side_on_angle_1, side_on_angle_2);
    vTaskDelay(2000);
    RotateTwoDynamixels(USART3, side_dynamixel_id_1, side_dynamixel_id_2, side_off_angle_1, side_off_angle_2);
    vTaskDelay(1000);

    speed.dphi = 0;
    speed.dx = 0.025f;
    speed.dy = 0;
    Odometry.SetSpeed(&speed);
    vTaskDelay(3000);
*/
    speed.dphi = 0;
    speed.dx = 0;
    speed.dy = 0;
    Odometry.SetSpeed(&speed);
    vTaskDelay(500);
    /*
    speed.dphi = 0;
    speed.dx = 0.05f;
    speed.dy = 0;
    Odometry.SetSpeed(&speed);
    vTaskDelay(1000);
    */

    speed.dphi = 0;
    speed.dx = 0;
    speed.dy = 0;
    Odometry.SetSpeed(&speed);
    vTaskDelay(500);

    RotateTwoDynamixels(USART3, side_dynamixel_id_1, side_dynamixel_id_2, side_on_angle_1, side_on_angle_2);
    vTaskDelay(1000);

    RotateGripper(USART3, dynamixel_id_1, dynamixel_id_2, end_angle);
    vTaskDelay(1000);

    speed.dphi = 0;
    speed.dx = -0.05f;
    speed.dy = 0;
    Odometry.SetSpeed(&speed);
    vTaskDelay(1000);

    speed.dphi = 0;
    speed.dx = 0;
    speed.dy = 0;
    Odometry.SetSpeed(&speed);
    vTaskDelay(500);
  }

  // terminal response (acknowledge flag)
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_GRAB_ON;
  terminal_response.data[0] = 1;

  Dynamixel_AX.GetAngle(USART3, dynamixel_id_1, &current_angle);
  while (current_angle < end_angle)
  {
    vTaskDelay(50);
    Dynamixel_AX.GetAngle(USART3, side_dynamixel_id_1, &current_angle);
  }
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;

  Terminal.SendResponse(&terminal_response);
  
}

static void grab_off(uint8_t* data, uint8_t data_length)
{
  uint8_t gripper_state = data[0];

  Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_1, dynamixel_speed);
	Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_2, dynamixel_speed);
  Dynamixel_AX.SetMovingSpeed(USART3, side_dynamixel_id_1, side_dynamixel_speed);
  Dynamixel_AX.SetMovingSpeed(USART3, side_dynamixel_id_2, side_dynamixel_speed);

  struct speed_s speed;

  if(gripper_state == 1) {

    Dynamixel_AX.TorqueEnable(USART3, dynamixel_id_1);
    Dynamixel_AX.TorqueEnable(USART3, dynamixel_id_2);
    Dynamixel_AX.TorqueEnable(USART3, side_dynamixel_id_1);
    Dynamixel_AX.TorqueEnable(USART3, side_dynamixel_id_2);
    
    speed.dphi = 0;
    speed.dx = 0;
    speed.dy = 0;
    Odometry.SetSpeed(&speed);
    vTaskDelay(500);

    // join all blocks
    PCA9685.SetServoAngleWithAddr(i2c_addr_1, PCA9685_CH5, servo5_start_angle);
    vTaskDelay(500);
    PCA9685.SetServoAngleWithAddr(i2c_addr_1, PCA9685_CH6, servo6_start_angle);
    vTaskDelay(500);

    RotateGripper(USART3, dynamixel_id_1, dynamixel_id_2, start_angle);
    vTaskDelay(500);
    
    RotateTwoDynamixels(USART3, side_dynamixel_id_1, side_dynamixel_id_2, side_on_angle_1, side_on_angle_2);
    vTaskDelay(1000);
    RotateTwoDynamixels(USART3, side_dynamixel_id_1, side_dynamixel_id_2, side_off_angle_1, side_off_angle_2);
    vTaskDelay(1000);

    SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_3 | SPI_POWER_OUTPUT_4); // turn off valves
    SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_1 | SPI_POWER_OUTPUT_2); // turn off pumps
    vTaskDelay(500);

    speed.dphi = 0;
    speed.dx = -0.05f;
    speed.dy = 0;
    Odometry.SetSpeed(&speed);
    vTaskDelay(1000);

    speed.dphi = 0;
    speed.dx = 0;
    speed.dy = 0;
    Odometry.SetSpeed(&speed);
    vTaskDelay(500);

    RotateGripper(USART3, dynamixel_id_1, dynamixel_id_2, end_angle);
    vTaskDelay(1000);

    PCA9685.SetServoAngleWithAddr(i2c_addr_1, PCA9685_CH1, 0);
    vTaskDelay(50);
    PCA9685.SetServoAngleWithAddr(i2c_addr_1, PCA9685_CH2, 0);
    vTaskDelay(50);
    PCA9685.SetServoAngleWithAddr(i2c_addr_1, PCA9685_CH3, 0);
    vTaskDelay(50);
    PCA9685.SetServoAngleWithAddr(i2c_addr_1, PCA9685_CH4, 0);
    vTaskDelay(500);
  }
  
  // terminal response (acknowledge flag)
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_GRAB_OFF;
  terminal_response.data[0] = 1;

  /*
  vTaskDekay(1000);
  Dynamixel_AX.GetAngle(USART3, side_dynamixel_id_1, &current_angle);
  if (current_angle == side_off_angle_1 + 10) terminal_response.data[0] = 0x01;
  */

  Dynamixel_AX.GetAngle(USART3, dynamixel_id_1, &current_angle);
  while (current_angle < end_angle)
  {
    vTaskDelay(50);
    Dynamixel_AX.GetAngle(USART3, side_dynamixel_id_1, &current_angle);
  }
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;

  Terminal.SendResponse(&terminal_response);
}



/*
static void grab_rotate(uint8_t* data, uint8_t data_length)
{
  uint8_t gripper_state = data[0];
  uint8_t blocks_rotation = data[1];

  Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_1, dynamixel_speed);
	Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_2, dynamixel_speed);
  
  if(gripper_state == 1) {

    switch (blocks_rotation)
    {
      case 0:
        PCA9685.SetServoAngleWithAddr(i2c_addr_1, 5, servo5_start_angle);
        vTaskDelay(500);
        PCA9685.SetServoAngleWithAddr(i2c_addr_1, 6, servo6_start_angle);
        vTaskDelay(500);
        break;
      case 1:
        PCA9685.SetServoAngleWithAddr(i2c_addr_1, 5, servo5_end_angle); // move the block #1 apart
        vTaskDelay(500);
        PCA9685.SetServoAngleWithAddr(i2c_addr_1, 6, servo6_end_angle); // move the block #3 apart
        vTaskDelay(500);
        PCA9685.SetServoAngleWithAddr(i2c_addr_1, 1, 180); // flip the block #1
        vTaskDelay(500);
				break;
			case 2:
        PCA9685.SetServoAngleWithAddr(i2c_addr_1, 5, servo5_end_angle); // move the block #1 apart
        vTaskDelay(500);
        PCA9685.SetServoAngleWithAddr(i2c_addr_1, 6, servo6_end_angle); // move the block #3 apart
        vTaskDelay(500);
        PCA9685.SetServoAngleWithAddr(i2c_addr_1, 2, 180); // flip the block #2
        vTaskDelay(500);
        break;
      case 3:
        PCA9685.SetServoAngleWithAddr(i2c_addr_1, 5, servo5_end_angle); // move the block #1 apart
        vTaskDelay(500);
        PCA9685.SetServoAngleWithAddr(i2c_addr_1, 6, servo6_end_angle); // move the block #3 apart
        vTaskDelay(500);
        PCA9685.SetServoAngleWithAddr(i2c_addr_1, 3, 180); // flip the block #3
        vTaskDelay(500);
        break;
      case 4:
        PCA9685.SetServoAngleWithAddr(i2c_addr_1, 5, servo5_end_angle); // move the block #1 apart
        vTaskDelay(500);
        PCA9685.SetServoAngleWithAddr(i2c_addr_1, 6, servo6_end_angle); // move the block #3 apart
        vTaskDelay(500);
        PCA9685.SetServoAngleWithAddr(i2c_addr_1, 4, 180); // flip the block #4
        vTaskDelay(500);
        break;
			default:
				break;
    }

    PCA9685.SetServoAngleWithAddr(i2c_addr_1, 5, servo5_start_angle);
    vTaskDelay(500);
    PCA9685.SetServoAngleWithAddr(i2c_addr_1, 6, servo6_start_angle);
    vTaskDelay(500);
  }
}
*/

/*
static void grab_rotate(uint8_t* data, uint8_t data_length)
{  
  // 1. СНАЧАЛА очищаем старые флаги, чтобы не сработать ложно
  ulTaskNotifyTake(pdTRUE, 0); 
  // 2. Указываем себя получателем
  xTerminalTaskHandle = xTaskGetCurrentTaskHandle();
  
  block_mask = data[0];
  gripper_cmd = 5;

  // ВМЕСТО ЦИКЛА WHILE:
  WaitForGripperCompletion();
  
  // Check this segment of terminal response
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_GRAB_ROTATE;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
  
}
*/

static void grab_rotate(uint8_t* data, uint8_t data_length)
{
  ulTaskNotifyTake(pdTRUE, 0); 
  
  gripper_id = data[0];
  block_mask = data[1];
  cmd_isFinished = 0; 
  gripper_cmd = 5;

  struct terminal_response_s terminal_response = {0};
  terminal_response.cmd = BOARD_GRAB_ROTATE;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}

static void ToggleCursor(uint8_t* data, uint8_t data_length)
{
  // Clear old notify flags to prevent false execution
  ulTaskNotifyTake(pdTRUE, 0); 
  // Point this FreeRTOS task as a reciever
  xTerminalTaskHandle = xTaskGetCurrentTaskHandle();
  
  cursor_cmd = data[0];
  gripper_cmd = 0;

  WaitForGripperCompletion();

  // terminal response
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_TOGGLE_CURSOR;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
  
}

static void SideGrips(uint8_t* data, uint8_t data_length)
{
  // Clear old notify flags to prevent false execution
  ulTaskNotifyTake(pdTRUE, 0); 
  // Point this FreeRTOS task as a reciever
  xTerminalTaskHandle = xTaskGetCurrentTaskHandle();
  
  gripper_id = data[0];
  grips_cmd = data[1];
  gripper_cmd = 1;

  WaitForGripperCompletion();

  // terminal response
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_SIDE_GRIPS;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}

static void GripperLift(uint8_t* data, uint8_t data_length)
{
  // Clear old notify flags to prevent false execution
  ulTaskNotifyTake(pdTRUE, 0); 
  // Point this FreeRTOS task as a reciever
  xTerminalTaskHandle = xTaskGetCurrentTaskHandle();
  
  gripper_id = data[0];
  lift_cmd = data[1];
  gripper_cmd = 2;

  WaitForGripperCompletion();

  // terminal response
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_GRIPPER_LIFT;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}

static void TogglePumps(uint8_t* data, uint8_t data_length)
{
  // Clear old notify flags to prevent false execution
  ulTaskNotifyTake(pdTRUE, 0); 
  // Point this FreeRTOS task as a reciever
  xTerminalTaskHandle = xTaskGetCurrentTaskHandle();
  
  gripper_id = data[0];
  pump_mask = data[1];
  gripper_cmd = 3;

  WaitForGripperCompletion();

  // terminal response
  struct terminal_response_s terminal_response;
  terminal_response.cmd = BOARD_TOGGLE_PUMPS;
  terminal_response.data[0] = 0x01;
  terminal_response.data_length = 1;
  Terminal.SendResponse(&terminal_response);
}

static void ToggleValves(uint8_t* data, uint8_t data_length)
{
  // Clear old notify flags to prevent false execution
  ulTaskNotifyTake(pdTRUE, 0); 
  // Point this FreeRTOS task as a reciever
  xTerminalTaskHandle = xTaskGetCurrentTaskHandle();
  
  gripper_id = data[0];
  valve_mask = data[1];
  gripper_cmd = 4;

  WaitForGripperCompletion();

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



void (*terminal_cmd_handlers[TERMINAL_CMD_TABLE_SIZE])(uint8_t* data, uint8_t data_length) = {[BOARD_Ping] = Ping,
                                                                                              [BOARD_SetSpeed] = SetRobotSpeed,
                                                                                              [BOARD_GetSpeed] = GetRobotSpeed,
                                                                                              [BOARD_SetCoord] = SetRobotCoord,
                                                                                              [BOARD_GetCoord] = GetRobotCoord,
                                                                                              [BOARD_GetBatteryState] = GetBatteryState,
                                                                                              [BOARD_Set_BMS_CalibrationValue] = Set_BMS_CalibrationValue,
																																															[BOARD_GET_AX_ANGLE] = GetAxAngle,
                                                                                              [BOARD_SET_AX_ANGLE] = SetAxAngle,
																																															[BOARD_SET_STEPPER_ZERO] = SetStepperZero,
																																															[BOARD_SET_STEPPER_POSITION] = SetStepperPosition,
																																															[BOARD_GET_STEPPER_POSITION] = GetStepperPosition,
																																															[BOARD_TOGGLE_PUMP] = TogglePump,
																																															[BOARD_SET_MAGNET_ANGLE] = SetMagnetAngle,
																																															[BOARD_GET_KEY] = GetKey,
																																															[BOARD_GET_TOGGLE] = GetToggle,
                                                                                              [BOARD_GET_DEBUG_MESSAGE] = GetDebugMessage,
                                                                                              [BOARD_GET_LINEAR_SWITCHES_STATE] = GetSwitchesState,
																																															[BOARD_SET_MAXONS_STATE] = SetMaxonsState,
																																															[BOARD_SET_COMM_STATE] = SetCommState,
																																															[BOARD_RESET] = BoardReset,
																																															[BOARD_GET_RESET] = BoardGetReset,
																																															[BOARD_CLEAR_RESET] = BoardClearReset,
                                                                                              [BOARD_GRAB_ON] = grab_on,
                                                                                              [BOARD_GRAB_OFF] = grab_off,
                                                                                              [BOARD_GRAB_ROTATE] = grab_rotate,
                                                                                              [BOARD_TOGGLE_CURSOR] = ToggleCursor,
                                                                                              [BOARD_SIDE_GRIPS] = SideGrips,
                                                                                              [BOARD_GRIPPER_LIFT] = GripperLift,
                                                                                              [BOARD_TOGGLE_PUMPS] = TogglePumps,
                                                                                              [BOARD_TOGGLE_VALVES] = ToggleValves,
                                                                                              [BOARD_MOVE] = SimpleMove
                                                                                              
                                                                                             

};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
