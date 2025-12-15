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
/*
#include "os_tasks.h"

#include "dynamixel_ax.h" 
#include "bus_device.h"
#include "spi_gpo.h" 
#include "spi_gpo_defines.h"
#include "robot_behavior_routine.h"
#include "h_bridge.h"
#include "stm32f4xx_ll_gpio.h"
*/
#include "os_tasks.h" 
#include "dynamixel_master.h"
#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.h"
#include "robot_gpio_defines.h"
#include "byte2word.h" 
#include "maxon.h" 
#include "dynamixel_ax.h" 
#include "stepper.h" 
#include "rc_servo.h" 
#include "stm32f4xx_ll_tim.h"
#include "terminal.h"
#include "h_bridge.h"
#include "serial.h"

#include "spi_gpo.h" 
#include "spi_gpo_defines.h"
#include "kinematics.h"
#include "odometry.h"
#include "ws2812b.h" 
#include "robot_behavior_routine.h"

#include "bus_device.h"


uint8_t gripper_state;

#define TASK_STACK_SIZE             512


static TaskHandle_t task_handle;
static StackType_t stack_buffer[TASK_STACK_SIZE];
static StaticTask_t task_buffer;

//#define MAXON
//#define VALVES_4
//#define SERVO
//#define GRIPPER_V2
//#define DYNAMIXEL
//#define DEMO_GRIPPER
//#define DEMO_MOVE
#define DEMO_VALVE


/**
 * @brief Rotate two Dynamixels by a given angle synchronously.
 * @param usart USART bus of the Dynamixels
 * @param id_1 Dynamixel ID 1
 * @param angle Dynamixel ID 2
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

static void Task(void *arg)
{
  	vTaskDelay(50);

	uint8_t dynamixel_id_1 = 98;
	uint8_t dynamixel_id_2 = 112;
	uint16_t dynamixel_speed = 150; //0-1024
	uint8_t start_angle = 62; //0-300 60
	uint8_t end_angle = 135; //0-300 150 125

	uint8_t servo5_start_angle = 90; // left shift servo
	uint8_t servo5_end_angle = 170;
	uint8_t servo6_start_angle = 110; // right shift servo
	uint8_t servo6_end_angle = 20;
	/*
	ValveDisable(1);
	ValveDisable(2);
	ValveDisable(3);
	ValveDisable(4);
	*/

	//uint8_t dynamixel_id = 0xFE;
	
	// Set custom id to single dynamixel (DISCONNECT ANY OTHER DYNAMIXELS!)
	/*
	while(dynamixel_id > 0)
	{
		dynamixel_id --;
		if(Dynamixel_AX.Ping(USART3, dynamixel_id) == 0)
		{
			Dynamixel_AX.Set_ID(USART3, dynamixel_id, 4);
			break;
		}
	}
	Dynamixel_AX.TorqueEnable(USART3, 4);
	Dynamixel_AX.SetAngle(USART3, 4, 0);
	*/
	

	/*
	uint8_t maxon_id = 0xFE;
	
	// Set custom id to single maxon (DISCONNECT ANY OTHER maxons!)
	
	while(maxon_id > 0)
	{
		maxon_id --;
		if(Maxon.Ping(USART6, maxon_id) == 0)
		{
			Maxon.Set_ID(USART6, maxon_id, 0x22);
			break;
		}
	}
	*/

	//Maxon.Set_ID(USART6, 0x21, 0x22);


	#ifdef VALVES_4
		Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_1, dynamixel_speed);
		Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_2, dynamixel_speed);
		Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, 70);
		Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, 70);
		vTaskDelay(100);
		//Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, -10);
		//Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, -10);
	#endif


	#ifdef SERVO
		RC_Servo.Enable(RC_SERVO_1);
		RC_Servo.Enable(RC_SERVO_2);
		RC_Servo.Enable(RC_SERVO_3);
		RC_Servo.Enable(RC_SERVO_4);
		RC_Servo.Enable(RC_SERVO_5);
		RC_Servo.Enable(RC_SERVO_6);
		RC_Servo.Enable(RC_SERVO_7);
	#endif
	
	#ifdef MAXON
		MaxonsDisable();
		//Maxon.Enable(bus_devices[MAXON_MOTOR_3].usart, bus_devices[MAXON_MOTOR_3].id);
    	Kinematics.MatrixInit();
    	MaxonsEnable();
		vTaskDelay(500);
	#endif

	#ifdef GRIPPER_V2
		// dynamixels init
		Dynamixel_AX.TorqueEnable(USART3, dynamixel_id_1);
		Dynamixel_AX.TorqueEnable(USART3, dynamixel_id_2);

		Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_1, dynamixel_speed);
		Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_2, dynamixel_speed);
		RotateGripper(USART3, dynamixel_id_1, dynamixel_id_2, 70);
		vTaskDelay(100);

		// maxons init
		MaxonsDisable();
    	Kinematics.MatrixInit();
    	MaxonsEnable();
		vTaskDelay(500);

		// servos init
		RC_Servo.Enable(RC_SERVO_1);
		RC_Servo.Enable(RC_SERVO_2);
		RC_Servo.Enable(RC_SERVO_3);
		RC_Servo.Enable(RC_SERVO_4);
		RC_Servo.Enable(RC_SERVO_5);
		RC_Servo.Enable(RC_SERVO_6);

		RC_Servo.SetServoAngle(RC_SERVO_1, 450, 2500, 200, 0);
		RC_Servo.SetServoAngle(RC_SERVO_2, 450, 2500, 200, 0);
		RC_Servo.SetServoAngle(RC_SERVO_3, 500, 2500, 180, 0);
		RC_Servo.SetServoAngle(RC_SERVO_4, 500, 2500, 180, 0);
		RC_Servo.SetServoAngle(RC_SERVO_5, 500, 2500, 180, servo5_start_angle); //shift servos
		RC_Servo.SetServoAngle(RC_SERVO_6, 500, 2500, 180, servo6_start_angle);
		vTaskDelay(500);
	#endif

	#ifdef DEMO_MOVE
		MaxonsDisable();
    	Kinematics.MatrixInit();
    	MaxonsEnable();
		vTaskDelay(500);
	#endif

	#ifdef DEMO_GRIPPER

	#endif

		#ifdef DEMO_VALVE
		// dynamixels init
		Dynamixel_AX.TorqueEnable(USART3, dynamixel_id_1);
		Dynamixel_AX.TorqueEnable(USART3, dynamixel_id_2);

		Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_1, dynamixel_speed);
		Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_id_2, dynamixel_speed);
		RotateGripper(USART3, dynamixel_id_1, dynamixel_id_2, 70);
		vTaskDelay(100);

		// maxons init
		MaxonsDisable();
    	Kinematics.MatrixInit();
    	MaxonsEnable();
		vTaskDelay(500);

		// servos init
		RC_Servo.Enable(RC_SERVO_1);
		RC_Servo.Enable(RC_SERVO_2);
		RC_Servo.Enable(RC_SERVO_3);
		RC_Servo.Enable(RC_SERVO_4);
		RC_Servo.Enable(RC_SERVO_5);
		RC_Servo.Enable(RC_SERVO_6);

		RC_Servo.SetServoAngle(RC_SERVO_1, 450, 2500, 200, 0);
		RC_Servo.SetServoAngle(RC_SERVO_2, 450, 2500, 200, 0);
		RC_Servo.SetServoAngle(RC_SERVO_3, 500, 2500, 180, 0);
		RC_Servo.SetServoAngle(RC_SERVO_4, 500, 2500, 180, 0);
		RC_Servo.SetServoAngle(RC_SERVO_5, 500, 2500, 180, servo5_start_angle); //shift servos
		RC_Servo.SetServoAngle(RC_SERVO_6, 500, 2500, 180, servo6_start_angle);
		vTaskDelay(500);
	#endif

	while(1)
	{
		vTaskDelay(100);
		
		#ifdef VALVES_4
		if(robot_ctrl.linear_switch_1 == LINEAR_SWITCH_PRESSED)
		{
			SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_5);
			SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_6);
			
			// 1st step
			ValveEnable(1);
			ValveEnable(2);
			ValveEnable(3);
			ValveEnable(4);
			
			if(robot_ctrl.linear_switch_3 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, start_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, start_angle);
				//vTaskDelay(1000);
			}
			vTaskDelay(1000);
			
			if(robot_ctrl.linear_switch_2 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, end_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, end_angle);
				//ValveDisable(1);
			}
			vTaskDelay(1000);
			
			// 2nd step
			//ValveEnable(1);
			//ValveEnable(2);
			//ValveEnable(3);
			//ValveEnable(4);
			
			ValveEnable(1);
			ValveEnable(2);
			ValveEnable(3);
			ValveDisable(4);
				
			if(robot_ctrl.linear_switch_3 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, start_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, start_angle);
				//vTaskDelay(1000);
			}
			vTaskDelay(1000);
			
			if(robot_ctrl.linear_switch_2 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, end_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, end_angle);
				//ValveDisable(2);
			}
			vTaskDelay(1000);
			
			// 3rd step
			//ValveEnable(1);
			//ValveEnable(2);
			//ValveEnable(3);
			//ValveEnable(4);
			
			ValveEnable(1);
			ValveEnable(2);
			ValveDisable(3);
			ValveDisable(4);
			
				
			if(robot_ctrl.linear_switch_3 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, start_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, start_angle);
				//vTaskDelay(1000);
			}
			vTaskDelay(1000);
			
			if(robot_ctrl.linear_switch_2 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, end_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, end_angle);
				ValveDisable(3);
			}
			vTaskDelay(1000);
			
			// 4th step
			//ValveEnable(1);
			//ValveEnable(2);
			//ValveEnable(3);
			//ValveEnable(4);
			
			ValveEnable(1);
			ValveDisable(2);
			ValveDisable(3);
			ValveDisable(4);
				
			if(robot_ctrl.linear_switch_3 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, start_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, start_angle);
				//vTaskDelay(1000);
				
			}
			vTaskDelay(1000);
			
			if(robot_ctrl.linear_switch_2 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, end_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, end_angle);
				ValveDisable(4);
			}
			vTaskDelay(1000);
			
			// final step (blocks dropping)
			ValveDisable(1);
			ValveDisable(2);
			ValveDisable(3);
			ValveDisable(4);
			
			if(robot_ctrl.linear_switch_3 == LINEAR_SWITCH_RELEASED)
			{
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_1, start_angle);
				Dynamixel_AX.SetAngle(USART3, dynamixel_id_2, start_angle);
				//vTaskDelay(1000);
			}
			vTaskDelay(1000);
			SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_5);
			SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_6);
			
		
		}
		#endif
		
		#ifdef SERVO
			/*
			RC_Servo.Enable(RC_SERVO_7);
			RC_Servo.SetServoAngle(RC_SERVO_7, 450, 2600, 200, 0);
			vTaskDelay(2000);
			RC_Servo.SetServoAngle(RC_SERVO_7, 450, 2600, 200, 200);
			vTaskDelay(200);
			RC_Servo.Disable(RC_SERVO_7);
			vTaskDelay(2000);
			*/
	
			RC_Servo.SetServoAngle(RC_SERVO_7, 450, 2500, 200, 0);
			vTaskDelay(2000);
			RC_Servo.SetServoAngle(RC_SERVO_7, 450, 2500, 200, 200);
			vTaskDelay(2000);
			

			/*
			// this PWM range is working (wide enough)
			RC_Servo.SetServoAngle(RC_SERVO_1, 500, 2500, 180, 0);
			vTaskDelay(1000);
			RC_Servo.SetServoAngle(RC_SERVO_1, 500, 2500, 180, 180);
			vTaskDelay(1000);

			RC_Servo.SetServoAngle(RC_SERVO_5, 500, 2500, 180, servo5_start_angle);
			vTaskDelay(1000);
			RC_Servo.SetServoAngle(RC_SERVO_5, 500, 2500, 180, servo5_end_angle);
			vTaskDelay(1000);

			RC_Servo.SetServoAngle(RC_SERVO_6, 500, 2500, 180, servo6_start_angle);
			vTaskDelay(1000);
			RC_Servo.SetServoAngle(RC_SERVO_6, 500, 2500, 180, servo6_end_angle);
			vTaskDelay(1000);
			*/

		#endif


		#ifdef MAXON
			/*
			Maxon.Set_TPM(bus_devices[MAXON_MOTOR_3].usart, bus_devices[MAXON_MOTOR_3].id, 10000000);
			vTaskDelay(2000);
			

			Maxon.Set_TPM(bus_devices[MAXON_MOTOR_1].usart, bus_devices[MAXON_MOTOR_1].id, -10000000);
			vTaskDelay(2000);
			*/
			struct speed_s speed;

			speed.dphi = 1.57f;
			speed.dx = 0;
			speed.dy = 0;
			Odometry.SetSpeed(&speed);
			vTaskDelay(1000);
			

			/*
			speed.dphi = -1.57f;
			Odometry.SetSpeed(&speed);
			vTaskDelay(1000);

			speed.dphi = 0;
			speed.dx = 0.1;
			speed.dy = 0;
			Odometry.SetSpeed(&speed);
			vTaskDelay(1000);
			
			speed.dphi = 0;
			speed.dx = -0.1;
			speed.dy = 0;
			Odometry.SetSpeed(&speed);
			vTaskDelay(1000);

			speed.dphi = 0;
			speed.dx = 0;
			speed.dy = 0.1;
			Odometry.SetSpeed(&speed);
			vTaskDelay(1000);
			
			speed.dphi = 0;
			speed.dx = 0;
			speed.dy = -0.1;
			Odometry.SetSpeed(&speed);
			vTaskDelay(1000);
			*/
		#endif

		#ifdef GRIPPER_V2
			if(robot_ctrl.linear_switch_1 == LINEAR_SWITCH_PRESSED)
			{
				struct speed_s speed; // local variable [Vx, Vy, omega] to prevent global rewriting and data race
				/*
				speed.dphi = 1.57f;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1000);
				*/
				speed.dphi = 0;
				speed.dx = -0.1f;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(500);

				speed.dphi = 0;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(100);

				//SPI_GPO_SetOutputPin(SPI_GPO_ALL);
				SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_3);
				
				SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_1 | SPI_POWER_OUTPUT_2); // turn on pumps
				vTaskDelay(1000);
				//SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_3); // turn on valve

				RC_Servo.SetServoAngle(RC_SERVO_1, 450, 2550, 200, 0);
				RC_Servo.SetServoAngle(RC_SERVO_2, 450, 2550, 200, 0);
				RC_Servo.SetServoAngle(RC_SERVO_3, 450, 2550, 200, 0);
				RC_Servo.SetServoAngle(RC_SERVO_4, 450, 2550, 200, 0);
				vTaskDelay(500);
				RC_Servo.SetServoAngle(RC_SERVO_5, 500, 2500, 180, servo5_start_angle);
				RC_Servo.SetServoAngle(RC_SERVO_6, 500, 2500, 180, servo6_start_angle);
				vTaskDelay(500);

				//if(robot_ctrl.linear_switch_3 == LINEAR_SWITCH_RELEASED)
				//{
					RotateGripper(USART3, dynamixel_id_1, dynamixel_id_2, start_angle);
					//vTaskDelay(1000);
				//}
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
				vTaskDelay(1000);


				//if(robot_ctrl.linear_switch_2 == LINEAR_SWITCH_RELEASED)
				//{
					RotateGripper(USART3, dynamixel_id_1, dynamixel_id_2, end_angle);
				//}
				vTaskDelay(1000);

				// rotation of the grabbed blocks
				RC_Servo.SetServoAngle(RC_SERVO_5, 500, 2500, 180, servo5_end_angle); // move the block #1 apart
				vTaskDelay(500);
				RC_Servo.SetServoAngle(RC_SERVO_6, 500, 2500, 180, servo6_end_angle); // move the block #3 apart
				vTaskDelay(500);
				RC_Servo.SetServoAngle(RC_SERVO_1, 450, 2550, 200, 200); // flip the block #1
				//RC_Servo.SetServoAngle(RC_SERVO_2, 450, 2550, 200, 200); // flip the block #2
				//RC_Servo.SetServoAngle(RC_SERVO_3, 450, 2550, 200, 200);
				//RC_Servo.SetServoAngle(RC_SERVO_4, 450, 2550, 200, 200);
				vTaskDelay(500);
				RC_Servo.SetServoAngle(RC_SERVO_5, 500, 2500, 180, servo5_start_angle); // move the block #1 apart
				vTaskDelay(500);
				RC_Servo.SetServoAngle(RC_SERVO_6, 500, 2500, 180, servo6_start_angle); // move the block #3 apart
				vTaskDelay(1000);

				//if(robot_ctrl.linear_switch_3 == LINEAR_SWITCH_RELEASED)
				//{
					RotateGripper(USART3, dynamixel_id_1, dynamixel_id_2, start_angle);
					//vTaskDelay(1000);
				//}
				vTaskDelay(1000);

				SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_3); // turn off valve
				SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_1 | SPI_POWER_OUTPUT_2); // turn off pumps
				vTaskDelay(1000);

				//if(robot_ctrl.linear_switch_2 == LINEAR_SWITCH_RELEASED)
				//{
					RotateGripper(USART3, dynamixel_id_1, dynamixel_id_2, end_angle);
				//}
				vTaskDelay(1000);

				speed.dphi = 0;
				speed.dx = 0.05f;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(500);

				speed.dphi = 0;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(100);

				vTaskDelay(2000);
			}
		#endif

		#ifdef DEMO_MOVE
			if(robot_ctrl.linear_switch_2 == LINEAR_SWITCH_PRESSED)
			{
				struct speed_s speed; // local variable [Vx, Vy, omega] to prevent global rewriting and data race

				speed.dphi = 0;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(3000);

				// rectangular without rotation

				speed.dphi = 0;
				speed.dx = -0.2f;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				speed.dphi = 0;
				speed.dx = 0;
				speed.dy = -0.2f;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				speed.dphi = 0;
				speed.dx = 0.2f;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				speed.dphi = 0;
				speed.dx = 0;
				speed.dy = 0.2f;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				speed.dphi = 0;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(4000);


				// rectangular with rotation

				speed.dphi = 0;
				speed.dx = -0.2f;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				speed.dphi = -1.6f;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1000);

				speed.dphi = 0;
				speed.dx = -0.2f;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				speed.dphi = -1.6f;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1000);

				speed.dphi = 0;
				speed.dx = -0.2f;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				speed.dphi = -1.6f;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1000);

				speed.dphi = 0;
				speed.dx = -0.2f;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				speed.dphi = -1.8f;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1000);

				speed.dphi = 0;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(2000);


				// move across X
				speed.dphi = 0;
				speed.dx = -0.2f;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				speed.dphi = 0;
				speed.dx = 0.2f;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				
				// move across Y
				speed.dphi = 0;
				speed.dx = 0;
				speed.dy = -0.2f;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				speed.dphi = 0;
				speed.dx = 0;
				speed.dy = 0.2f;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				// move across XY+
				speed.dphi = 0;
				speed.dx = -0.2f;
				speed.dy = -0.2f;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				speed.dphi = 0;
				speed.dx = 0.2f;
				speed.dy = 0.2f;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				// move across XY-
				speed.dphi = 0;
				speed.dx = 0.2f;
				speed.dy = -0.2f;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				speed.dphi = 0;
				speed.dx = -0.2f;
				speed.dy = 0.2f;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				// rotation CW
				speed.dphi = -1.57f;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(2000);

				// rotation CCW
				speed.dphi = 1.57f;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(2000);

				// stop
				speed.dphi = 0;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(2000);

				// circle trajectory
			}
		#endif

		#ifdef DEMO_VALVE
			if(robot_ctrl.linear_switch_1 == LINEAR_SWITCH_PRESSED)
			{
				struct speed_s speed; // local variable [Vx, Vy, omega] to prevent global rewriting and data race
				/*
				speed.dphi = 1.57f;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1000);
				*/
				speed.dphi = 0;
				speed.dx = -0.1f;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(500);

				speed.dphi = 0;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(100);

				SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_3 | SPI_POWER_OUTPUT_4); // turn on valves
				SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_1 | SPI_POWER_OUTPUT_2); // turn on pumps
				vTaskDelay(1000);

				RC_Servo.SetServoAngle(RC_SERVO_1, 450, 2550, 200, 0);
				RC_Servo.SetServoAngle(RC_SERVO_2, 450, 2550, 200, 0);
				RC_Servo.SetServoAngle(RC_SERVO_3, 450, 2550, 200, 0);
				RC_Servo.SetServoAngle(RC_SERVO_4, 450, 2550, 200, 0);
				vTaskDelay(500);
				RC_Servo.SetServoAngle(RC_SERVO_5, 500, 2500, 180, servo5_start_angle);
				RC_Servo.SetServoAngle(RC_SERVO_6, 500, 2500, 180, servo6_start_angle);
				vTaskDelay(500);

				RotateGripper(USART3, dynamixel_id_1, dynamixel_id_2, start_angle);
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
				vTaskDelay(1000);

				RotateGripper(USART3, dynamixel_id_1, dynamixel_id_2, end_angle);
				vTaskDelay(1000);

				// rotation of the grabbed blocks
				RC_Servo.SetServoAngle(RC_SERVO_5, 500, 2500, 180, servo5_end_angle); // move the block #1 apart
				vTaskDelay(500);
				RC_Servo.SetServoAngle(RC_SERVO_6, 500, 2500, 180, servo6_end_angle); // move the block #3 apart
				vTaskDelay(500);
				RC_Servo.SetServoAngle(RC_SERVO_1, 450, 2550, 200, 200); // flip the block #1
				//RC_Servo.SetServoAngle(RC_SERVO_2, 450, 2550, 200, 200); // flip the block #2
				//RC_Servo.SetServoAngle(RC_SERVO_3, 450, 2550, 200, 200);
				//RC_Servo.SetServoAngle(RC_SERVO_4, 450, 2550, 200, 200);
				vTaskDelay(500);
				RC_Servo.SetServoAngle(RC_SERVO_5, 500, 2500, 180, servo5_start_angle); // move the block #1 apart
				vTaskDelay(500);
				RC_Servo.SetServoAngle(RC_SERVO_6, 500, 2500, 180, servo6_start_angle); // move the block #3 apart
				vTaskDelay(1000);

				
				// moving part
				speed.dphi = 0;
				speed.dx = 0.05f;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(500);

				speed.dphi = 0;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(100);

				speed.dphi = -1.5f;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(2000);

				speed.dphi = 0;
				speed.dx = -1.0f;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(2400);

				speed.dphi = -1.5f;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1000);

				speed.dphi = 0;
				speed.dx = -1.0f;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1000);

				speed.dphi = -1.6f;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1500);

				speed.dphi = 0;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(1000);


				RotateGripper(USART3, dynamixel_id_1, dynamixel_id_2, start_angle);
				vTaskDelay(1000);

				SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_3 | SPI_POWER_OUTPUT_4); // turn off valves
				SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_1 | SPI_POWER_OUTPUT_2); // turn off pumps
				vTaskDelay(1000);

				RotateGripper(USART3, dynamixel_id_1, dynamixel_id_2, end_angle);
				vTaskDelay(1000);

				speed.dphi = 0;
				speed.dx = 0.05f;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(500);

				speed.dphi = 0;
				speed.dx = 0;
				speed.dy = 0;
				Odometry.SetSpeed(&speed);
				vTaskDelay(100);

				vTaskDelay(2000);
			}
		#endif

	}
	
}

static TaskHandle_t GetTaskHandle(void)
{
    return task_handle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    task_handle = xTaskCreateStatic(Task, "GripperTask", TASK_STACK_SIZE,
                              NULL, uxPriority, stack_buffer, &task_buffer);
}


const struct os_task_base_s GripperTask = { .CreateTask = CreateTask,
                                          .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
