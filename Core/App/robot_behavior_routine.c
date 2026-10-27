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
#include "robot_behavior_routine.h" 
#include "robot_gpio_defines.h"
#include "bus_device.h"
#include "maxon.h"
#include "dynamixel_ax.h"

struct robot_ctrl_s robot_ctrl;
QueueHandle_t button_callbacks_queue_handle = NULL;


#if defined (ROBOT_GUIDO)

  	

#endif

#if defined ROBOT_BIG

  	

#endif

void PrepareRobot(void)
{
	Dynamixel_AX.TorqueEnable(bus_devices[GRIPPER_TOP].usart, bus_devices[GRIPPER_TOP].id);
	Dynamixel_AX.TorqueEnable(bus_devices[GRIPPER_BOTTOM].usart, bus_devices[GRIPPER_BOTTOM].id);
	// Dynamixel_AX.TorqueEnable(bus_devices[GRIPPER_BOTTOM].usart, 1);
	// Dynamixel_AX.Enable_LED(USART3, 1);
	// Dynamixel_AX.SetAngle(USART3,1,300);
}

void MaxonsEnable(void)
{
	Maxon.Enable(bus_devices[MAXON_MOTOR_1].usart, bus_devices[MAXON_MOTOR_1].id);
  	Maxon.Enable(bus_devices[MAXON_MOTOR_2].usart, bus_devices[MAXON_MOTOR_2].id);
  	Maxon.Enable(bus_devices[MAXON_MOTOR_3].usart, bus_devices[MAXON_MOTOR_3].id);
}

void MaxonsDisable(void)
{
	Maxon.Disable(bus_devices[MAXON_MOTOR_1].usart, bus_devices[MAXON_MOTOR_1].id);
  	Maxon.Disable(bus_devices[MAXON_MOTOR_2].usart, bus_devices[MAXON_MOTOR_2].id);
  	Maxon.Disable(bus_devices[MAXON_MOTOR_3].usart, bus_devices[MAXON_MOTOR_3].id);
}

void MaxonsStop(void)
{
	Maxon.Set_TPM(bus_devices[MAXON_MOTOR_1].usart, bus_devices[MAXON_MOTOR_1].id, 0);
  	Maxon.Set_TPM(bus_devices[MAXON_MOTOR_2].usart, bus_devices[MAXON_MOTOR_2].id, 0);
  	Maxon.Set_TPM(bus_devices[MAXON_MOTOR_3].usart, bus_devices[MAXON_MOTOR_3].id, 0);
}


void StopAllActuators(void)
{
	MaxonsDisable();
}




void ButtonsCallback(enum buttons_e button_id, enum button_callback_type_s callback_type, enum button_action_e button_action, enum button_state_e button_state)
{
	switch (button_id)
	{
		case STARTING_CORD_BTN:
			if((callback_type == STATE_CHANGED_CALLBACK) || (callback_type == INIT_COMPLETE_CALLBACK))
			{
				if(button_state == BUTTON_STATE_PRESSED)
					robot_ctrl.starting_cord = STARTING_CORD_ATTACHED;
				if(button_state == BUTTON_STATE_RELEASED)
					robot_ctrl.starting_cord = STARTING_CORD_RELEASED;
			}

		case PLAYGROUND_SIDE_BTN:
			if((callback_type == STATE_CHANGED_CALLBACK) || (callback_type == INIT_COMPLETE_CALLBACK))
			{
				if(button_state == BUTTON_STATE_PRESSED)
					robot_ctrl.playground_side = RIGHT_SIDE;
				if(button_state == BUTTON_STATE_RELEASED)
					robot_ctrl.playground_side = LEFT_SIDE;
			}

		case STRATEGY_BTN:
			if(callback_type == ACTION_CALLBACK)
			{
				if(button_action == BUTTON_ACTION_CLICK)
					robot_ctrl.strategy = (robot_ctrl.strategy < STRATEGIES_NUMBER) ? robot_ctrl.strategy++ : (enum strategy_e)0;
			}

		case MAXON_ENABLE_BTN:
			if((callback_type == STATE_CHANGED_CALLBACK) || (callback_type == INIT_COMPLETE_CALLBACK))
			{
				if(button_state == BUTTON_STATE_PRESSED){
					robot_ctrl.maxons_state = MAXONS_ENABLED;
					MaxonsEnable();
				}
				if(button_state == BUTTON_STATE_RELEASED){
					robot_ctrl.maxons_state = MAXONS_DISABLED;
					MaxonsDisable();
				}
			}
			
		default:
			break;
	}
}




// void ControlButtons_Callback(enum buttons_e button_id, enum button_event_e button_event)
// {
// 	switch (button_id)
// 	{
// 		case STARTING_CORD_BTN:
// 			if(button_event == BUTTON_EVENT_PRESSED)
// 				robot_ctrl.starting_cord = STARTING_CORD_ATTACHED;
// 			if(button_event == BUTTON_EVENT_RELEASED)
// 				robot_ctrl.starting_cord = STARTING_CORD_RELEASED;
// 			break;

// 		case PLAYGROUND_SIDE_BTN:
// 			if(button_event == BUTTON_EVENT_PRESSED)
// 				robot_ctrl.playground_side = RIGHT_SIDE;
// 			if(button_event == BUTTON_EVENT_RELEASED)
// 				robot_ctrl.playground_side = LEFT_SIDE;
// 			break;

// 		case STRATEGY_BTN:
// 			if(button_event == BUTTON_EVENT_CLICK)
// 			{
// 				if(robot_ctrl.strategy<STRATEGIES_NUMBER)
// 					robot_ctrl.strategy++;
// 				else
// 					robot_ctrl.strategy = (enum strategy_e)0;
// 			}
// 			break;

// 		case MAXON_ENABLE_BTN:
// 			if(button_event == BUTTON_EVENT_PRESSED){
// 				robot_ctrl.maxons_state = MAXONS_ENABLED;
// 				MaxonsEnable();
// 			}
// 			if(button_event == BUTTON_EVENT_RELEASED){
// 				robot_ctrl.maxons_state = MAXONS_DISABLED;
// 				MaxonsDisable();
// 			}
// 			break;

// 		default:
// 			break;
// 	}
// 	// LL_GPIO_TogglePin(GPIOA, PWM8_PA3);
// }


// void Starting_Cord_BTN_Callback(enum buttons_e button_id, enum button_event_e button_event)
// {

// }

// void Playground_Side_BTN_Callback(enum buttons_e button_id, enum button_event_e button_event)
// {

// }

// void Strategy_BTN_Callback(enum buttons_e button_id, enum button_event_e button_event)
// {
	
// }

// void Maxon_Enable_BTN_Callback(enum buttons_e button_id, enum button_event_e button_event)
// {
// 	if(button_event == BUTTON_EVENT_PRESSED)
// 		MaxonsEnable();
// 	if(button_event == BUTTON_EVENT_RELEASED)
// 		MaxonsDisable();
// }

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
