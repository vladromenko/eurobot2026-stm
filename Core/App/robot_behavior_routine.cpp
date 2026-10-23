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
#include "robot_behavior_routine.hpp" 
#include "hw_instance.hpp"
#include "robot_gpio_defines.hpp"
#include "bus_device.hpp"
#include "spi_gpo.hpp" 
#include "spi_gpo_defines.hpp"
#include "stepper.hpp"

struct robot_ctrl_s robot_ctrl;
QueueHandle_t button_callbacks_queue_handle = NULL;

constinit RobotBehavior Robot{GetHwInstance().maxon};


namespace {

inline bool IsStateEvent(button_callback_type_s callback_type)
{
	return (callback_type == STATE_CHANGED_CALLBACK) || (callback_type == INIT_COMPLETE_CALLBACK);
}

/*	Linear switches are wired inverted: a pressed button means a released switch	*/
void UpdateLinearSwitch(linear_switch_state_e& linear_switch, button_callback_type_s callback_type, button_state_e button_state)
{
	if(!IsStateEvent(callback_type))
		return;

	if(button_state == BUTTON_STATE_PRESSED)
		linear_switch = LINEAR_SWITCH_RELEASED;
	if(button_state == BUTTON_STATE_RELEASED)
		linear_switch = LINEAR_SWITCH_PRESSED;
}

}


void RobotBehavior::MaxonsEnable()
{
	maxon.Enable(bus_devices[MAXON_MOTOR_1].id);
	maxon.Enable(bus_devices[MAXON_MOTOR_2].id);
	maxon.Enable(bus_devices[MAXON_MOTOR_3].id);
	robot_ctrl.maxons_state = MAXONS_ENABLED;
}

void RobotBehavior::MaxonsDisable()
{
	maxon.Disable(bus_devices[MAXON_MOTOR_1].id);
	maxon.Disable(bus_devices[MAXON_MOTOR_2].id);
	maxon.Disable(bus_devices[MAXON_MOTOR_3].id);
	robot_ctrl.maxons_state = MAXONS_DISABLED;
}

void RobotBehavior::MaxonsStop()
{
	maxon.Set_TPM(bus_devices[MAXON_MOTOR_1].id, 0);
	maxon.Set_TPM(bus_devices[MAXON_MOTOR_2].id, 0);
	maxon.Set_TPM(bus_devices[MAXON_MOTOR_3].id, 0);
}

void RobotBehavior::StopAllActuators()
{
	MaxonsDisable();
	SPI_GPO.ResetOutputPin(SPI_GPO_ALL);
	Stepper.StopMotor(STEPPER_MOTOR_1);
}

void RobotBehavior::ButtonsCallback(buttons_e button_id, button_callback_type_s callback_type, button_action_e button_action, button_state_e button_state)
{
	switch (button_id)
	{
		case STARTING_CORD_BTN:
			if(IsStateEvent(callback_type))
			{
				if(button_state == BUTTON_STATE_PRESSED)
					robot_ctrl.starting_cord = STARTING_CORD_ATTACHED;
				if(button_state == BUTTON_STATE_RELEASED)
					robot_ctrl.starting_cord = STARTING_CORD_RELEASED;
			}
			break;

		case PLAYGROUND_SIDE_BTN:
			if(IsStateEvent(callback_type))
			{
				if(button_state == BUTTON_STATE_PRESSED)
					robot_ctrl.playground_side = RIGHT_SIDE;
				if(button_state == BUTTON_STATE_RELEASED)
					robot_ctrl.playground_side = LEFT_SIDE;
			}
			break;

		case STRATEGY_BTN:
			if(callback_type == ACTION_CALLBACK)
			{
				if(button_action == BUTTON_ACTION_CLICK)
					// C++ has no ++ for enums; cycle through strategies. The C original modified
					// strategy twice in one expression (undefined behavior) and in practice never changed it.
					robot_ctrl.strategy = (robot_ctrl.strategy + 1 < STRATEGIES_NUMBER) ? (strategy_e)(robot_ctrl.strategy + 1) : (strategy_e)0;
			}
			break;

		case MAXON_ENABLE_BTN:
			if(IsStateEvent(callback_type))
			{
				if(button_state == BUTTON_STATE_PRESSED)
					robot_ctrl.maxons_state = MAXONS_ENABLED;
				if(button_state == BUTTON_STATE_RELEASED)
					robot_ctrl.maxons_state = MAXONS_DISABLED;
			}
			break;

		case LINEAR_SWITCH_1_BTN: UpdateLinearSwitch(robot_ctrl.linear_switch_1, callback_type, button_state); break;
		case LINEAR_SWITCH_2_BTN: UpdateLinearSwitch(robot_ctrl.linear_switch_2, callback_type, button_state); break;
		case LINEAR_SWITCH_3_BTN: UpdateLinearSwitch(robot_ctrl.linear_switch_3, callback_type, button_state); break;
		case LINEAR_SWITCH_4_BTN: UpdateLinearSwitch(robot_ctrl.linear_switch_4, callback_type, button_state); break;
		case LINEAR_SWITCH_5_BTN: UpdateLinearSwitch(robot_ctrl.linear_switch_5, callback_type, button_state); break;
		case LINEAR_SWITCH_6_BTN: UpdateLinearSwitch(robot_ctrl.linear_switch_6, callback_type, button_state); break;
		case LINEAR_SWITCH_7_BTN: UpdateLinearSwitch(robot_ctrl.linear_switch_7, callback_type, button_state); break;
		case LINEAR_SWITCH_8_BTN: UpdateLinearSwitch(robot_ctrl.linear_switch_8, callback_type, button_state); break;
		case LINEAR_SWITCH_9_BTN: UpdateLinearSwitch(robot_ctrl.linear_switch_9, callback_type, button_state); break;

		default:
			break;
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
