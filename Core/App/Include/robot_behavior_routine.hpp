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
#ifndef __ROBOT_BEHAVIOR_ROUTINE_H
#define __ROBOT_BEHAVIOR_ROUTINE_H

#include "std_def.hpp"
#include "button.hpp"
#include "maxon.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

struct button_callback_queue_item_s{
  enum buttons_e button_id;
  enum button_callback_type_s callback_type;
  enum button_action_e button_action;
  enum button_state_e button_state;
};

extern QueueHandle_t button_callbacks_queue_handle;


enum starting_cord_state_e{
  STARTING_CORD_ATTACHED,
  STARTING_CORD_RELEASED
};

enum strategy_e{
  STRATEGY1,
  STRATEGY2,
  STRATEGY3,
  STRATEGIES_NUMBER
};

enum playground_side_e{
  LEFT_SIDE,
  RIGHT_SIDE
};

enum robot_mode_e{
  GAME_MODE,
  DEBUG_MODE
};

enum maxons_state_e{
  MAXONS_DISABLED,
  MAXONS_ENABLED
};

enum linear_switch_state_e{
  LINEAR_SWITCH_PRESSED,
  LINEAR_SWITCH_RELEASED
};

enum reset_state_e{
	RESETED,
	NOT_RESETED
};

enum valve_1_enable_e{
	VALVE_1_DISABLED = 0,
	VALVE_1_ENABLED = 1
};

enum valve_2_enable_e{
	VALVE_2_DISABLED = 0,
	VALVE_2_ENABLED = 1
};

enum valve_3_enable_e{
	VALVE_3_DISABLED = 0,
	VALVE_3_ENABLED = 1
};

enum valve_4_enable_e{
	VALVE_4_DISABLED = 0,
	VALVE_4_ENABLED = 1
};

struct robot_ctrl_s{
	enum starting_cord_state_e starting_cord;
	enum playground_side_e playground_side;
	enum strategy_e strategy;
  enum maxons_state_e maxons_state;
  enum robot_mode_e robot_mode;
  enum linear_switch_state_e linear_switch_1;
  enum linear_switch_state_e linear_switch_2;
	enum linear_switch_state_e linear_switch_3;
	enum linear_switch_state_e linear_switch_4;
	enum linear_switch_state_e linear_switch_5;
	enum linear_switch_state_e linear_switch_6;
	enum linear_switch_state_e linear_switch_7;
	enum linear_switch_state_e linear_switch_8;
	enum linear_switch_state_e linear_switch_9;
	enum reset_state_e reset_state;
	enum valve_1_enable_e valve_1_enable;
	enum valve_2_enable_e valve_2_enable;
	enum valve_2_enable_e valve_3_enable;
	enum valve_2_enable_e valve_4_enable;
};

extern struct robot_ctrl_s robot_ctrl;

/*  Robot level actions and reaction to the buttons; state is kept in robot_ctrl  */
class RobotBehavior{
public:
  constexpr explicit RobotBehavior(MaxonDriver& maxon) : maxon(maxon) {}

  void MaxonsEnable();
  void MaxonsDisable();
  void MaxonsStop();
  void StopAllActuators();

  /*  Called from ButtonsCallbackTask for every button event  */
  void ButtonsCallback(buttons_e button_id, button_callback_type_s callback_type, button_action_e button_action, button_state_e button_state);

private:
  MaxonDriver& maxon;
};

extern RobotBehavior Robot;

#if defined (ROBOT_GUIDO)

  

#endif

#if defined (ROBOT_BIG)

  

#endif



#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
