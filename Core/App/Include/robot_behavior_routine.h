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

#ifdef __cplusplus
 extern "C" {
#endif
#include "std_def.h"
#include "button.h"

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

struct robot_ctrl_s{
	enum starting_cord_state_e starting_cord;
	enum playground_side_e playground_side;
	enum strategy_e strategy;
  enum maxons_state_e maxons_state;
  enum robot_mode_e robot_mode;
};

extern struct robot_ctrl_s robot_ctrl;

void ButtonsCallback(enum buttons_e button_id, enum button_callback_type_s callback_type, enum button_action_e button_action, enum button_state_e button_state);
// void ControlButtons_Callback(enum buttons_e button_id, enum button_event_e button_event);
void MaxonsEnable(void);
void MaxonsDisable(void);
void MaxonsStop(void);
void StopAllActuators(void);
void PrepareRobot(void);

// void Starting_Cord_BTN_Callback(enum buttons_e button_id, enum button_event_e button_event);
// void Playground_Side_BTN_Callback(enum buttons_e button_id, enum button_event_e button_event);
// void Strategy_BTN_Callback(enum buttons_e button_id, enum button_event_e button_event);
// void Maxon_Enable_BTN_Callback(enum buttons_e button_id, enum button_event_e button_event);

#if defined (ROBOT_GUIDO)

  

#endif

#if defined (ROBOT_BIG)

  

#endif


#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
