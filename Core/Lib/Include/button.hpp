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
#ifndef __BUTTON_H
#define __BUTTON_H

#ifdef __cplusplus
 extern "C" {
#endif
#include "std_def.hpp"
#include "buttons_enum.hpp"


enum button_polarity_e{
  BUTTON_POLARITY_HI,
  BUTTON_POLARITY_LO
};

enum button_state_e{
  BUTTON_STATE_UNKNOWN,
  BUTTON_STATE_PRESSED,
  BUTTON_STATE_RELEASED
};

enum button_action_e{
  BUTTON_NO_ACTION,
  BUTTON_ACTION_CLICK,
  BUTTON_ACTION_HOLD
};

enum button_callback_type_s{
  INIT_COMPLETE_CALLBACK,
  STATE_CHANGED_CALLBACK,
  ACTION_CALLBACK
};

struct button_s{
  void (*AddButton)(enum buttons_e button_id, GPIO_TypeDef* button_gpio, uint32_t button_pin, enum button_polarity_e button_polarity, 
                    void (*ButtonCallback)(enum buttons_e button_id, enum button_callback_type_s callback_type, enum button_action_e button_action, enum button_state_e button_state));
  enum button_state_e (*GetState)(enum buttons_e button_id);
  void (*PollButtons)(void);
};

extern struct button_s Button;

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
