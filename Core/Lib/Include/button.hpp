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

using ButtonCallback_t = void (*)(buttons_e button_id, button_callback_type_s callback_type, button_action_e button_action, button_state_e button_state);

/*  Debounced buttons; PollButtons() must be called every 1 ms  */
class ButtonDriver{
public:
  void AddButton(buttons_e button_id, GPIO_TypeDef* button_gpio, uint32_t button_pin, button_polarity_e button_polarity, ButtonCallback_t ButtonCallback);
  button_state_e GetState(buttons_e button_id);
  void PollButtons();

private:
  struct ButtonParams{
    GPIO_TypeDef* gpio;
    uint32_t pin;
    button_polarity_e polarity;
    ButtonCallback_t ButtonCallback;
    button_state_e state;
    uint32_t is_init_state_changed;
    uint32_t shift_reg;
    uint32_t press_counter_ms;
  };

  void Notify(uint8_t button_id, button_callback_type_s callback_type, button_action_e button_action, button_state_e button_state);

  ButtonParams buttons[BUTTONS_NUMBER];
};

/*  The object lives in HwInstance (hw_instance.hpp); this is a shortcut to it  */
extern ButtonDriver& Button;

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
