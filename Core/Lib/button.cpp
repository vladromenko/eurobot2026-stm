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
#include "button.hpp"
#include "stm32f4xx_ll_gpio.h"

namespace {

constexpr uint32_t PRESSED_TIME_ms = 8;
constexpr uint32_t HOLD_TIME_ms = 500;

/*  Last PRESSED_TIME_ms samples of the debounce shift register  */
constexpr uint32_t PRESSED_MASK = 0xFFFFFFFF>>(32-PRESSED_TIME_ms);

}

button_state_e ButtonDriver::GetState(buttons_e button_id)
{
    return buttons[button_id].state;
}

void ButtonDriver::AddButton(buttons_e button_id, GPIO_TypeDef* button_gpio, uint32_t button_pin, button_polarity_e button_polarity, ButtonCallback_t ButtonCallback)
{
    buttons[button_id].gpio = button_gpio;
    buttons[button_id].pin = button_pin;
    buttons[button_id].polarity = button_polarity;
    buttons[button_id].ButtonCallback = ButtonCallback;

    buttons[button_id].state = BUTTON_STATE_UNKNOWN;
    buttons[button_id].is_init_state_changed = 0;
    buttons[button_id].shift_reg = 0x55;
}

void ButtonDriver::Notify(uint8_t button_id, button_callback_type_s callback_type, button_action_e button_action, button_state_e button_state)
{
    if(buttons[button_id].ButtonCallback != nullptr)
        buttons[button_id].ButtonCallback((buttons_e)button_id, callback_type, button_action, button_state);
}

void ButtonDriver::PollButtons()
{
    for(uint8_t ik=0; ik<BUTTONS_NUMBER; ik++)
    {
        ButtonParams& button = buttons[ik];

        uint32_t pin_state = LL_GPIO_IsInputPinSet(button.gpio, button.pin)^button.polarity;

        /*  BUTTON BOUNCING FILTER  */
        button.shift_reg = (button.shift_reg<<1)|pin_state;

        bool is_stable_released = (button.shift_reg&PRESSED_MASK) == 0;
        bool is_stable_pressed = (button.shift_reg&PRESSED_MASK) == PRESSED_MASK;

        /*  DETERMINE STATE    */
        switch (button.state)
        {
            case BUTTON_STATE_PRESSED:
                if(is_stable_released)
                {
                    button.state = BUTTON_STATE_RELEASED;
                    button.is_init_state_changed = 1;
                    Notify(ik, STATE_CHANGED_CALLBACK, BUTTON_NO_ACTION, BUTTON_STATE_RELEASED);
                }
                break;
            case BUTTON_STATE_RELEASED:
                if(is_stable_pressed)
                {
                    button.state = BUTTON_STATE_PRESSED;
                    button.is_init_state_changed = 1;
                    Notify(ik, STATE_CHANGED_CALLBACK, BUTTON_NO_ACTION, BUTTON_STATE_PRESSED);
                }
                break;
            default:
                if(is_stable_released)
                {
                    button.state = BUTTON_STATE_RELEASED;
                    Notify(ik, INIT_COMPLETE_CALLBACK, BUTTON_NO_ACTION, BUTTON_STATE_RELEASED);
                }
                else if(is_stable_pressed)
                {
                    button.state = BUTTON_STATE_PRESSED;
                    Notify(ik, INIT_COMPLETE_CALLBACK, BUTTON_NO_ACTION, BUTTON_STATE_PRESSED);
                }
                break;
        }

        /*  Prevent action detection on buttons init state  */
        if(button.is_init_state_changed==0)
            continue;

        /*  ACTION DETECTION    */
        switch(button.state)
        {
            case BUTTON_STATE_PRESSED:
                if(button.press_counter_ms==HOLD_TIME_ms)
                    Notify(ik, ACTION_CALLBACK, BUTTON_ACTION_HOLD, BUTTON_STATE_PRESSED);
                if(button.press_counter_ms<=HOLD_TIME_ms)
                    button.press_counter_ms++;
                break;

            case BUTTON_STATE_RELEASED:
                if((button.press_counter_ms)&&(button.press_counter_ms<=HOLD_TIME_ms))
                    Notify(ik, ACTION_CALLBACK, BUTTON_ACTION_CLICK, BUTTON_STATE_RELEASED);
                button.press_counter_ms = 0;
                break;

            default:
                button.press_counter_ms = 0;
                break;
        }
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
