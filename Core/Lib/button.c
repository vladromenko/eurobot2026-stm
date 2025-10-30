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
#include "button.h"
#include "stm32f4xx_ll_gpio.h"



#define PRESSED_TIME_ms     8
#define HOLD_TIME_ms        500


struct button_params_s{
    GPIO_TypeDef* gpio;
    uint32_t pin;
    enum button_polarity_e polarity;
    void (*ButtonCallback)(enum buttons_e button_id, enum button_callback_type_s callback_type, enum button_action_e button_action, enum button_state_e button_state);
    enum button_state_e state;
    uint32_t is_init_state_changed;
    uint32_t shift_reg;
    uint32_t press_counter_ms;
};

struct button_params_s buttons[BUTTONS_NUMBER];


static enum button_state_e GetState(enum buttons_e button_id)
{
    return buttons[button_id].state;
}

static void AddButton(enum buttons_e button_id, GPIO_TypeDef* button_gpio, uint32_t button_pin, enum button_polarity_e button_polarity, 
                        void (*ButtonCallback)(enum buttons_e button_id, enum button_callback_type_s callback_type, enum button_action_e button_action, enum button_state_e button_state))
{
    buttons[button_id].gpio = button_gpio;
    buttons[button_id].pin = button_pin;
    buttons[button_id].polarity = button_polarity;
    buttons[button_id].ButtonCallback = ButtonCallback;

    buttons[button_id].state = BUTTON_STATE_UNKNOWN;
    buttons[button_id].is_init_state_changed = 0;
    buttons[button_id].shift_reg = 0x55;
}

static void PollButtons(void)
{
    for(uint8_t ik=0; ik<BUTTONS_NUMBER; ik++)
    {
        uint32_t pin_state = LL_GPIO_IsInputPinSet(buttons[ik].gpio, buttons[ik].pin)^buttons[ik].polarity;

        /*  BUTTON BOUNCING FILTER  */
        buttons[ik].shift_reg = (buttons[ik].shift_reg<<1)|pin_state;

        /*  DETERMINE STATE    */
        switch (buttons[ik].state)
        {
            case (uint32_t)BUTTON_STATE_PRESSED:
                if((buttons[ik].shift_reg&(0xFFFFFFFF>>(32-PRESSED_TIME_ms))) == 0)
                {
                    buttons[ik].state = BUTTON_STATE_RELEASED;
                    buttons[ik].is_init_state_changed = 1;
                    if(buttons[ik].ButtonCallback != NULL)
                        buttons[ik].ButtonCallback((enum buttons_e)ik, STATE_CHANGED_CALLBACK, BUTTON_NO_ACTION, BUTTON_STATE_RELEASED);
                }
                break;
            case (uint32_t)BUTTON_STATE_RELEASED:
                if((buttons[ik].shift_reg&(0xFFFFFFFF>>(32-PRESSED_TIME_ms))) == (uint32_t)(0xFFFFFFFF>>(32-PRESSED_TIME_ms)))
                {
                    buttons[ik].state = BUTTON_STATE_PRESSED;
                    buttons[ik].is_init_state_changed = 1;
                    if(buttons[ik].ButtonCallback != NULL)
                        buttons[ik].ButtonCallback((enum buttons_e)ik, STATE_CHANGED_CALLBACK, BUTTON_NO_ACTION, BUTTON_STATE_PRESSED);
                }
                break;
            default:
                if((buttons[ik].shift_reg&(0xFFFFFFFF>>(32-PRESSED_TIME_ms))) == 0)
                {
                    buttons[ik].state = BUTTON_STATE_RELEASED;
                    if(buttons[ik].ButtonCallback != NULL)
                        buttons[ik].ButtonCallback((enum buttons_e)ik, INIT_COMPLETE_CALLBACK, BUTTON_NO_ACTION, BUTTON_STATE_RELEASED);
                    break;
                }
                if((buttons[ik].shift_reg&(0xFFFFFFFF>>(32-PRESSED_TIME_ms))) == (uint32_t)(0xFFFFFFFF>>(32-PRESSED_TIME_ms)))
                {
                    buttons[ik].state = BUTTON_STATE_PRESSED;
                    if(buttons[ik].ButtonCallback != NULL)
                        buttons[ik].ButtonCallback((enum buttons_e)ik, INIT_COMPLETE_CALLBACK, BUTTON_NO_ACTION, BUTTON_STATE_PRESSED);
                    break;
                }
                break;
        }

        /*  Prevent action detection on buttons init state  */
        if(buttons[ik].is_init_state_changed==0)
            continue;

        /*  ACTION DETECTION    */
        switch(buttons[ik].state)
        {
            case BUTTON_STATE_PRESSED:
                if(buttons[ik].press_counter_ms==HOLD_TIME_ms)
                {
                    if(buttons[ik].ButtonCallback != NULL)
                        buttons[ik].ButtonCallback((enum buttons_e)ik, ACTION_CALLBACK, BUTTON_ACTION_HOLD, BUTTON_STATE_PRESSED);
                }
                if(buttons[ik].press_counter_ms<=HOLD_TIME_ms)
                    buttons[ik].press_counter_ms++;
                break;

            case BUTTON_STATE_RELEASED:
                if((buttons[ik].press_counter_ms)&&(buttons[ik].press_counter_ms<=HOLD_TIME_ms))
                {
                    if(buttons[ik].ButtonCallback != NULL)
                        buttons[ik].ButtonCallback((enum buttons_e)ik, ACTION_CALLBACK, BUTTON_ACTION_CLICK, BUTTON_STATE_RELEASED);
                }
                buttons[ik].press_counter_ms = 0;
                break;

            default:
                buttons[ik].press_counter_ms = 0;
                break;
        }
    }
}

struct button_s Button = {.AddButton = AddButton,
                            .GetState = GetState,
                            .PollButtons = PollButtons};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
