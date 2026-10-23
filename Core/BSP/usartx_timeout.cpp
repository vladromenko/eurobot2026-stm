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


#include "std_def.hpp"
#include "usartx_timeout.hpp" 
#include "stm32f4xx_ll_tim.h"

#include "board_gpio_defines.hpp"
#include "stm32f4xx_ll_gpio.h"

#define CHANNELS_NUMBER     6
#define TIM_PERIOD_US       100
#define TIMx                TIM14



struct timeout_s{
    uint32_t period_counter;
    void (*TimeoutCallback)(void* callback_params);
    void* callback_params;
};

struct timeout_s timeout_list[CHANNELS_NUMBER] = {0};
uint8_t working_flag = 0;


void SetTimeout(uint8_t timeout_channel, uint32_t timeout_us, void (*TimeoutCallback)(void* callback_params), void* callback_params)
{
    timeout_list[timeout_channel].period_counter = timeout_us/TIM_PERIOD_US;
    timeout_list[timeout_channel].TimeoutCallback = TimeoutCallback;
    timeout_list[timeout_channel].callback_params = callback_params;

    if(working_flag==0){
        working_flag = 1;
        LL_TIM_EnableCounter(TIMx);
        LL_TIM_EnableIT_UPDATE(TIMx);
    }
}

void ResetTimeout(uint8_t timeout_channel)
{
    timeout_list[timeout_channel].period_counter = 0;
}

static void IRQ_Process(void)
{
    uint8_t stop_flag = 1;

    for(uint8_t ik=0; ik<CHANNELS_NUMBER; ik++)
    {
        if(timeout_list[ik].period_counter)
        {
            timeout_list[ik].period_counter--;
            
            if(timeout_list[ik].period_counter==0)
                timeout_list[ik].TimeoutCallback(timeout_list[ik].callback_params);
            else
                stop_flag = 0;
        }
    }  

    if(stop_flag){
        LL_TIM_DisableCounter(TIMx);
        LL_TIM_DisableIT_UPDATE(TIMx);
        working_flag = 0;
    }
}

void USARTx_Timeout_IRQ_Process(void)
{
	if(LL_TIM_IsActiveFlag_UPDATE(TIMx))
	{
		LL_TIM_ClearFlag_UPDATE(TIMx);

		if(LL_TIM_IsEnabledIT_UPDATE(TIMx))
            IRQ_Process();
    }	
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
