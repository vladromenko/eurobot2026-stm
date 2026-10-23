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
#include "pwm.hpp"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.hpp"


namespace {

TIM_TypeDef* const tim_group[] = {TIM4,TIM5};
TIM_TypeDef* const tim_output[] = {TIM4,TIM4,TIM4,TIM4,TIM5,TIM5,TIM5,TIM5};

constexpr uint32_t tim_channel[] = {LL_TIM_CHANNEL_CH4,LL_TIM_CHANNEL_CH3,LL_TIM_CHANNEL_CH2,LL_TIM_CHANNEL_CH1,
                                  LL_TIM_CHANNEL_CH1,LL_TIM_CHANNEL_CH2,LL_TIM_CHANNEL_CH3,LL_TIM_CHANNEL_CH4};

volatile uint32_t* const tim_ccr[] = {&TIM4->CCR4,&TIM4->CCR3,&TIM4->CCR2,&TIM4->CCR1,&TIM5->CCR1,&TIM5->CCR2,&TIM5->CCR3,&TIM5->CCR4};
volatile uint32_t* const tim_ccer[] = {&TIM4->CCER,&TIM4->CCER,&TIM4->CCER,&TIM4->CCER,&TIM5->CCER,&TIM5->CCER,&TIM5->CCER,&TIM5->CCER};

GPIO_TypeDef* const pwm_gpio[] = {GPIOD,GPIOD,GPIOD,GPIOD,GPIOA,GPIOA,GPIOA,GPIOA};
constexpr uint32_t pwm_pin[] = {PWM1_PD15,PWM2_PD14,PWM3_PD13,PWM4_PD12,
                              PWM5_PA0,PWM6_PA1,PWM7_PA2,PWM8_PA3};



void TIM_OC_SetCompareCH(uint32_t pwm_output, uint32_t cc_value)
{
  WRITE_REG(*tim_ccr[pwm_output], cc_value);
}

void TIM_CC_EnableChannel(uint32_t pwm_output)
{
  SET_BIT(*tim_ccer[pwm_output], tim_channel[pwm_output]);
}

void TIM_CC_DisableChannel(uint32_t pwm_output)
{
  CLEAR_BIT(*tim_ccer[pwm_output], tim_channel[pwm_output]);
}

}

PWM_Driver PWM;



uint32_t PWM_Driver::GetGroupTimAutoreload(pwm_group_e pwm_group)
{
  return LL_TIM_GetAutoReload(tim_group[pwm_group]);
}

void PWM_Driver::SetOutput_CC(pwm_output_e pwm_output, uint32_t cc_value)
{
  TIM_OC_SetCompareCH(pwm_output, cc_value);
}

void PWM_Driver::SetGroupFrequency(pwm_group_e pwm_group, uint32_t frequency_hz)
{ 
  uint32_t prescaler = LL_TIM_GetPrescaler(tim_group[pwm_group]);
  LL_TIM_SetAutoReload(tim_group[pwm_group], (84000000/(prescaler+1)/frequency_hz - 1));
}

void PWM_Driver::SetOutputDutyCycle(pwm_output_e pwm_output, uint32_t duty_cycle)
{
  if(duty_cycle>100)
    return;

  uint32_t cc_value = ((LL_TIM_GetAutoReload(tim_output[pwm_output])+1)*duty_cycle)/100;

  TIM_OC_SetCompareCH(pwm_output, cc_value);
}

void PWM_Driver::Enable(pwm_output_e pwm_output)
{
  LL_GPIO_InitTypeDef GPIO_Struct;

  GPIO_Struct.Pin = pwm_pin[pwm_output];
	GPIO_Struct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Struct.Alternate  = LL_GPIO_AF_2;
	
	LL_GPIO_Init(pwm_gpio[pwm_output], &GPIO_Struct);

  TIM_CC_EnableChannel(pwm_output);
  LL_TIM_GenerateEvent_UPDATE(tim_output[pwm_output]);
}

void PWM_Driver::Disable(pwm_output_e pwm_output)
{
  LL_GPIO_InitTypeDef GPIO_Struct;

  GPIO_Struct.Pin = pwm_pin[pwm_output];
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	LL_GPIO_Init(pwm_gpio[pwm_output], &GPIO_Struct);

  TIM_CC_DisableChannel(pwm_output);
  LL_TIM_GenerateEvent_UPDATE(tim_output[pwm_output]);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
