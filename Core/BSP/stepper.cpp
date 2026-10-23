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
#include "stepper.hpp"
#include "l6474_drvr.hpp"
#include "stm32f4xx_ll_tim.h"
#include "board_gpio_defines.hpp"
#include "stm32f4xx_ll_gpio.h"
#include "spi_gpo_defines.hpp"

namespace {

/**
 * V1_FACTOR is essentially a so-called magic number. It was 
 * defined during the algorithm modelling in Python as a value 
 * giving the maximum number of transition speeds and their
 * longest duration dx. The optimal value lies between 0.75 and 0.80.
 * I do not know the mathematical nature of this magic number
 * and why it takes such values.
*/
constexpr float V1_FACTOR = 0.78f;

constexpr uint32_t ll_tim_channel[] = {LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2, LL_TIM_CHANNEL_CH3, LL_TIM_CHANNEL_CH4};

/*	L6474 settings applied by Init()	*/
struct motor_config_s{
	uint32_t alarm_en;
	uint32_t config;
	uint32_t toff_min;		/*	0 - leave the default value	*/
	float current_limit_mA;
};

constexpr uint32_t ALARMS_ALL = L6474_OVERCURRENT_MASK|L6474_THERMAL_SHUTDOWN_MASK|L6474_THERMAL_WARNING_MASK|L6474_WRONG_COMMAND_MASK;

constexpr uint32_t L6474_ConfigValue(uint32_t toff, uint32_t oc_sd)
{
	return (toff<<L6474_TOFF_POS)|(0x00<<L6474_POW_SR_POS)
			|(oc_sd<<L6474_OC_SD_POS)|(0x00<<L6474_EN_TQREG_POS)
			|(0x00<<L6474_EXT_CLK_POS)|(0x03<<L6474_OSC_SEL_POS);
}

constexpr motor_config_s motor_config[] = {
	/*	STEPPER_MOTOR_1	*/
	{.alarm_en = L6474_OVERCURRENT_MASK|L6474_THERMAL_SHUTDOWN_MASK,
	 .config = L6474_ConfigValue(0x1F, 0x00),
	 .toff_min = 0x1F<<L6474_TOFF_MIN_POS,
	 .current_limit_mA = 1100},
	/*	STEPPER_MOTOR_2	*/
	{.alarm_en = ALARMS_ALL, .config = L6474_ConfigValue(0x0A, 0x01), .toff_min = 0, .current_limit_mA = 1500},
	/*	STEPPER_MOTOR_3	*/
	{.alarm_en = ALARMS_ALL, .config = L6474_ConfigValue(0x0A, 0x01), .toff_min = 0, .current_limit_mA = 1100},
	/*	STEPPER_MOTOR_4	*/
	{.alarm_en = ALARMS_ALL, .config = L6474_ConfigValue(0x0A, 0x01), .toff_min = 0, .current_limit_mA = 1100},
};

inline uint32_t TIM_IsActiveFlag_CC(TIM_TypeDef *TIMx, uint32_t m_index)
{
	return ((READ_BIT(TIMx->SR, (1UL<<(m_index+1))) == (1UL<<(m_index+1))) ? 1UL : 0UL);
}

inline void TIM_ClearFlag_CC(TIM_TypeDef *TIMx, uint32_t m_index)
{
	WRITE_REG(TIMx->SR, ~ (1<<(m_index+1)));
}

inline void TIM_DisableIT_CC(TIM_TypeDef *TIMx, uint32_t m_index)
{
  	CLEAR_BIT(TIMx->DIER, (1<<(m_index+1)));
}

inline void TIM_EnableIT_CC(TIM_TypeDef *TIMx, uint32_t m_index)
{
  	SET_BIT(TIMx->DIER, (1<<(m_index+1)));
}

inline uint32_t TIM_CC_IsEnabledChannel(TIM_TypeDef *TIMx, uint32_t m_index)
{
  	return ((READ_BIT(TIMx->CCER, ll_tim_channel[m_index]) == (ll_tim_channel[m_index])) ? 1UL : 0UL);
}

inline void TIM_CC_DisableChannel(TIM_TypeDef *TIMx, uint32_t m_index)
{
  	CLEAR_BIT(TIMx->CCER, ll_tim_channel[m_index]);
}

inline void TIM_CC_EnableChannel(TIM_TypeDef *TIMx, uint32_t m_index)
{
  	SET_BIT(TIMx->CCER, ll_tim_channel[m_index]);
}

}

constinit StepperDriver Stepper;


void StepperDriver::Init(l6474h_motor_s m_index)
{
	const motor_config_s& cfg = motor_config[m_index];

	/*	Hard reset L6474	*/
	L6474H.HardReset(m_index);
		
	/*	Set L6474 parameters	*/
	L6474H.SetParam(m_index, L6474_ALARM_EN, cfg.alarm_en, L6474_ALARM_EN_LENGTH);
	L6474H.SetParam(m_index, L6474_CONFIG, cfg.config, L6474_CONFIG_LENGTH);

	if(cfg.toff_min)
		L6474H.SetParam(m_index, L6474_TOFF_MIN, cfg.toff_min, L6474_TOFF_MIN_LENGTH);
		
	L6474H.SetStepMode(m_index, L6474_FULL_STEP);
	L6474H.SetCurrentLimit_mA(m_index, cfg.current_limit_mA);
		
	L6474H.SetDirection(m_index, DIRECTION_FORWARD);
}

void StepperDriver::TIM_IRQ_Process()
{
	for(uint8_t m_index = 0; m_index < MOTORS_NUMBER; m_index ++)
	{
		if(TIM_IsActiveFlag_CC(TIM1,m_index))
		{
			TIM_ClearFlag_CC(TIM1,m_index);

			if(TIM_CC_IsEnabledChannel(TIM1, m_index))
			{
				Motor& m = motor[m_index];

				/*	Calculate *pReg value which is common for LL_TIM_OC_GetMode and LL_TIM_OC_SetMode	*/
				uint8_t iChannel = TIM_GET_CHANNEL_INDEX(ll_tim_channel[m_index]);
  				__IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TIM1->CCMR1) + OFFSET_TAB_CCMRx[iChannel]));

				/*	uint32_t mode = LL_TIM_OC_GetMode(TIM1, ll_tim_channel[m_index])	*/
				uint32_t mode = READ_BIT(*pReg, ((TIM_CCMR1_OC1M | TIM_CCMR1_CC1S) << SHIFT_TAB_OCxx[iChannel])) >> SHIFT_TAB_OCxx[iChannel];

				/*	Update Step Counter	*/
				if(mode == LL_TIM_OCMODE_PWM1)
				{
					if(m.present_position == m.goal_position){
						TIM_CC_DisableChannel(TIM1, m_index);
						TIM_DisableIT_CC(TIM1,m_index);
						continue;
					}

					/* Just because "present_position == goal_position" case was addressed above	*/
					if(m.present_position < m.goal_position)
						m.present_position ++;
					else
						m.present_position --;

					/*	Deceleration Start	*/
					if(m.present_position == (int32_t)m.start_dec_point)
						m.speed_seq_index_comp = m.speed_seq_len;
				}

				/*	Acceleration Downcounter	*/
				if(m.acc_dwcntr == 0){
					if(m.speed_seq_index < m.speed_seq_index_comp){
						m.div_dwcntr_atrld = m.speed_seq[m.speed_seq_index].div - 1;
						m.acc_dwcntr = m.speed_seq[m.speed_seq_index].dx - 1;
						m.speed_seq_index ++;
					}	
				}else{
					m.acc_dwcntr --;
				}

				/*	Speed Division Downcounter	*/
				if(m.div_dwcntr == 0){
					m.div_dwcntr = m.div_dwcntr_atrld;
					/*	LL_TIM_OC_SetMode(TIM1, ll_tim_channel[m_index], LL_TIM_OCMODE_PWM1)	*/
					MODIFY_REG(*pReg, ((TIM_CCMR1_OC1M  | TIM_CCMR1_CC1S) << SHIFT_TAB_OCxx[iChannel]), LL_TIM_OCMODE_PWM1 << SHIFT_TAB_OCxx[iChannel]);
				}else{
					m.div_dwcntr --;
					/*	LL_TIM_OC_SetMode(TIM1, ll_tim_channel[m_index], LL_TIM_OCMODE_FORCED_INACTIVE)	*/
					MODIFY_REG(*pReg, ((TIM_CCMR1_OC1M  | TIM_CCMR1_CC1S) << SHIFT_TAB_OCxx[iChannel]), LL_TIM_OCMODE_FORCED_INACTIVE << SHIFT_TAB_OCxx[iChannel]);
				}
				
			}
		}
	}
	LL_TIM_GenerateEvent_COM(TIM1);
}

/*  Stop Stepper Motor  */
void StepperDriver::StopMotor(l6474h_motor_s m_index)
{
	TIM_CC_DisableChannel(TIM1, m_index);
	LL_TIM_OC_SetMode(TIM1, ll_tim_channel[m_index], LL_TIM_OCMODE_FORCED_INACTIVE);
	LL_TIM_GenerateEvent_COM(TIM1);
	TIM_DisableIT_CC(TIM1, m_index);
}

void StepperDriver::SetZeroPosisition(l6474h_motor_s m_index)
{
	StopMotor(m_index);
	motor[m_index].present_position = 0;
	motor[m_index].goal_position = 0;
}

int32_t StepperDriver::GetPresentPosisition(l6474h_motor_s m_index)
{
	return motor[m_index].present_position;
}


/**
 * Step "m_index" motor to "position" with start/end speed division of "edge_div",
 * the maximum speed division of "working_div" and acceleration/deceleration
 * period of "acc_ramp_ms" milliseconds. The max value of any division is MAX_SPEED_DIV.
 * The "edge_div" should be greater than or equal to "working_div".
 * The "acc_ramp_ms" should be greater then 10 ms, otherwise it is considered as 0 and
 * the "working_div" speed is applied immidiately.
*/
void StepperDriver::StepToPosisition(l6474h_motor_s m_index, int32_t position, uint32_t edge_div, uint32_t working_div, uint32_t acc_ramp_ms)
{
	Motor& m = motor[m_index];

	if(m.present_position == position)
		return;

	if(edge_div < working_div)
		return;

	if(edge_div > MAX_SPEED_DIV)
		edge_div = MAX_SPEED_DIV;

	if(acc_ramp_ms < 10)
		edge_div = working_div;

	StopMotor(m_index);

	float tg, v1;
	uint32_t dx, steps = 0, speed_seq_len;


	m.goal_position = position;
	m.acc_dwcntr = 0;
	m.div_dwcntr = 0;
	m.div_dwcntr_atrld = 0;

	uint32_t distance = (position > m.present_position) ? (position - m.present_position) : (m.present_position - position);

	uint32_t f_pulses = SystemCoreClock/(LL_TIM_GetPrescaler(TIM1)+1)/(LL_TIM_GetAutoReload(TIM1)+1);
	uint32_t acc_pulses = f_pulses*acc_ramp_ms/1000;


	while(1)
	{
		if(working_div == edge_div){
			speed_seq_len = 0;
			m.div_dwcntr_atrld = working_div - 1;
			break;
		}

		tg = (1.0f/working_div - 1.0f/edge_div)/acc_pulses;

		v1 = (1.0f/working_div - 1.0f/(working_div + 1))*V1_FACTOR;
		dx = v1*2.0f/tg;

		m.speed_seq[MAX_SPEED_DIV].div = working_div;
		m.speed_seq[MAX_SPEED_DIV].dx = dx/8;
		m.speed_seq[MAX_SPEED_DIV - 1].div = working_div;
		m.speed_seq[MAX_SPEED_DIV - 1].dx = dx/8;

		steps = dx/working_div/8;
		speed_seq_len = 1;

		uint32_t div_prev = working_div;

		for(uint32_t div_new = (div_prev + 1); div_new < (edge_div + 1); div_new ++)
		{
			float dv = 1.0f/div_prev - 1.0f/div_new;
			float v2 = dv - v1;

			if(v2 <= 0) continue;

			dx = dx*v2/v1;

			if(dx == 0) continue;

			div_prev = div_new;
			v1 = v2;

			m.speed_seq[MAX_SPEED_DIV + speed_seq_len].div = div_new;
			m.speed_seq[MAX_SPEED_DIV + speed_seq_len].dx = dx;
			m.speed_seq[MAX_SPEED_DIV - 1 - speed_seq_len].div = div_new;
			m.speed_seq[MAX_SPEED_DIV - 1 - speed_seq_len].dx = dx;

			speed_seq_len ++;
			steps += dx/div_new;

			if(steps*2 > distance){
				working_div ++;
				break;
			}
		}

		if(steps*2 <= distance)
			break;
	}

	m.speed_seq_index = MAX_SPEED_DIV - speed_seq_len;
	m.speed_seq_index_comp = MAX_SPEED_DIV;
	m.speed_seq_len = MAX_SPEED_DIV + speed_seq_len - 1;

	if(position > m.present_position){
		m.start_dec_point = position - steps;
		L6474H.SetDirection(m_index,DIRECTION_FORWARD);
	}else{
		m.start_dec_point = position + steps;
		L6474H.SetDirection(m_index,DIRECTION_BACKWARD);
	}

	TIM_CC_EnableChannel(TIM1, m_index);
	LL_TIM_OC_SetMode(TIM1, ll_tim_channel[m_index], LL_TIM_OCMODE_FORCED_INACTIVE);
	LL_TIM_GenerateEvent_COM(TIM1);
	TIM_EnableIT_CC(TIM1, m_index);
}

void StepperDriver::Enable(l6474h_motor_s m_index)
{
	L6474H.Enable(m_index);
}

void StepperDriver::Disable(l6474h_motor_s m_index)
{
	L6474H.Disable(m_index);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
