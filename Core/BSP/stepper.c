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
#include "stepper.h"
#include "l6474_drvr.h"
#include "stm32f4xx_ll_tim.h"
#include "board_gpio_defines.h"
#include "stm32f4xx_ll_gpio.h"
#include "spi_gpo_defines.h"

#define MOTORS_NUMBER 	4
#define MAX_SPEED_DIV 	32
#define V1_FACTOR 		0.78f

/**
 * V1_FACTOR is essentially a so-called magic number. It was 
 * defined during the algorithm modelling in Python as a value 
 * giving the maximum number of transition speeds and their
 * longest duration dx. The optimal value lies between 0.75 and 0.80.
 * I do not know the mathematical nature of this magic number
 * and why it takes such values.
*/


void StepperMotor_1_Init(void)
{
	uint32_t temp;

	/*	Hard reset L6474	*/
	L6474H.HardReset(STEPPER_MOTOR_1);
		
	/*	Set L6474 parameters	*/
	// temp = L6474_OVERCURRENT_MASK|L6474_THERMAL_SHUTDOWN_MASK|L6474_THERMAL_WARNING_MASK|L6474_WRONG_COMMAND_MASK;
	temp = L6474_OVERCURRENT_MASK|L6474_THERMAL_SHUTDOWN_MASK;
	L6474H.SetParam(STEPPER_MOTOR_1, L6474_ALARM_EN, temp, L6474_ALARM_EN_LENGTH);
		
	temp = (0x1F<<L6474_TOFF_POS)|(0x00<<L6474_POW_SR_POS)
					|(0x00<<L6474_OC_SD_POS)|(0x00<<L6474_EN_TQREG_POS)
					|(0x00<<L6474_EXT_CLK_POS)|(0x03<<L6474_OSC_SEL_POS);
	L6474H.SetParam(STEPPER_MOTOR_1, L6474_CONFIG, temp, L6474_CONFIG_LENGTH);

	temp = 0x1F<<L6474_TOFF_MIN_POS;
	L6474H.SetParam(STEPPER_MOTOR_1, L6474_TOFF_MIN, temp, L6474_TOFF_MIN_LENGTH);
		
	L6474H.SetStepMode(STEPPER_MOTOR_1, L6474_FULL_STEP);
	L6474H.SetCurrentLimit_mA(STEPPER_MOTOR_1, 1100);
		
    L6474H.SetDirection(STEPPER_MOTOR_1, DIRECTION_FORWARD);
}

void StepperMotor_2_Init(void)
{
	uint32_t temp;

	/*	Hard reset L6474	*/
	L6474H.HardReset(STEPPER_MOTOR_2);
		
	/*	Set L6474 parameters	*/
	temp = L6474_OVERCURRENT_MASK|L6474_THERMAL_SHUTDOWN_MASK|L6474_THERMAL_WARNING_MASK|L6474_WRONG_COMMAND_MASK;
	L6474H.SetParam(STEPPER_MOTOR_2, L6474_ALARM_EN, temp, L6474_ALARM_EN_LENGTH);
		
	temp = (0x0A<<L6474_TOFF_POS)|(0x00<<L6474_POW_SR_POS)
					|(0x01<<L6474_OC_SD_POS)|(0x00<<L6474_EN_TQREG_POS)
					|(0x00<<L6474_EXT_CLK_POS)|(0x03<<L6474_OSC_SEL_POS);
	L6474H.SetParam(STEPPER_MOTOR_2, L6474_CONFIG, temp, L6474_CONFIG_LENGTH);
		
	L6474H.SetStepMode(STEPPER_MOTOR_2, L6474_FULL_STEP);
	L6474H.SetCurrentLimit_mA(STEPPER_MOTOR_2, 1500);
		
    L6474H.SetDirection(STEPPER_MOTOR_2, DIRECTION_FORWARD);
}

void StepperMotor_3_Init(void)
{
	uint32_t temp;

	/*	Hard reset L6474	*/
	L6474H.HardReset(STEPPER_MOTOR_3);
		
	/*	Set L6474 parameters	*/
	temp = L6474_OVERCURRENT_MASK|L6474_THERMAL_SHUTDOWN_MASK|L6474_THERMAL_WARNING_MASK|L6474_WRONG_COMMAND_MASK;
	L6474H.SetParam(STEPPER_MOTOR_3, L6474_ALARM_EN, temp, L6474_ALARM_EN_LENGTH);
		
	temp = (0x0A<<L6474_TOFF_POS)|(0x00<<L6474_POW_SR_POS)
					|(0x01<<L6474_OC_SD_POS)|(0x00<<L6474_EN_TQREG_POS)
					|(0x00<<L6474_EXT_CLK_POS)|(0x03<<L6474_OSC_SEL_POS);
	L6474H.SetParam(STEPPER_MOTOR_3, L6474_CONFIG, temp, L6474_CONFIG_LENGTH);
		
	L6474H.SetStepMode(STEPPER_MOTOR_3, L6474_FULL_STEP);
	L6474H.SetCurrentLimit_mA(STEPPER_MOTOR_3, 1100);
		
    L6474H.SetDirection(STEPPER_MOTOR_3, DIRECTION_FORWARD);
}

void StepperMotor_4_Init(void)
{
	uint32_t temp;

	/*	Hard reset L6474	*/
	L6474H.HardReset(STEPPER_MOTOR_4);
		
	/*	Set L6474 parameters	*/
	temp = L6474_OVERCURRENT_MASK|L6474_THERMAL_SHUTDOWN_MASK|L6474_THERMAL_WARNING_MASK|L6474_WRONG_COMMAND_MASK;
	L6474H.SetParam(STEPPER_MOTOR_4, L6474_ALARM_EN, temp, L6474_ALARM_EN_LENGTH);
		
	temp = (0x0A<<L6474_TOFF_POS)|(0x00<<L6474_POW_SR_POS)
					|(0x01<<L6474_OC_SD_POS)|(0x00<<L6474_EN_TQREG_POS)
					|(0x00<<L6474_EXT_CLK_POS)|(0x03<<L6474_OSC_SEL_POS);
	L6474H.SetParam(STEPPER_MOTOR_4, L6474_CONFIG, temp, L6474_CONFIG_LENGTH);
		
	L6474H.SetStepMode(STEPPER_MOTOR_4, L6474_FULL_STEP);
	L6474H.SetCurrentLimit_mA(STEPPER_MOTOR_4, 1100);
		
    L6474H.SetDirection(STEPPER_MOTOR_4, DIRECTION_FORWARD);
}


struct speed_s{
	uint32_t div;
	uint32_t dx;
};

struct motor_s{
	uint32_t ll_tim_channel;
	int32_t present_position;
	int32_t goal_position;
	uint32_t acc_dwcntr;
	uint32_t div_dwcntr;
	uint32_t div_dwcntr_atrld;
	uint32_t start_dec_point;
	uint32_t speed_seq_index;
	uint32_t speed_seq_index_comp;
	uint32_t speed_seq_len;
	struct speed_s speed_seq[MAX_SPEED_DIV*2];
};

static struct motor_s motor[MOTORS_NUMBER] = {	[0].ll_tim_channel = LL_TIM_CHANNEL_CH1,
												[1].ll_tim_channel = LL_TIM_CHANNEL_CH2,
												[2].ll_tim_channel = LL_TIM_CHANNEL_CH3,
												[3].ll_tim_channel = LL_TIM_CHANNEL_CH4};



static inline uint32_t TIM_IsActiveFlag_CC(TIM_TypeDef *TIMx, uint32_t m_index)
{
	return ((READ_BIT(TIMx->SR, (1<<(m_index+1))) == (1<<(m_index+1))) ? 1UL : 0UL);
}

static inline void TIM_ClearFlag_CC(TIM_TypeDef *TIMx, uint32_t m_index)
{
	WRITE_REG(TIMx->SR, ~ (1<<(m_index+1)));
}

static inline void TIM_DisableIT_CC(TIM_TypeDef *TIMx, uint32_t m_index)
{
  	CLEAR_BIT(TIMx->DIER, (1<<(m_index+1)));
}

static inline void TIM_EnableIT_CC(TIM_TypeDef *TIMx, uint32_t m_index)
{
  	SET_BIT(TIMx->DIER, (1<<(m_index+1)));
}

static inline uint32_t TIM_CC_IsEnabledChannel(TIM_TypeDef *TIMx, uint32_t m_index)
{
  	return ((READ_BIT(TIMx->CCER, motor[m_index].ll_tim_channel) == (motor[m_index].ll_tim_channel)) ? 1UL : 0UL);
}

static inline void TIM_CC_DisableChannel(TIM_TypeDef *TIMx, uint32_t m_index)
{
  	CLEAR_BIT(TIMx->CCER, motor[m_index].ll_tim_channel);
}

static inline void TIM_CC_EnableChannel(TIM_TypeDef *TIMx, uint32_t m_index)
{
  	SET_BIT(TIMx->CCER, motor[m_index].ll_tim_channel);
}



void TIM1_Steppers_PWM_CC_IRQ_Process(void)
{
	for(uint8_t m_index = 0; m_index < MOTORS_NUMBER; m_index ++)
	{
		if(TIM_IsActiveFlag_CC(TIM1,m_index))
		{
			TIM_ClearFlag_CC(TIM1,m_index);

			if(TIM_CC_IsEnabledChannel(TIM1, m_index))
			{
				/*	Calculate *pReg value which is common for LL_TIM_OC_GetMode and LL_TIM_OC_SetMode	*/
				uint8_t iChannel = TIM_GET_CHANNEL_INDEX(motor[m_index].ll_tim_channel);
  				__IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&TIM1->CCMR1) + OFFSET_TAB_CCMRx[iChannel]));

				/*	uint32_t mode = LL_TIM_OC_GetMode(TIM1, motor[m_index].ll_tim_channel)	*/
				uint32_t mode = READ_BIT(*pReg, ((TIM_CCMR1_OC1M | TIM_CCMR1_CC1S) << SHIFT_TAB_OCxx[iChannel])) >> SHIFT_TAB_OCxx[iChannel];

				/*	Update Step Counter	*/
				if(mode == LL_TIM_OCMODE_PWM1)
				{
					if(motor[m_index].present_position == motor[m_index].goal_position){
						TIM_CC_DisableChannel(TIM1, m_index);
						TIM_DisableIT_CC(TIM1,m_index);
						continue;
					}

					/* Just because "motor[m_index].present_position == motor[m_index].goal_position" case was addressed above	*/
					if(motor[m_index].present_position < motor[m_index].goal_position)
						motor[m_index].present_position ++;
					else
						motor[m_index].present_position --;

					/*	Deceleration Start	*/
					if(motor[m_index].present_position == motor[m_index].start_dec_point)
						motor[m_index].speed_seq_index_comp = motor[m_index].speed_seq_len;
				}

				/*	Acceleration Downcounter	*/
				if(motor[m_index].acc_dwcntr == 0){
					if(motor[m_index].speed_seq_index < motor[m_index].speed_seq_index_comp){
						motor[m_index].div_dwcntr_atrld = motor[m_index].speed_seq[motor[m_index].speed_seq_index].div - 1;
						motor[m_index].acc_dwcntr = motor[m_index].speed_seq[motor[m_index].speed_seq_index].dx - 1;
						motor[m_index].speed_seq_index ++;
					}	
				}else{
					motor[m_index].acc_dwcntr --;
				}

				/*	Speed Division Downcounter	*/
				if(motor[m_index].div_dwcntr == 0){
					motor[m_index].div_dwcntr = motor[m_index].div_dwcntr_atrld;
					/*	LL_TIM_OC_SetMode(TIM1, motor[m_index].ll_tim_channel, LL_TIM_OCMODE_PWM1)	*/
					MODIFY_REG(*pReg, ((TIM_CCMR1_OC1M  | TIM_CCMR1_CC1S) << SHIFT_TAB_OCxx[iChannel]), LL_TIM_OCMODE_PWM1 << SHIFT_TAB_OCxx[iChannel]);
				}else{
					motor[m_index].div_dwcntr --;
					/*	LL_TIM_OC_SetMode(TIM1, motor[m_index].ll_tim_channel, LL_TIM_OCMODE_FORCED_INACTIVE)	*/
					MODIFY_REG(*pReg, ((TIM_CCMR1_OC1M  | TIM_CCMR1_CC1S) << SHIFT_TAB_OCxx[iChannel]), LL_TIM_OCMODE_FORCED_INACTIVE << SHIFT_TAB_OCxx[iChannel]);
				}
				
			}
		}
	}
	LL_TIM_GenerateEvent_COM(TIM1);
}

// LL_GPIO_SetOutputPin(GPIOD, PWM1_PD15);
// LL_GPIO_ResetOutputPin(GPIOD, PWM1_PD15);

/*  Stop Stepper Motor  */
static void StopMotor(enum l6474h_motor_s m_index)
{
	TIM_CC_DisableChannel(TIM1, m_index);
	LL_TIM_OC_SetMode(TIM1, motor[m_index].ll_tim_channel, LL_TIM_OCMODE_FORCED_INACTIVE);
	LL_TIM_GenerateEvent_COM(TIM1);
	TIM_DisableIT_CC(TIM1, m_index);
}

static void SetZeroPosisition(enum l6474h_motor_s m_index)
{
	StopMotor(m_index);
	motor[m_index].present_position = 0;
	motor[m_index].goal_position = 0;
}

static int32_t GetPresentPosisition(enum l6474h_motor_s m_index)
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
static void StepToPosisition(enum l6474h_motor_s m_index, int32_t position, uint32_t edge_div, uint32_t working_div, uint32_t acc_ramp_ms)
{
	if(motor[m_index].present_position == position)
		return;

	if(edge_div < working_div)
		return;

	if(edge_div > MAX_SPEED_DIV)
		edge_div = MAX_SPEED_DIV;

	if(acc_ramp_ms < 10)
		edge_div = working_div;

	StopMotor(m_index);

	static float tg, v1;
	static uint32_t dx, steps, speed_seq_len;


	motor[m_index].goal_position = position;
	motor[m_index].acc_dwcntr = 0;
	motor[m_index].div_dwcntr = 0;
	motor[m_index].div_dwcntr_atrld = 0;

	uint32_t distance = (position > motor[m_index].present_position) ? (position - motor[m_index].present_position) : (motor[m_index].present_position - position);

	uint32_t f_pulses = SystemCoreClock/(LL_TIM_GetPrescaler(TIM1)+1)/(LL_TIM_GetAutoReload(TIM1)+1);
	uint32_t acc_pulses = f_pulses*acc_ramp_ms/1000;


	while(1)
	{
		if(working_div == edge_div){
			speed_seq_len = 0;
			motor[m_index].div_dwcntr_atrld = working_div - 1;
			break;
		}

		tg = (1.0f/working_div - 1.0f/edge_div)/acc_pulses;

		v1 = (1.0f/working_div - 1.0f/(working_div + 1))*V1_FACTOR;
		dx = v1*2.0f/tg;

		motor[m_index].speed_seq[MAX_SPEED_DIV].div = working_div;
		motor[m_index].speed_seq[MAX_SPEED_DIV].dx = dx/8;
		motor[m_index].speed_seq[MAX_SPEED_DIV - 1].div = working_div;
		motor[m_index].speed_seq[MAX_SPEED_DIV - 1].dx = dx/8;

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

			motor[m_index].speed_seq[MAX_SPEED_DIV + speed_seq_len].div = div_new;
			motor[m_index].speed_seq[MAX_SPEED_DIV + speed_seq_len].dx = dx;
			motor[m_index].speed_seq[MAX_SPEED_DIV - 1 - speed_seq_len].div = div_new;
			motor[m_index].speed_seq[MAX_SPEED_DIV - 1 - speed_seq_len].dx = dx;

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

	motor[m_index].speed_seq_index = MAX_SPEED_DIV - speed_seq_len;
	motor[m_index].speed_seq_index_comp = MAX_SPEED_DIV;
	motor[m_index].speed_seq_len = MAX_SPEED_DIV + speed_seq_len - 1;

	if(position > motor[m_index].present_position){
		motor[m_index].start_dec_point = position - steps;
		L6474H.SetDirection(m_index,DIRECTION_FORWARD);
	}else{
		motor[m_index].start_dec_point = position + steps;
		L6474H.SetDirection(m_index,DIRECTION_BACKWARD);
	}

	TIM_CC_EnableChannel(TIM1, m_index);
	LL_TIM_OC_SetMode(TIM1, motor[m_index].ll_tim_channel, LL_TIM_OCMODE_FORCED_INACTIVE);
	LL_TIM_GenerateEvent_COM(TIM1);
	TIM_EnableIT_CC(TIM1, m_index);
}

static void Enable(enum l6474h_motor_s m_index)
{
	L6474H.Enable(m_index);
}

static void Disable(enum l6474h_motor_s m_index)
{
	L6474H.Disable(m_index);
}

static void (*MotorInit[MOTORS_NUMBER])(void) = {StepperMotor_1_Init, StepperMotor_2_Init, StepperMotor_3_Init, StepperMotor_4_Init};

static void Init(enum l6474h_motor_s m_index)
{
	MotorInit[m_index]();
}

const struct stepper_s Stepper = {	.SetZeroPosisition = SetZeroPosisition,
									.StepToPosisition = StepToPosisition,
									.GetPresentPosisition = GetPresentPosisition,
									.StopMotor = StopMotor,
									.Enable = Enable,
									.Disable = Disable,
									.Init = Init};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
