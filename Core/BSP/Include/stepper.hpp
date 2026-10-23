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
#ifndef __STEPPER_H
#define __STEPPER_H

#include "std_def.hpp"
#include "l6474_drvr.hpp" 


/*  4 stepper motors: STEP pulses from TIM1 CH1..CH4, L6474H drivers on SPI1  */
class StepperDriver{
public:
    void SetZeroPosisition(l6474h_motor_s m_index);
    void StepToPosisition(l6474h_motor_s m_index, int32_t position, uint32_t edge_div, uint32_t working_div, uint32_t acc_ramp_ms);
    int32_t GetPresentPosisition(l6474h_motor_s m_index);
    void StopMotor(l6474h_motor_s m_index);
    void Enable(l6474h_motor_s m_index);
    void Disable(l6474h_motor_s m_index);
    void Init(l6474h_motor_s m_index);

    /*  Called from TIM1_CC_IRQHandler  */
    void TIM_IRQ_Process();

private:
    static constexpr uint32_t MOTORS_NUMBER = 4;
    static constexpr uint32_t MAX_SPEED_DIV = 32;

    struct Speed{
        uint32_t div;
        uint32_t dx;
    };

    struct Motor{
        int32_t present_position;
        int32_t goal_position;
        uint32_t acc_dwcntr;
        uint32_t div_dwcntr;
        uint32_t div_dwcntr_atrld;
        uint32_t start_dec_point;
        uint32_t speed_seq_index;
        uint32_t speed_seq_index_comp;
        uint32_t speed_seq_len;
        Speed speed_seq[MAX_SPEED_DIV*2];
    };

    Motor motor[MOTORS_NUMBER];
};

extern StepperDriver Stepper;

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
