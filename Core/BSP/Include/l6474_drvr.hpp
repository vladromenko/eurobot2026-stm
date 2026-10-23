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
#ifndef __L6474_DRVR_H
#define __L6474_DRVR_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.hpp"



enum l6474h_motor_s{
  STEPPER_MOTOR_1,
  STEPPER_MOTOR_2,
  STEPPER_MOTOR_3,
  STEPPER_MOTOR_4
};

enum l6474h_motor_dir_s{
  DIRECTION_BACKWARD,
  DIRECTION_FORWARD
};


/*L6474 L6474_T_FAST Register Values ------------------------------------ */
#define L6474_FULL_STEP					0X00		/* Step					                  */
#define L6474_HALF_STEP					0x01		/* Step					                  */
#define L6474_STEP_MODE_4				0X02		/* Microstep		                  */
#define L6474_STEP_MODE_8				0x03		/* Microstep		                  */
#define L6474_STEP_MODE_16			0x04		/* Microstep		                  */
	 
/*L6474 commands define ------------------------------------------------------------ */
#define L6474_SetParam_CMD      0x00    /* Current Position (3 bytes)     */
#define L6474_GetParam_CMD      0x20    /* Current Position (3 bytes)     */
#define L6474_ENABLE_CMD        0xB8    /* Electrical Position (2 bytes) 	*/
#define L6474_DISABLE_CMD       0xA8    /* Electrical Position (2 bytes) 	*/
#define L6474_GetStatus_CMD     0xD0    /* Electrical Position (2 bytes) 	*/
#define L6474_NOP_CMD         	0x00    /* Electrical Position (2 bytes) 	*/


	 
/*L6474 registers define ------------------------------------------------------------ */
#define L6474_ABS_POS           0x01    /* Current Position (3 bytes)     */
#define L6474_EL_POS            0x02    /* Electrical Position (2 bytes) 	*/
#define L6474_MARK              0x03    /* Mark Position (3 bytes) 				*/
#define L6474_TVAL              0x09    /* Reference Current  						*/
#define L6474_T_FAST            0x0E    /* Fast Decay/Fall Step Time  		*/
#define L6474_TON_MIN           0x0F    /* Minimum ON Time  							*/
#define L6474_TOFF_MIN          0x10    /* Minimum OFF Time               */
#define L6474_ADC_OUT          	0x12    /* ADC Output     								*/
#define L6474_OCD_TH           	0x13    /* OCD Threshold     							*/
#define L6474_STEP_MODE         0x16    /* Step Mode     									*/
#define L6474_ALARM_EN          0x17    /* Alarms Enables     						*/
#define L6474_CONFIG            0x18    /* IC Configuration (2 bytes) 		*/
#define L6474_STATUS            0x19    /* Status (2 bytes)    						*/



/*L6474 L6474_ABS_POS Bit mask definition ----------------------------------------------- */
#define L6474_ABS_POS_MASK   								0x3FFFFF	/* Voltage mode bit mask            */
#define L6474_ABS_POS_LENGTH								3

/*L6474 L6474_EL_POS Bit mask definition ------------------------------------------------ */
#define L6474_STEP_MASK											0x0180		/* Step					                    */
#define L6474_MICROSTEP_MASK								0x007F		/* Microstep		                    */
#define L6474_EL_POS_LENGTH									2

/*L6474 L6474_MARK Bit mask definition -------------------------------------------------- */
#define L6474_MARK_MASK											0x3FFFFF	/* Mark					                    */
#define L6474_MARK_LENGTH										3

/*L6474 L6474_TVAL Bit mask definition -------------------------------------------------- */
#define L6474_TVAL_MASK											0x7F			/* Current value, Range: 0 to 0x7F	*/
#define L6474_TVAL_LENGTH										1

/*L6474 L6474_T_FAST Bit mask definition ------------------------------------------------ */
#define L6474_TOFF_FAST_POS									4					/* Step					                    */
#define L6474_TOFF_FAST_MASK								0xF0			/* Step					                    */
#define L6474_FAST_STEP_POS									0					/* Microstep		                    */
#define L6474_FAST_STEP_MASK								0x0F			/* Microstep		                    */
#define L6474_T_FAST_LENGTH									1

/*L6474 L6474_TON_MIN Bit mask definition ----------------------------------------------- */
#define L6474_TON_MIN_POS										0x00			/* TVAL					                    */
#define L6474_TON_MIN_MASK									0x7F			/* TVAL					                    */
#define L6474_TON_MIN_LENGTH								1

/*L6474 L6474_TOFF_MIN Bit mask definition ---------------------------------------------- */
#define L6474_TOFF_MIN_POS									0x00			/* TVAL					                    */
#define L6474_TOFF_MIN_MASK									0x7F			/* TVAL					                    */
#define L6474_TOFF_MIN_LENGTH								1

/*L6474 L6474_ADC_OUT Bit mask definition ----------------------------------------------- */
#define L6474_VADCIN_VREG_MASK							0x20			/* Step					                    */
#define L6474_ADC_OUT_MASK									0x1F			/* Microstep		                    */
#define L6474_ADC_OUT_LENGTH								1

/*L6474 L6474_OCD_TH Bit mask definition ------------------------------------------------ */
#define L6474_OCD_TH_MASK										0x0F			/* TVAL					                    */
#define L6474_OCD_TH_LENGTH									1

/*L6474 L6474_STEP_MODE Bit mask definition --------------------------------------------- */
#define L6474_SYNC_SEL_POS									4					/* Step					                    */
#define L6474_SYNC_SEL_MASK									0x70			/* Step					                    */
#define L6474_STEP_SEL_POS									0					/* Microstep		                    */
#define L6474_STEP_SEL_MASK									0x07			/* Microstep		                    */
#define L6474_STEP_MODE_LENGTH							1

/*L6474 L6474_ALARM_EN Bit mask definition ---------------------------------------------- */
#define L6474_OVERCURRENT_MASK							0x01			/* TVAL					                    */
#define L6474_THERMAL_SHUTDOWN_MASK					0x02			/* TVAL					                    */
#define L6474_THERMAL_WARNING_MASK					0x04			/* TVAL					                    */
#define L6474_UNDERVOLTAGE_MASK							0x08			/* TVAL					                    */
#define L6474_SWITCH_TURN_ON_EVENT_MASK			0x40			/* TVAL					                    */
#define L6474_WRONG_COMMAND_MASK						0x80			/* TVAL					                    */
#define L6474_ALARM_EN_LENGTH								1

/*L6474 L6474_CONFIG Bit mask definition ------------------------------------------------ */
#define L6474_TOFF_POS											10				/* TVAL					                    */
#define L6474_TOF_MASKF											0x7C00		/* TVAL					                    */
#define L6474_POW_SR_POS										8					/* TVAL					                    */
#define L6474_POW_SR_MASK										0x0300		/* TVAL					                    */
#define L6474_OC_SD_POS											7					/* TVAL					                    */
#define L6474_OC_SD_MASK										0x0080		/* TVAL					                    */
#define L6474_EN_TQREG_POS									5					/* TVAL					                    */
#define L6474_EN_TQREG_MASK									0x0020		/* TVAL					                    */
#define L6474_EXT_CLK_POS										3					/* TVAL					                    */
#define L6474_EXT_CLK_MASK									0x0008		/* TVAL					                    */
#define L6474_OSC_SEL_POS										0					/* TVAL					                    */
#define L6474_OSC_SEL_MASK									0x0007		/* TVAL					                    */
#define L6474_CONFIG_LENGTH									2

/*L6474 L6474_STATUS Bit mask definition ------------------------------------------------ */
#define L6474_OCD_MASK											0x1000		/* TVAL					                    */
#define L6474_TH_SD_MASK										0x0800		/* TVAL					                    */
#define L6474_TH_WRN_MASK										0x0400		/* TVAL					                    */
#define L6474_UVLO_MASK											0x0200		/* TVAL					                    */
#define L6474_WRONG_CMD_MASK								0x0100		/* TVAL					                    */
#define L6474_NOTPERF_CMD_MASK							0x0080		/* TVAL					                    */
#define L6474_DIR_MASK											0x0010		/* TVAL					                    */
#define L6474_HiZ_MASK											0x0001		/* TVAL					                    */
#define L6474_STATUS_LENGTH									2




struct l6474h_s{
  void (*SetParam)(enum l6474h_motor_s motor, uint8_t address, uint32_t data, uint8_t length);
  uint32_t (*GetParam)(enum l6474h_motor_s motor, uint8_t address, uint8_t length);
  uint32_t (*GetStatus)(enum l6474h_motor_s motor);
  void (*Enable)(enum l6474h_motor_s motor);
  void (*Disable)(enum l6474h_motor_s motor);
  void (*SetStepMode)(enum l6474h_motor_s motor, uint32_t step_mode);
  void (*SetCurrentLimit)(enum l6474h_motor_s motor, uint32_t current_limit);
  void (*SetCurrentLimit_mA)(enum l6474h_motor_s motor, float current_limit_mA);
  void (*SetDirection)(enum l6474h_motor_s motor, enum l6474h_motor_dir_s direction);
  void (*HardReset)(enum l6474h_motor_s motor);
};

extern const struct l6474h_s L6474H;

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
