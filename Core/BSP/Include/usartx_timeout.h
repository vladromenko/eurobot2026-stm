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
#ifndef __USARTX_TIMEOUT_H
#define __USARTX_TIMEOUT_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
void SetTimeout(uint8_t timeout_channel, uint32_t timeout_us, void (*TimeoutCallback)(void* callback_params), void* callback_params);
void ResetTimeout(uint8_t timeout_channel);
void USARTx_Timeout_IRQ_Process(void);


#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
