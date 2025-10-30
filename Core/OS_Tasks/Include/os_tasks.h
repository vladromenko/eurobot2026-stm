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
#ifndef __OS_TASKS_H
#define __OS_TASKS_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "std_def.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"


struct os_task_base_s{
    void (*CreateTask)(UBaseType_t uxPriority);
    TaskHandle_t (*GetTaskHandle)(void);
};

extern const struct os_task_base_s ButtonsCallbackTask;
extern const struct os_task_base_s ButtonsPollTask;
extern const struct os_task_base_s DemoTask;
extern const struct os_task_base_s GameTask;
extern const struct os_task_base_s OdometryTask;
extern const struct os_task_base_s TerminalTask;
extern const struct os_task_base_s GripperTask;
extern const struct os_task_base_s StepperTask;

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
