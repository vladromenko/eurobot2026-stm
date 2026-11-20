/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         impeller_task.c
  * @brief        Runs the impeller state machine (button press, boot pause,
  *               speed ramp) with a fixed period. Terminal handlers only post
  *               requests through Impeller.TurnOn()/TurnOff()/SetSpeed().
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "os_tasks.h"
#include "impeller.h"


#define TASK_STACK_SIZE             256
#define IMPELLER_TASK_PERIOD_MS     10


static TaskHandle_t task_handle;
static StackType_t stack_buffer[TASK_STACK_SIZE];
static StaticTask_t task_buffer;


static void Task(void *arg)
{
  (void)arg;

	while(1)
	{
    Impeller.Update(IMPELLER_TASK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(IMPELLER_TASK_PERIOD_MS));
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return task_handle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    task_handle = xTaskCreateStatic(Task, "ImpellerTask", TASK_STACK_SIZE,
                              NULL, uxPriority, stack_buffer, &task_buffer);
}


const struct os_task_base_s ImpellerTask = { .CreateTask = CreateTask,
                                          .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
