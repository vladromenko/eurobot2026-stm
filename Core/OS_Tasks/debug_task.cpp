#include "os_tasks.hpp" 
#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.hpp"
#include "debug_terminal_cmd_handlers.hpp" 
#include "terminal.hpp" 





#define TASK_STACK_SIZE             512

static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;


static void Task(void *arg)
{
    struct debug_terminal_request_s debug_terminal_request;

    vTaskDelay(100);
    
	while(1)
	{
        while(Debug_Terminal.GetRequest(&debug_terminal_request))
        {
            DebugTerminalCmd.Handle(&debug_terminal_request);
        }

        taskYIELD();
	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "DebugTerminalTask", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


const struct os_task_base_s DebugTerminalTask = {   .CreateTask = CreateTask,
                                            .GetTaskHandle = GetTaskHandle};
