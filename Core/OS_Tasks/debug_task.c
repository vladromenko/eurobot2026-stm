#include "os_tasks.h" 
#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.h"
#include "terminal_cmd_handlers.h" 
#include "terminal.h" 





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
        while(Debug_Terminal.DebugGetRequest(&debug_terminal_request))
        {
            if((debug_terminal_cmd_handlers[debug_terminal_request.cmd]!=NULL)&&(debug_terminal_request.cmd<TERMINAL_CMD_TABLE_SIZE))
                debug_terminal_cmd_handlers[debug_terminal_request.cmd](debug_terminal_request.data,debug_terminal_request.data_length);
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
