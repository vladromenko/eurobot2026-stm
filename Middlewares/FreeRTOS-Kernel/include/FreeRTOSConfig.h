/* USER CODE BEGIN Header */
/*
 * FreeRTOS Kernel V10.0.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */
/* USER CODE END Header */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * These parameters and more are described within the 'configuration' section of the
 * FreeRTOS API documentation available on the FreeRTOS.org web site.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
/* Section where include file can be added */
/* USER CODE END Includes */

/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  extern uint32_t SystemCoreClock;
#endif

// #define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         0xf
// #define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY   5


/* AFTER CALLING taskENTER_CRITICAL(), INTERRUPTS WHICH
 * NVIC PRIORITIES >= (configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4)
 * WILL BE MASKED, WHILE OTHERS WITH PRIORITIES < (configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4)
 * WILL CONTINUE TO OPERATE. REGARDLESS THE NVIC SysTick TIMER PRIORITY,
 * AT THE taskENTER_CRITICAL() SECTION SCHEDULER WILL NOT SWITCH THE ONGOING TASK
 * TO ANOTHER TASK. NVIC SysTick TIMER PRIORITY = 0 DOES NOT WORK CORRECTLY
 * FreeRTOS AUTOMATICLY SET THE NVIC SysTick TIMER PRIORITY TO THE LOWEST
 * VALUE 0xF DURING THE vTaskStartScheduler() CALLING. NVIC SysTick TIMER PRIORITY
 * DOES NOT AFFECT PROGRAM OPERATE AND DEFINE ONLY WHETHER SOME ISR CAN PREEMPT SCHEDULER
 * OR VICE VERSA. WE HAVE TO MAKE A SHIFT (PRIORITY<<4) BECAUSE CORTEX-M3/M4 USES ONLY 4
 * BITS OUT OF 8 FOR PRIORITY LEVEL, SO THERE ARE 16 PRIORITIES LEVEL IN TOTAL, AND THEY
 * ARE ARRANGED AT THE MOST SIGNIFICANT BITS SIDE FROM 7:4 BITS, IT IS A STANDART
 * PRIOTIY BITS DISTRIBUTION FOR ARM, NOT STM32 ONLY. __NVIC_PRIO_BITS DEFINE THE
 * NUMBER OF BITS USED FOR PRIORITY LEVEL. FOR CORTEX M4 __NVIC_PRIO_BITS = 4.
 * SINCE WE ARE USING SysTick TIMER FOR THE SCHEDULER CALLING ONE MAY THING
 * WE CAN JUST DUSABLE SysTick_IRQn INTERRUPT TO PREVENT THE CONTEXT SWITCH BY THE SCHEDULER,
 * HOWEVER THE NUMBER OF SysTick_IRQn = -1 IS NEGATIVE, AND NEAR THE __NVIC_DisableIRQ(IRQn_Type IRQn)
 * IT IS WRITTEN "IRQn must not be negative", SO WE CANNOT MASK A SysTick INTERRUPT BY NVIC.
 * wE CAN SWITCH OFF IT ONLY BY WRITING SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk;
 * BUT IT WILL LEAD TO MISS A ONE SysTick PERIOD AND VIOLATION IN THE FreeRTOS SCHEDULER
 * BEHAVIOUR RESULTING IN THE LONG-PERIOD TIME STOP OF THE CONTEXT SWITCH, MORE THAN JUST
 * A ONE SysTickPERIOD. AT THE REST, USING THE NVIC_SetPriorityGrouping(0); INSTEAD OF 
 * THE NVIC_SetPriorityGrouping(4); AS IT IS SUGGESTED IN THE CORTEX-M3-M4 FreeRTOS
 * DOCS SECTION, LOOKS MORE REASONABLE
*/
// #define configKERNEL_INTERRUPT_PRIORITY			(0xF<<4)
// #define configMAX_SYSCALL_INTERRUPT_PRIORITY	        (11<<4)

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
        /* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
        #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
        #define configPRIO_BITS         4        /* 15 priority levels */
#endif

#define configKERNEL_INTERRUPT_PRIORITY			(0xF << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY	        (11 << (8 - configPRIO_BITS))

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
// #define configKERNEL_INTERRUPT_PRIORITY                 \
//         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
// #define configMAX_SYSCALL_INTERRUPT_PRIORITY            \
//         (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
		

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
#define configASSERT(x) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; ); }


#define configUSE_PREEMPTION                     1
#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         0
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )
#define configTICK_RATE_HZ                       ((TickType_t)1000)
#define configMAX_PRIORITIES                     ( 7 )
#define configMINIMAL_STACK_SIZE                 ((uint16_t)16)
#define configMAX_TASK_NAME_LEN                  ( 16 )
#define configUSE_16_BIT_TICKS                   0
#define configUSE_MUTEXES                        1
#define configQUEUE_REGISTRY_SIZE                8

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                1
// #define INCLUDE_vTaskDelayUntil             0
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_xTaskGetSchedulerState      1
#define INCLUDE_xTaskAbortDelay		        1

#define INCLUDE_eTaskGetState 			1
#define INCLUDE_xTaskDelayUntil                 1

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
/* USER CODE BEGIN 1 */
//#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
/* USER CODE END 1 */

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

/* IMPORTANT: This define is commented when used with STM32Cube firmware, when the timebase source is SysTick,
              to prevent overwriting SysTick_Handler defined within STM32Cube HAL */

#define xPortSysTickHandler SysTick_Handler

/* USER CODE BEGIN Defines */
/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */
/* USER CODE END Defines */

#endif /* FREERTOS_CONFIG_H */
