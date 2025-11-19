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
#include "serial.h"
#include "fifo_routine.h" 

#include "board_gpio_defines.h"
#include "stm32f4xx_ll_gpio.h"


#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "usartx_timeout.h" 


struct tx_fifo_element_s{
    uint8_t bytes[SERIAL_MAX_BYTES_NUMBER];
    uint8_t bytes_number;
};

struct rx_fifo_element_s{
    uint8_t bytes[SERIAL_MAX_BYTES_NUMBER];
    uint8_t bytes_number;
};

static struct usartx_control_s* usartx_control;
static uint8_t is_timeout_expired;


static void TransmitBytes(struct usartx_line_resources_s* line_resources, struct tx_fifo_element_s* tx_fifo_data_unit);


static void WriteBytes(USART_TypeDef* usart, uint8_t* bytes, uint8_t bytes_number)
{
    if(bytes_number>SERIAL_MAX_BYTES_NUMBER)
        return;
    
    struct usartx_line_resources_s* line_resources = usartx_control->GetLineResources(usart);

    struct tx_fifo_element_s tx_fifo_data_unit;

    memcpy(tx_fifo_data_unit.bytes, bytes, bytes_number);
    tx_fifo_data_unit.bytes_number = bytes_number;

    TransmitBytes(line_resources, &tx_fifo_data_unit);
}

static uint32_t ReadBytes(USART_TypeDef* usart, uint8_t* bytes, uint32_t timeout_ms)
{
    struct usartx_line_resources_s* line_resources = usartx_control->GetLineResources(usart);

    if(timeout_ms){
        is_timeout_expired = 0;
        usartx_control->SetRxTimeout(line_resources, timeout_ms*1000);
    }
    
    while(1)
    {
        if(FIFO_IsNotEmpty(&(line_resources->rx_fifo))){
            struct rx_fifo_element_s rx_fifo_data_unit;
            FIFO_Remove(&(line_resources->rx_fifo), &rx_fifo_data_unit);
            if(timeout_ms)
                usartx_control->ResetRxTimeout(line_resources);
            memcpy(bytes, rx_fifo_data_unit.bytes, rx_fifo_data_unit.bytes_number);
            return rx_fifo_data_unit.bytes_number;
        }


        if((timeout_ms && is_timeout_expired)||(timeout_ms==0))
            return 0;

        taskYIELD();
    }
}

static void TransmitNextQueueElement(struct usartx_line_resources_s* line_resources)
{
    FIFO_Remove(&(line_resources->tx_fifo), line_resources->tx_buffer);
    usartx_control->StartTx(line_resources, ((struct tx_fifo_element_s*)(line_resources->tx_buffer))->bytes_number);
}

static void TransmitBytes(struct usartx_line_resources_s* line_resources, struct tx_fifo_element_s* tx_fifo_data_unit)
{
    while(1)
    {
        vTaskSuspendAll();

        if(FIFO_IsNotFull(&(line_resources->tx_fifo)))
            break;

        xTaskResumeAll();
        taskYIELD();
    }
    
    FIFO_Add(&(line_resources->tx_fifo), tx_fifo_data_unit);

    if(line_resources->is_line_busy==0){
        line_resources->is_line_busy = 1;
        TransmitNextQueueElement(line_resources);
    }

    xTaskResumeAll();
}

static void RxTimeoutCallback(struct usartx_line_resources_s* line_resources)
{
    is_timeout_expired = 1;
}

static void TxCompleteCallback(struct usartx_line_resources_s* line_resources)
{
   usartx_control->StopTx(line_resources);
    
    if(FIFO_IsNotEmpty(&(line_resources->tx_fifo)))
        TransmitNextQueueElement(line_resources);
    else
        line_resources->is_line_busy = 0;

    
}

static void RxCompleteCallback(struct usartx_line_resources_s* line_resources, uint32_t data_length)
{
    usartx_control->StopRx(line_resources);

    if(FIFO_IsNotFull(&(line_resources->rx_fifo))){
        ((struct rx_fifo_element_s*)(line_resources->rx_buffer))->bytes_number = data_length;
        FIFO_Add(&(line_resources->rx_fifo), line_resources->rx_buffer);
    }
	    
    usartx_control->StartRx(line_resources);
}



static void InitLine(struct usartx_line_init_params_s* usartx_line_init_params)
{
    struct usartx_line_resources_s* line_resources = usartx_line_init_params->line_resources;
    struct usartx_line_config_s* line_config = usartx_line_init_params->line_config;
    usartx_control = usartx_line_init_params->usartx_control;

    if((line_resources->tx_buffer_size < 2*sizeof(struct tx_fifo_element_s))||
        (line_resources->rx_buffer_size < 2*sizeof(struct rx_fifo_element_s)))
        return;

    uint8_t* tx_fifo = (uint8_t*)((uint32_t)line_resources->tx_buffer + sizeof(struct tx_fifo_element_s));
    uint8_t* rx_fifo = (uint8_t*)((uint32_t)line_resources->rx_buffer + sizeof(struct rx_fifo_element_s));

    uint32_t tx_fifo_size = line_resources->tx_buffer_size/sizeof(struct tx_fifo_element_s) - 1;
    uint32_t rx_fifo_size = line_resources->rx_buffer_size/sizeof(struct rx_fifo_element_s) - 1;

    FIFO_StructInit(&line_resources->tx_fifo, tx_fifo, tx_fifo_size, sizeof(struct tx_fifo_element_s));
    FIFO_StructInit(&line_resources->rx_fifo, rx_fifo, rx_fifo_size, sizeof(struct rx_fifo_element_s));

    line_resources->is_line_busy = 0;
    
    line_config->TxCompleteCallback = TxCompleteCallback;
    line_config->RxCompleteCallback = RxCompleteCallback;
    line_config->RxTimeoutCallback = RxTimeoutCallback;
    line_config->is_line_half_duplex = 0;

    usartx_control->StartRx(line_resources);
}

static struct usartx_protocol_s protocol = {.InitLine = InitLine};

static struct usartx_protocol_s* GetProtocol(void)
{
    return &protocol;
}

const struct serial_s Serial = {.WriteBytes = WriteBytes,
                                .ReadBytes = ReadBytes,
                                .GetProtocol = GetProtocol};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
