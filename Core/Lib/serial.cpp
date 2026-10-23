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
#include "serial.hpp"
#include "fifo_routine.hpp" 

#include "FreeRTOS.h"
#include "task.h"


namespace {

struct tx_fifo_element_s{
    uint8_t bytes[SERIAL_MAX_BYTES_NUMBER];
    uint8_t bytes_number;
};

struct rx_fifo_element_s{
    uint8_t bytes[SERIAL_MAX_BYTES_NUMBER];
    uint8_t bytes_number;
};

void TransmitNextQueueElement(USARTx_Line* line)
{
    line->tx_fifo.Remove(line->tx_buffer);
    line->StartTx(reinterpret_cast<tx_fifo_element_s*>(line->tx_buffer)->bytes_number);
}

void TransmitBytes(USARTx_Line* line, tx_fifo_element_s* tx_fifo_data_unit)
{
    while(1)
    {
        vTaskSuspendAll();

        if(line->tx_fifo.IsNotFull())
            break;

        xTaskResumeAll();
        taskYIELD();
    }
    
    line->tx_fifo.Add(tx_fifo_data_unit);

    if(line->is_line_busy==0){
        line->is_line_busy = 1;
        TransmitNextQueueElement(line);
    }

    xTaskResumeAll();
}

}

constinit SerialDriver Serial;


void SerialDriver::WriteBytes(USART_TypeDef* usart, uint8_t* bytes, uint8_t bytes_number)
{
    if(bytes_number>SERIAL_MAX_BYTES_NUMBER)
        return;
    
    tx_fifo_element_s tx_fifo_data_unit;

    memcpy(tx_fifo_data_unit.bytes, bytes, bytes_number);
    tx_fifo_data_unit.bytes_number = bytes_number;

    TransmitBytes(USARTx.GetLine(usart), &tx_fifo_data_unit);
}

uint32_t SerialDriver::ReadBytes(USART_TypeDef* usart, uint8_t* bytes, uint32_t timeout_ms)
{
    USARTx_Line* line = USARTx.GetLine(usart);

    if(timeout_ms){
        is_timeout_expired = 0;
        line->SetRxTimeout(timeout_ms*1000);
    }
    
    while(1)
    {
        if(line->rx_fifo.IsNotEmpty()){
            rx_fifo_element_s rx_fifo_data_unit;
            line->rx_fifo.Remove(&rx_fifo_data_unit);
            if(timeout_ms)
                line->ResetRxTimeout();
            memcpy(bytes, rx_fifo_data_unit.bytes, rx_fifo_data_unit.bytes_number);
            return rx_fifo_data_unit.bytes_number;
        }


        if((timeout_ms && is_timeout_expired)||(timeout_ms==0))
            return 0;

        taskYIELD();
    }
}

void SerialDriver::RxTimeoutCallback(USARTx_Line* line)
{
    (void)line;
    is_timeout_expired = 1;
}

void SerialDriver::TxCompleteCallback(USARTx_Line* line)
{
    line->StopTx();
    
    if(line->tx_fifo.IsNotEmpty())
        TransmitNextQueueElement(line);
    else
        line->is_line_busy = 0;
}

void SerialDriver::RxCompleteCallback(USARTx_Line* line, uint32_t data_length)
{
    line->StopRx();

    if(line->rx_fifo.IsNotFull()){
        reinterpret_cast<rx_fifo_element_s*>(line->rx_buffer)->bytes_number = data_length;
        line->rx_fifo.Add(line->rx_buffer);
    }
        
    line->StartRx();
}

void SerialDriver::InitLine(USARTx_Line* line)
{
    if((line->tx_buffer_size < 2*sizeof(tx_fifo_element_s))||
        (line->rx_buffer_size < 2*sizeof(rx_fifo_element_s)))
        return;

    uint8_t* tx_fifo = line->tx_buffer + sizeof(tx_fifo_element_s);
    uint8_t* rx_fifo = line->rx_buffer + sizeof(rx_fifo_element_s);

    uint32_t tx_fifo_size = line->tx_buffer_size/sizeof(tx_fifo_element_s) - 1;
    uint32_t rx_fifo_size = line->rx_buffer_size/sizeof(rx_fifo_element_s) - 1;

    line->tx_fifo.Init(tx_fifo, tx_fifo_size, sizeof(tx_fifo_element_s));
    line->rx_fifo.Init(rx_fifo, rx_fifo_size, sizeof(rx_fifo_element_s));

    line->is_line_busy = 0;
    line->is_line_half_duplex = 0;

    line->StartRx();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
