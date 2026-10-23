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
#include "hl_rtu.hpp"
#include "fifo_routine.hpp" 

#include "FreeRTOS.h"
#include "task.h"


namespace {

struct pdu_s{
    uint8_t pdu_length;
    uint8_t cmd;
    uint8_t data_crc[HL_RTU_MAX_DATA_LENGTH+1];
};

struct packet_s{
    uint8_t header1;
    uint8_t header2;
    pdu_s pdu;
};

/*  FIFOs hold bare PDUs, headers are added on transmit  */
using tx_fifo_element_s = pdu_s;
using rx_fifo_element_s = pdu_s;

uint8_t Calc_CRC(const pdu_s* pdu)
{
    uint16_t crc = pdu->pdu_length + pdu->cmd;
    
    for(uint8_t ik=0; ik<(pdu->pdu_length-2); ik++)
        crc += pdu->data_crc[ik];
    
    crc = ~crc;
    crc &= 0xFF;

    return (uint8_t)crc;
}

uint8_t VerifyPacket(const packet_s* packet, uint8_t pdu_length)
{
    if((packet->header1!=0xFF)||(packet->header2!=0xFF))
        return 1;

    if(pdu_length<5)
        return 1;

    if(pdu_length!=(packet->pdu.pdu_length+3))
        return 1;

    uint8_t packet_crc = packet->pdu.data_crc[packet->pdu.pdu_length - 2];
    uint8_t calc_crc = Calc_CRC(&(packet->pdu));

    if(packet_crc!=calc_crc)
        return 1;
    
    return 0;
}

void TransmitNext(USARTx_Line* line)
{
    packet_s* packet = reinterpret_cast<packet_s*>(line->tx_buffer);

    packet->header1 = 0xFF;
    packet->header2 = 0xFF;
    line->tx_fifo.Remove(&packet->pdu);

    line->StartTx(packet->pdu.pdu_length+3);
}

void TransmitCmd(USARTx_Line* line, tx_fifo_element_s* tx_fifo_data_unit)
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
        TransmitNext(line);
    }

    xTaskResumeAll();
}

}

constinit HL_RTU_Driver HL_RTU;


void HL_RTU_Driver::SendResponse(USART_TypeDef* usart, hl_rtu_response_s* response)
{
    tx_fifo_element_s tx_fifo_data_unit;

    tx_fifo_data_unit.cmd = response->cmd;
    tx_fifo_data_unit.pdu_length = response->data_length+2;
    memcpy(tx_fifo_data_unit.data_crc, response->data, response->data_length);
    tx_fifo_data_unit.data_crc[response->data_length] = Calc_CRC(&tx_fifo_data_unit);

    TransmitCmd(USARTx.GetLine(usart), &tx_fifo_data_unit);
}

uint8_t HL_RTU_Driver::GetRequest(USART_TypeDef* usart, hl_rtu_request_s* request)
{
    USARTx_Line* line = USARTx.GetLine(usart);

    if(line->rx_fifo.IsNotEmpty()){
        rx_fifo_element_s rx_fifo_data_unit;
        line->rx_fifo.Remove(&rx_fifo_data_unit);
        request->cmd = rx_fifo_data_unit.cmd;
        request->data_length = rx_fifo_data_unit.pdu_length-2;
        memcpy(request->data, rx_fifo_data_unit.data_crc, rx_fifo_data_unit.pdu_length-2);
        return 1;
    }

    return 0;
}

void HL_RTU_Driver::TxCompleteCallback(USARTx_Line* line)
{
    line->StopTx();
    
    if(line->tx_fifo.IsNotEmpty())
        TransmitNext(line);
    else
        line->is_line_busy = 0;
}

void HL_RTU_Driver::RxCompleteCallback(USARTx_Line* line, uint32_t data_length)
{
    line->StopRx();

    packet_s* packet = reinterpret_cast<packet_s*>(line->rx_buffer);

    if(VerifyPacket(packet,data_length)==0)
    {
        if(line->rx_fifo.IsNotFull())
            line->rx_fifo.Add(&packet->pdu);
    }
    
    memset(line->rx_buffer, 0x55, sizeof(packet_s));

    line->StartRx();
}

void HL_RTU_Driver::InitLine(USARTx_Line* line)
{
    if((line->tx_buffer_size < (sizeof(tx_fifo_element_s)+sizeof(packet_s)))||
        (line->rx_buffer_size < (sizeof(rx_fifo_element_s)+sizeof(packet_s))))
        return;

    uint8_t* tx_fifo = line->tx_buffer + sizeof(packet_s);
    uint8_t* rx_fifo = line->rx_buffer + sizeof(packet_s);

    uint32_t tx_fifo_size = (line->tx_buffer_size - sizeof(packet_s))/sizeof(tx_fifo_element_s);
    uint32_t rx_fifo_size = (line->rx_buffer_size - sizeof(packet_s))/sizeof(rx_fifo_element_s);

    line->tx_fifo.Init(tx_fifo, tx_fifo_size, sizeof(tx_fifo_element_s));
    line->rx_fifo.Init(rx_fifo, rx_fifo_size, sizeof(rx_fifo_element_s));

    line->is_line_busy = 0;
    line->is_line_half_duplex = 0;

    memset(line->rx_buffer, 0x55, sizeof(packet_s));
    line->StartRx();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
