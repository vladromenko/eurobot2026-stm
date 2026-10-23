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
#include "dynamixel_master.hpp"
#include "fifo_routine.hpp" 

#include "FreeRTOS.h"
#include "task.h"


namespace {

constexpr uint8_t MAX_RETRANSMIT_NUMBER = 2;
constexpr uint32_t TIMEOUT_US = 2000;


struct packet_s{
    uint8_t header1;
    uint8_t header2;
    uint8_t packet_id;
    uint8_t length;
    uint8_t instruction_error;
    uint8_t param_crc[MAX_DATA_LENGTH+1];
};

/*  The first element of the tx buffer is the packet being sent, the rest is the FIFO  */
struct tx_fifo_element_s{
    packet_s packet;
    dynamixel_master_response_s* response;
    uint8_t responses_number;
    uint8_t retransmit_downcounter;
};

struct rx_fifo_element_s{
    packet_s packet;
    uint8_t responses_downcounter;
    uint8_t cumulative_error;
};

inline tx_fifo_element_s* TxData(USARTx_Line* line)
{
    return reinterpret_cast<tx_fifo_element_s*>(line->tx_buffer);
}

inline rx_fifo_element_s* RxData(USARTx_Line* line)
{
    return reinterpret_cast<rx_fifo_element_s*>(line->rx_buffer);
}

uint8_t Calc_CRC(const packet_s* packet)
{
    uint16_t crc = packet->packet_id + packet->length + packet->instruction_error;
    
    for(uint8_t ik=0; ik<(packet->length-2); ik++)
        crc += packet->param_crc[ik];
    
    crc = ~crc;
    crc &= 0xFF;

    return (uint8_t)crc;
}

uint8_t VerifyPacket(const packet_s* packet, uint8_t packet_length)
{
    if((packet->header1 != 0xFF)||(packet->header2 != 0xFF))
        return 1;

    if(packet_length<6)
        return 1;

    if(packet_length != (packet->length + 4))
        return 1;

    uint8_t packet_crc = packet->param_crc[packet->length - 2];
    uint8_t calc_crc = Calc_CRC(packet);

    if(packet_crc != calc_crc)
        return 1;
    
    return 0;
}

void Retransmit(USARTx_Line* line)
{
    TxData(line)->retransmit_downcounter--;
    line->StartTx(TxData(line)->packet.length + 4);
}

void TransmitNext(USARTx_Line* line)
{
    line->tx_fifo.Remove(line->tx_buffer);
    TxData(line)->retransmit_downcounter = MAX_RETRANSMIT_NUMBER;
    line->StartTx(TxData(line)->packet.length + 4);
}

void TransmitCmd(USARTx_Line* line, tx_fifo_element_s* tx_fifo_data)
{
    while(1)
    {
        vTaskSuspendAll();

        if(line->tx_fifo.IsNotFull())
            break;

        xTaskResumeAll();
        taskYIELD();
    }
    
    line->tx_fifo.Add(tx_fifo_data);

    if(line->is_line_busy==0){
        line->is_line_busy = 1;
        TransmitNext(line);
    }

    xTaskResumeAll();
}

void Check_TX_FIFO(USARTx_Line* line)
{
    if(line->tx_fifo.IsNotEmpty())
        TransmitNext(line);
    else
        line->is_line_busy = 0;
}

/*  Fill the packet header, append the CRC and queue the request  */
void SendPacket(USART_TypeDef* usart, tx_fifo_element_s* tx_fifo_data)
{
    tx_fifo_data->packet.header1 = 0xFF;
    tx_fifo_data->packet.header2 = 0xFF;
    tx_fifo_data->packet.param_crc[tx_fifo_data->packet.length-2] = Calc_CRC(&(tx_fifo_data->packet));

    TransmitCmd(USARTx.GetLine(usart), tx_fifo_data);
}

}

constinit DynamixelMasterDriver DynamixelMaster;


void DynamixelMasterDriver::Ping(USART_TypeDef* usart, uint8_t device_id, dynamixel_master_response_s* response)
{
    tx_fifo_element_s tx_fifo_data;
    
    tx_fifo_data.packet.packet_id = device_id;
    tx_fifo_data.packet.length = 2;
    tx_fifo_data.packet.instruction_error = Ping_CMD;

    tx_fifo_data.response = response;
    tx_fifo_data.responses_number = 1;

    response->is_data_ready = 0;

    SendPacket(usart, &tx_fifo_data);
}

void DynamixelMasterDriver::Read(USART_TypeDef* usart, uint8_t device_id, uint8_t starting_address, uint8_t data_length, dynamixel_master_response_s* response)
{
    tx_fifo_element_s tx_fifo_data;
    
    tx_fifo_data.packet.packet_id = device_id;
    tx_fifo_data.packet.length = 4;
    tx_fifo_data.packet.instruction_error = Read_CMD;
    tx_fifo_data.packet.param_crc[0] = starting_address;
    tx_fifo_data.packet.param_crc[1] = data_length;

    tx_fifo_data.response = response;
    tx_fifo_data.responses_number = 1;

    response->is_data_ready = 0;

    SendPacket(usart, &tx_fifo_data);
}

/*  "response" may be nullptr when the answer is not needed  */
void DynamixelMasterDriver::Write(USART_TypeDef* usart, uint8_t device_id, uint8_t starting_address, uint8_t* data, uint8_t data_length, dynamixel_master_response_s* response)
{
    tx_fifo_element_s tx_fifo_data;
    
    tx_fifo_data.packet.packet_id = device_id;
    tx_fifo_data.packet.length = data_length+3;
    tx_fifo_data.packet.instruction_error = Write_CMD;
    tx_fifo_data.packet.param_crc[0] = starting_address;
    memcpy(&(tx_fifo_data.packet.param_crc[1]), data, data_length);

    tx_fifo_data.response = response;
    tx_fifo_data.responses_number = 1;

    if(response != nullptr)
        response->is_data_ready = 0;

    SendPacket(usart, &tx_fifo_data);
}

/*  Devices do not answer a broadcast; "response_array" may be nullptr  */
void DynamixelMasterDriver::SyncWrite(USART_TypeDef* usart, uint8_t* device_id_array, uint8_t boards_number, uint8_t starting_address, uint8_t* data_array, uint8_t data_length, dynamixel_master_response_s* response_array)
{
    tx_fifo_element_s tx_fifo_data;
    
    tx_fifo_data.packet.packet_id = BROADCAST_ID;
    tx_fifo_data.packet.length = 4 + boards_number*(data_length+1);
    tx_fifo_data.packet.instruction_error = SyncWrite_CMD;
    tx_fifo_data.packet.param_crc[0] = starting_address;
    tx_fifo_data.packet.param_crc[1] = data_length;

    uint8_t data_length_p1 = data_length + 1;

    for(uint8_t ik=0; ik<boards_number; ik++)
    {
        tx_fifo_data.packet.param_crc[2+data_length_p1*ik] = device_id_array[ik];
        memcpy(&tx_fifo_data.packet.param_crc[3+data_length_p1*ik], &data_array[data_length*ik], data_length);
        if(response_array != nullptr)
            response_array[ik].is_data_ready = 0;
    }

    tx_fifo_data.response = response_array;
    tx_fifo_data.responses_number = 0;

    SendPacket(usart, &tx_fifo_data);
}


void DynamixelMasterDriver::RxTimeoutCallback(USARTx_Line* line)
{
    line->StopRx();

    tx_fifo_element_s* tx_fifo_data = TxData(line);

    if(tx_fifo_data->retransmit_downcounter){
        Retransmit(line);
        return;
    }

    if(tx_fifo_data->response != nullptr){
        tx_fifo_data->response->bus_error = 1;
        tx_fifo_data->response->is_data_ready = 1;
    }

    Check_TX_FIFO(line);
}

void DynamixelMasterDriver::TxCompleteCallback(USARTx_Line* line)
{
    line->StopTx();

    if(TxData(line)->responses_number){
        RxData(line)->responses_downcounter = TxData(line)->responses_number;
        RxData(line)->cumulative_error = 0;
        memset(&(RxData(line)->packet), 0x55, sizeof(packet_s));
        line->StartRx();
        line->SetRxTimeout(TIMEOUT_US);
        return;
    }

    Check_TX_FIFO(line);
}

void DynamixelMasterDriver::RxCompleteCallback(USARTx_Line* line, uint32_t data_length)
{
    rx_fifo_element_s* rx_fifo_data = RxData(line);
    tx_fifo_element_s* tx_fifo_data = TxData(line);
    packet_s* packet = &rx_fifo_data->packet;
    uint8_t packet_length = data_length;

    if(VerifyPacket(packet, packet_length))
        return;

    line->ResetRxTimeout();
    line->StopRx();

    rx_fifo_data->responses_downcounter--;

    rx_fifo_data->cumulative_error |= (packet->instruction_error)&(INSTRUCTION_ERROR|CHECKSUM_ERROR|RANGE_ERROR);

    if(rx_fifo_data->cumulative_error)
    {     
        if(tx_fifo_data->retransmit_downcounter)
        {
            if(rx_fifo_data->responses_downcounter)
                return;
            Retransmit(line);
            return;
        }
    }
        
    if(tx_fifo_data->response != nullptr){
        tx_fifo_data->response->bus_error = 0;
        tx_fifo_data->response->device_error = packet->instruction_error;
        tx_fifo_data->response->data_length = packet->length-2;
        memcpy(tx_fifo_data->response->data, packet->param_crc, packet->length-2);
    }
    

    if(rx_fifo_data->responses_downcounter){
        if(tx_fifo_data->response != nullptr)
            tx_fifo_data->response++;
        line->StartRx();
        return;
    }

    if(tx_fifo_data->response != nullptr)
        tx_fifo_data->response->is_data_ready = 1;

    Check_TX_FIFO(line);
}

void DynamixelMasterDriver::InitLine(USARTx_Line* line)
{
    if((line->tx_buffer_size < 2*sizeof(tx_fifo_element_s))||
        (line->rx_buffer_size < 2*sizeof(rx_fifo_element_s)))
        return;

    uint8_t* tx_fifo = line->tx_buffer + sizeof(tx_fifo_element_s);
    uint32_t tx_fifo_size = line->tx_buffer_size/sizeof(tx_fifo_element_s) - 1;

    line->tx_fifo.Init(tx_fifo, tx_fifo_size, sizeof(tx_fifo_element_s));

    line->is_line_busy = 0;
    line->is_line_half_duplex = 1;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
