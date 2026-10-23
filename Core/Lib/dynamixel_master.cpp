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

#include "board_gpio_defines.hpp"
#include "stm32f4xx_ll_gpio.h"


#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "usartx_timeout.hpp" 

#define MAX_RETRANSMIT_NUMBER	 2
#define TIMEOUT_US               2000


struct packet_s{
    uint8_t header1;
	uint8_t header2;
    uint8_t packet_id;
    uint8_t length;
	uint8_t instruction_error;
	uint8_t param_crc[MAX_DATA_LENGTH+1];
};

struct tx_fifo_element_s{
    struct packet_s packet;
    struct dynamixel_master_response_s* response;
    uint8_t responses_number;
    uint8_t retransmit_downcounter;
};

struct rx_fifo_element_s{
    struct packet_s packet;
    uint8_t responses_downcounter;
    uint8_t cumulative_error;
};

static struct usartx_control_s* usartx_control;

static void Retransmit(struct usartx_line_resources_s* line_resources);
static void TransmitNext(struct usartx_line_resources_s* line_resources);
static void TransmitCmd(struct usartx_line_resources_s* line_resources, struct tx_fifo_element_s* tx_fifo_data);
static uint8_t Calc_CRC(struct packet_s* packet);


static void PingCmd(USART_TypeDef* usart, uint8_t device_id, struct dynamixel_master_response_s* response)
{
    struct usartx_line_resources_s* line_resources = usartx_control->GetLineResources(usart);

    struct tx_fifo_element_s tx_fifo_data;
    
    tx_fifo_data.packet.header1 = 0xFF;
    tx_fifo_data.packet.header2 = 0xFF;
    tx_fifo_data.packet.packet_id = device_id;
    tx_fifo_data.packet.length = 2;
    tx_fifo_data.packet.instruction_error = Ping_CMD;
    tx_fifo_data.packet.param_crc[0] = Calc_CRC(&(tx_fifo_data.packet));

    tx_fifo_data.response = response;
    tx_fifo_data.responses_number = 1;

    response->is_data_ready = 0;

    TransmitCmd(line_resources, &tx_fifo_data);
}

static void ReadCmd(USART_TypeDef* usart, uint8_t device_id, uint8_t starting_address, uint8_t data_length, struct dynamixel_master_response_s* response)
{
    struct usartx_line_resources_s* line_resources = usartx_control->GetLineResources(usart);

    struct tx_fifo_element_s tx_fifo_data;
    
    tx_fifo_data.packet.header1 = 0xFF;
    tx_fifo_data.packet.header2 = 0xFF;
    tx_fifo_data.packet.packet_id = device_id;
    tx_fifo_data.packet.length = 4;
    tx_fifo_data.packet.instruction_error = Read_CMD;
    tx_fifo_data.packet.param_crc[0] = starting_address;
    tx_fifo_data.packet.param_crc[1] = data_length;
    tx_fifo_data.packet.param_crc[2] = Calc_CRC(&(tx_fifo_data.packet));

    tx_fifo_data.response = response;
    tx_fifo_data.responses_number = 1;

    response->is_data_ready = 0;

    TransmitCmd(line_resources, &tx_fifo_data);
}

static void WriteCmd(USART_TypeDef* usart, uint8_t device_id, uint8_t starting_address, uint8_t* data, uint8_t data_length, struct dynamixel_master_response_s* response)
{
    struct usartx_line_resources_s* line_resources = usartx_control->GetLineResources(usart);

    struct tx_fifo_element_s tx_fifo_data;
    
    tx_fifo_data.packet.header1 = 0xFF;
    tx_fifo_data.packet.header2 = 0xFF;
    tx_fifo_data.packet.packet_id = device_id;
    tx_fifo_data.packet.length = data_length+3;
    tx_fifo_data.packet.instruction_error = Write_CMD;
    tx_fifo_data.packet.param_crc[0] = starting_address;
    memcpy(&(tx_fifo_data.packet.param_crc[1]), data, data_length);
    tx_fifo_data.packet.param_crc[data_length+1] = Calc_CRC(&(tx_fifo_data.packet));

    tx_fifo_data.response = response;
    tx_fifo_data.responses_number = 1;

    response->is_data_ready = 0;

    TransmitCmd(line_resources, &tx_fifo_data);
}

static void RegWriteCmd(USART_TypeDef* usart, uint8_t device_id, uint8_t starting_address, uint8_t* data, uint8_t data_length, uint8_t* response)
{

}
static void ActionCmd(USART_TypeDef* usart, uint8_t device_id, uint8_t* response)
{

}
static void FactoryResetCmd(USART_TypeDef* usart, uint8_t device_id, uint8_t* response)
{

}
static void RebootCmd(USART_TypeDef* usart, uint8_t device_id, uint8_t* response)
{

}

static void SyncWriteCmd(USART_TypeDef* usart, uint8_t* device_id_array, uint8_t boards_number, uint8_t starting_address, uint8_t* data_array, uint8_t data_length, struct dynamixel_master_response_s* response_array)
{
    struct usartx_line_resources_s* line_resources = usartx_control->GetLineResources(usart);

    struct tx_fifo_element_s tx_fifo_data;
    
    tx_fifo_data.packet.header1 = 0xFF;
    tx_fifo_data.packet.header2 = 0xFF;
    tx_fifo_data.packet.packet_id = BROADCAST_ID;
    tx_fifo_data.packet.length = 4 + boards_number*(data_length+1);
    tx_fifo_data.packet.instruction_error = SyncWrite_CMD;
    tx_fifo_data.packet.param_crc[0] = starting_address;
    tx_fifo_data.packet.param_crc[1] = data_length;

    uint8_t data_length_p1 = data_length + 1;

    do{
        uint8_t devices_number_m1 = boards_number - 1;
        tx_fifo_data.packet.param_crc[2+data_length_p1*devices_number_m1] = device_id_array[devices_number_m1];
        memcpy(&tx_fifo_data.packet.param_crc[3+data_length_p1*devices_number_m1], &data_array[data_length*devices_number_m1], data_length);
        response_array[devices_number_m1].is_data_ready = 0;
    }while(boards_number--);
    
    tx_fifo_data.packet.param_crc[tx_fifo_data.packet.length-2] = Calc_CRC(&(tx_fifo_data.packet));

    tx_fifo_data.response = response_array;
    tx_fifo_data.responses_number = 0;

    TransmitCmd(line_resources, &tx_fifo_data);
}

static void BulkReadCmd(USART_TypeDef* usart, uint8_t* device_id_array, uint8_t* starting_address_array, uint8_t* data_length, struct dynamixel_master_response_s* response_array)
{

}

static uint8_t Calc_CRC(struct packet_s* packet)
{
    uint16_t crc = packet->packet_id + packet->length + packet->instruction_error;
	
	for(uint8_t ik=0; ik<(packet->length-2); ik++)
		crc += packet->param_crc[ik];
	
	crc = ~crc;
	crc &= 0xFF;

    return (uint8_t)crc;
}

static uint8_t VerifyPacket(struct packet_s* packet, uint8_t packet_length)
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

static void Retransmit(struct usartx_line_resources_s* line_resources)
{
    ((struct tx_fifo_element_s*)(line_resources->tx_buffer))->retransmit_downcounter--;
    usartx_control->StartTx(line_resources, ((struct tx_fifo_element_s*)(line_resources->tx_buffer))->packet.length + 4);
}

static void TransmitNext(struct usartx_line_resources_s* line_resources)
{
    FIFO_Remove(&line_resources->tx_fifo, line_resources->tx_buffer);
    ((struct tx_fifo_element_s*)(line_resources->tx_buffer))->retransmit_downcounter = MAX_RETRANSMIT_NUMBER;
    usartx_control->StartTx(line_resources, ((struct tx_fifo_element_s*)(line_resources->tx_buffer))->packet.length + 4);
}

static void TransmitCmd(struct usartx_line_resources_s* line_resources, struct tx_fifo_element_s* tx_fifo_data)
{
    while(1)
    {
        vTaskSuspendAll();

        if(FIFO_IsNotFull(&line_resources->tx_fifo))
            break;

        xTaskResumeAll();
        taskYIELD();
    }
    
    FIFO_Add(&line_resources->tx_fifo,tx_fifo_data);

    if(line_resources->is_line_busy==0){
        line_resources->is_line_busy = 1;
        TransmitNext(line_resources);
    }

    xTaskResumeAll();
}

static void Check_TX_FIFO(struct usartx_line_resources_s* line_resources)
{
    if(FIFO_IsNotEmpty(&line_resources->tx_fifo))
        TransmitNext(line_resources);
    else
        line_resources->is_line_busy = 0;
}


static void RxTimeoutCallback(struct usartx_line_resources_s* line_resources)
{
    usartx_control->StopRx(line_resources);

    struct tx_fifo_element_s* tx_fifo_data = (struct tx_fifo_element_s*)(line_resources->tx_buffer);

    if(tx_fifo_data->retransmit_downcounter){
        Retransmit(line_resources);
        return;
    }

    if(tx_fifo_data->response != NULL){
        do{
            tx_fifo_data->response->bus_error = 1;
            tx_fifo_data->response->is_data_ready = 1;
        }while(tx_fifo_data->responses_number--);
     }

    Check_TX_FIFO(line_resources);
}

static void TxCompleteCallback(struct usartx_line_resources_s* line_resources)
{
    usartx_control->StopTx(line_resources);

    if(((struct tx_fifo_element_s*)(line_resources->tx_buffer))->responses_number){
        ((struct rx_fifo_element_s*)(line_resources->rx_buffer))->responses_downcounter = ((struct tx_fifo_element_s*)(line_resources->tx_buffer))->responses_number;
        ((struct rx_fifo_element_s*)(line_resources->rx_buffer))->cumulative_error = 0;
        memset(&(((struct rx_fifo_element_s*)(line_resources->rx_buffer))->packet), 0x55, sizeof(struct packet_s));
        usartx_control->StartRx(line_resources);
        usartx_control->SetRxTimeout(line_resources, TIMEOUT_US);
        return;
    }

    Check_TX_FIFO(line_resources);
}

static void RxCompleteCallback(struct usartx_line_resources_s* line_resources, uint32_t data_length)
{
    struct packet_s* packet = (struct packet_s*)&((struct rx_fifo_element_s*)(line_resources->rx_buffer))->packet;
    uint8_t packet_length = data_length;

    if(VerifyPacket(packet, packet_length))
        return;

    usartx_control->ResetRxTimeout(line_resources);
    usartx_control->StopRx(line_resources);

    struct rx_fifo_element_s* rx_fifo_data = (struct rx_fifo_element_s*)(line_resources->rx_buffer);
    struct tx_fifo_element_s* tx_fifo_data = (struct tx_fifo_element_s*)(line_resources->tx_buffer);

    rx_fifo_data->responses_downcounter--;

    rx_fifo_data->cumulative_error |= (packet->instruction_error)&(INSTRUCTION_ERROR|CHECKSUM_ERROR|RANGE_ERROR);

    if(rx_fifo_data->cumulative_error)
    {     
        if(tx_fifo_data->retransmit_downcounter)
        {
            if(rx_fifo_data->responses_downcounter)
                return;
            Retransmit(line_resources);
            return;
        }
    }
        
    if(tx_fifo_data->response != NULL){
        tx_fifo_data->response->bus_error = 0;
        tx_fifo_data->response->device_error = packet->instruction_error;
        tx_fifo_data->response->data_length = packet->length-2;
        memcpy(tx_fifo_data->response->data, packet->param_crc, packet->length-2);
    }
    

    if(rx_fifo_data->responses_downcounter){
        if(tx_fifo_data->response != NULL)
            *((uint32_t*)(&(tx_fifo_data->response))) += sizeof(struct dynamixel_master_response_s);
        usartx_control->StartRx(line_resources);
        return;
    }

    if(tx_fifo_data->response != NULL)
        tx_fifo_data->response->is_data_ready = 1;

    Check_TX_FIFO(line_resources);
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

    uint32_t tx_fifo_size = line_resources->tx_buffer_size/sizeof(struct tx_fifo_element_s) - 1;

    FIFO_StructInit(&line_resources->tx_fifo, tx_fifo, tx_fifo_size, sizeof(struct tx_fifo_element_s));

    line_resources->is_line_busy = 0;
    
    line_config->TxCompleteCallback = TxCompleteCallback;
    line_config->RxCompleteCallback = RxCompleteCallback;
    line_config->RxTimeoutCallback = RxTimeoutCallback;
    line_config->is_line_half_duplex = 1;
}

static struct usartx_protocol_s protocol = {.InitLine = InitLine};

static struct usartx_protocol_s* GetProtocol(void)
{
    return &protocol;
}

const struct dynamixel_master_s DynamixelMaster = {.Ping = PingCmd,
                                                    .Read = ReadCmd,
                                                    .Write = WriteCmd,
                                                    .RegWrite = RegWriteCmd,
                                                    .Action = ActionCmd,
                                                    .FactoryReset = FactoryResetCmd,
                                                    .Reboot = RebootCmd,
                                                    .SyncWrite = SyncWriteCmd,
                                                    .BulkRead = BulkReadCmd,
                                                    .GetProtocol = GetProtocol};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
