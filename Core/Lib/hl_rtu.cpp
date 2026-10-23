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

#include "board_gpio_defines.hpp"
#include "stm32f4xx_ll_gpio.h"


#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "usartx_timeout.hpp" 


struct pdu_s{
    uint8_t pdu_length;
    uint8_t cmd;
    uint8_t data_crc[HL_RTU_MAX_DATA_LENGTH+1];
};

struct packet_s{
    uint8_t header1;
    uint8_t header2;
    struct pdu_s pdu;
};

struct tx_fifo_element_s{
    struct pdu_s pdu;
};

struct rx_fifo_element_s{
    struct pdu_s pdu;
};

static struct usartx_control_s* usartx_control;


static uint8_t Calc_CRC(struct pdu_s* pdu)
{
    uint16_t crc = pdu->pdu_length + pdu->cmd;
	
	for(uint8_t ik=0; ik<(pdu->pdu_length-2); ik++)
		crc += pdu->data_crc[ik];
	
	crc = ~crc;
	crc &= 0xFF;

    return (uint8_t)crc;
}

static void TransmitNext(struct usartx_line_resources_s* line_resources)
{
    struct packet_s* packet = (struct packet_s*)line_resources->tx_buffer;

    packet->header1 = 0xFF;
    packet->header2 = 0xFF;
    FIFO_Remove(&line_resources->tx_fifo, &packet->pdu);

    usartx_control->StartTx(line_resources, packet->pdu.pdu_length+3);
}

static void TransmitCmd(USART_TypeDef* usart, struct tx_fifo_element_s* tx_fifo_data_unit)
{
    struct usartx_line_resources_s* line_resources = usartx_control->GetLineResources(usart);

    while(1)
    {
        vTaskSuspendAll();

        if(FIFO_IsNotFull(&line_resources->tx_fifo))
            break;

        xTaskResumeAll();
        taskYIELD();
    }
    
    FIFO_Add(&line_resources->tx_fifo, tx_fifo_data_unit);

    if(line_resources->is_line_busy==0){
        line_resources->is_line_busy = 1;
        TransmitNext(line_resources);
    }

    xTaskResumeAll();
}

static void SendResponse(USART_TypeDef* usart, struct hl_rtu_response_s* response)
{
    struct tx_fifo_element_s tx_fifo_data_unit;

    tx_fifo_data_unit.pdu.cmd = response->cmd;
    tx_fifo_data_unit.pdu.pdu_length = response->data_length+2;
    memcpy(tx_fifo_data_unit.pdu.data_crc, response->data, response->data_length);
    tx_fifo_data_unit.pdu.data_crc[response->data_length] = Calc_CRC(&tx_fifo_data_unit.pdu);

    TransmitCmd(usart, &tx_fifo_data_unit);
}

static uint8_t GetRequest(USART_TypeDef* usart, struct hl_rtu_request_s* request)
{
    struct usartx_line_resources_s* line_resources = usartx_control->GetLineResources(usart);

    if(FIFO_IsNotEmpty(&(line_resources->rx_fifo))){
        struct rx_fifo_element_s rx_fifo_data_unit;
        FIFO_Remove(&line_resources->rx_fifo, &rx_fifo_data_unit);
        request->cmd = rx_fifo_data_unit.pdu.cmd;
        request->data_length = rx_fifo_data_unit.pdu.pdu_length-2;
        memcpy(request->data, rx_fifo_data_unit.pdu.data_crc, rx_fifo_data_unit.pdu.pdu_length-2);
        return 1;
    }

    return 0;
}

static uint8_t VerifyPacket(struct packet_s* packet, uint8_t pdu_length)
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

static void TxCompleteCallback(struct usartx_line_resources_s* line_resources)
{
    usartx_control->StopTx(line_resources);
    
    if(FIFO_IsNotEmpty(&line_resources->tx_fifo))
        TransmitNext(line_resources);
    else
        line_resources->is_line_busy = 0;
}

static void RxCompleteCallback(struct usartx_line_resources_s* line_resources, uint32_t data_length)
{
    usartx_control->StopRx(line_resources);

    struct packet_s* packet = (struct packet_s*)line_resources->rx_buffer;

    if(VerifyPacket(packet,data_length)==0)
    {
        if(FIFO_IsNotFull(&line_resources->rx_fifo))
	        FIFO_Add(&line_resources->rx_fifo, &packet->pdu);
    }
    
    memset(line_resources->rx_buffer, 0x55, sizeof(struct packet_s));

    usartx_control->StartRx(line_resources);
    
}

static void InitLine(struct usartx_line_init_params_s* usartx_line_init_params)
{
    struct usartx_line_resources_s* line_resources = usartx_line_init_params->line_resources;
    struct usartx_line_config_s* line_config = usartx_line_init_params->line_config;
    usartx_control = usartx_line_init_params->usartx_control;

    if((line_resources->tx_buffer_size < (sizeof(struct tx_fifo_element_s)+sizeof(struct packet_s)))||
        (line_resources->rx_buffer_size < (sizeof(struct rx_fifo_element_s)+sizeof(struct packet_s))))
        return;

    uint8_t* tx_fifo = (uint8_t*)((uint32_t)line_resources->tx_buffer + sizeof(struct packet_s));
    uint8_t* rx_fifo = (uint8_t*)((uint32_t)line_resources->rx_buffer + sizeof(struct packet_s));

    uint32_t tx_fifo_size = (line_resources->tx_buffer_size - sizeof(struct packet_s))/sizeof(struct tx_fifo_element_s);
    uint32_t rx_fifo_size = (line_resources->rx_buffer_size - sizeof(struct packet_s))/sizeof(struct rx_fifo_element_s);

    FIFO_StructInit(&line_resources->tx_fifo, tx_fifo, tx_fifo_size, sizeof(struct tx_fifo_element_s));
    FIFO_StructInit(&line_resources->rx_fifo, rx_fifo, rx_fifo_size, sizeof(struct rx_fifo_element_s));

    line_resources->is_line_busy = 0;
    
    line_config->TxCompleteCallback = TxCompleteCallback;
    line_config->RxCompleteCallback = RxCompleteCallback;
    line_config->RxTimeoutCallback = NULL;
    line_config->is_line_half_duplex = 0;

    memset(line_resources->rx_buffer, 0x55, sizeof(struct packet_s));
    usartx_control->StartRx(line_resources);
}

static struct usartx_protocol_s protocol = {.InitLine = InitLine};

static struct usartx_protocol_s* GetProtocol(void)
{
    return &protocol;
}

const struct hl_rtu_s HL_RTU = {.SendResponse = SendResponse,
                                    .GetRequest = GetRequest,
                                    .GetProtocol = GetProtocol};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
