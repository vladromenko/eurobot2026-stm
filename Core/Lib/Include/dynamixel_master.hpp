/**
  ******************************************************************************
  * @file    	functions.h
  * @brief   HAL configuration file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DYNAMIXEL_MASTER_H
#define __DYNAMIXEL_MASTER_H

#include "std_def.hpp"
#include "stm32f4xx_ll_usart.h"
#include "usartx_communication.hpp"



#define BROADCAST_ID            0xFE
#define MAX_DATA_LENGTH         32

/* Dynamixel Protocol 1.0 commands define ----------------------------------------------------------------------------------------------------------------------- */
#define Ping_CMD      				  0x01        /* Instruction that checks whether the Packet has arrived to a device with the same ID as Packet ID                   */
#define Read_CMD      				  0x02        /* Instruction to read data from the Device                                                                           */
#define Write_CMD        			  0x03        /* Instruction to write data on the Device	                                                                          */
#define RegWrite_CMD       			0x04        /* Instruction that registers the Instruction Packet to a standby status 	                                            */
#define Action_CMD     				  0x05        /* Instruction that executes the Packet that was registered beforehand using Reg Write	                              */
#define FactoryReset_CMD    		0x06        /* Instruction that resets the Control Table to its initial factory default settings 	                                */
#define Reboot_CMD       			  0x08        /* Instruction that reboots DYNAMIXEL (See supported products in the description)	                                    */

/*  NOT USED IN AX12A */
#define SyncWrite_CMD     			0x83        /* For multiple devices, Instruction to write data on the same Address with the same length at once 	                */
#define BulkRead_CMD        		0x92        /* For multiple devices, Instruction to write data on different Addresses with different lengths at once 	            */

/*	Dynamixel Protocol 1.0 ERROR codes define ------------------------------------------------------------------------------------------------------------------- */
#define NO_DEVICE_ERROR         0    				/* No error    						                                                                                            */
#define INSTRUCTION_ERROR       (1U<<6UL)   /* In case of sending an undefined instruction or delivering the action instruction without the Reg Write instruction */
#define OVERLOAD_ERROR       		(1U<<5UL)   /* When the current load cannot be controlled by the set Torque	                                                      */
#define CHECKSUM_ERROR       		(1U<<4UL)   /* When the Checksum of the transmitted Instruction Packet is incorrect	                                              */
#define RANGE_ERROR       			(1U<<3UL)   /* When an instruction is out of the range for use	                                                                  */
#define OVERHEATING_ERROR       (1U<<2UL)   /* When internal temperature of DYNAMIXEL is out of the range of operating temperature set in the Control table	      */
#define ANGLE_LIMIT_ERROR       (1U<<1UL)   /* When Goal Position is written out of the range from CW Angle Limit to CCW Angle Limit	                            */
#define INPUT_VOLTAGE_ERROR     (1U<<0UL)   /* When the applied voltage is out of the range of operating voltage set in the Control table	                        */



struct dynamixel_master_response_s{
	uint8_t is_data_ready;
  uint8_t bus_error;
  uint8_t device_error;
  uint8_t data_length;
  uint8_t data[MAX_DATA_LENGTH];
};


/**
 * Dynamixel Protocol 1.0 master. One object serves all half-duplex lines
 * it is bound to; per-line state lives in the line buffers.
 * Requests are queued and the call returns at once: wait for
 * response->is_data_ready before reading the response.
 */
class DynamixelMasterDriver : public USARTx_Protocol{
public:
    void Ping(USART_TypeDef* usart, uint8_t device_id, dynamixel_master_response_s* response);
    void Read(USART_TypeDef* usart, uint8_t device_id, uint8_t starting_address, uint8_t data_length, dynamixel_master_response_s* response);
    void Write(USART_TypeDef* usart, uint8_t device_id, uint8_t starting_address, uint8_t* data, uint8_t data_length, dynamixel_master_response_s* response);
    void SyncWrite(USART_TypeDef* usart, uint8_t* device_id_array, uint8_t boards_number, uint8_t starting_address, uint8_t* data_array, uint8_t data_length, dynamixel_master_response_s* response_array);

private:
    void InitLine(USARTx_Line* line) override;
    void TxCompleteCallback(USARTx_Line* line) override;
    void RxCompleteCallback(USARTx_Line* line, uint32_t data_length) override;
    void RxTimeoutCallback(USARTx_Line* line) override;
};

extern DynamixelMasterDriver DynamixelMaster;


#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
