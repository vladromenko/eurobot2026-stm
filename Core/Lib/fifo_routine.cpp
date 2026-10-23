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
#include "fifo_routine.hpp"

void FIFO::Init(void* fifo_ptr, uint32_t fifo_size, uint32_t fifo_element_size)
{
  this->fifo_bot_edge_ptr = static_cast<uint8_t*>(fifo_ptr);
  this->fifo_top_edge_ptr = fifo_bot_edge_ptr + (fifo_size-1) * fifo_element_size;
  this->head_ptr = fifo_bot_edge_ptr;
  this->tail_ptr = fifo_bot_edge_ptr;
  this->fifo_size = fifo_size;
  this->fifo_element_size = fifo_element_size;
  this->elements_count = 0;
}

uint8_t* FIFO::NextPtr(uint8_t* ptr) const
{
  return (ptr < fifo_top_edge_ptr) ? (ptr + fifo_element_size) : fifo_bot_edge_ptr;
}

uint8_t FIFO::Add(const void* data_in_ptr)
{
  if(IsFull())
    return 1;

  memcpy(tail_ptr, data_in_ptr, fifo_element_size);
  tail_ptr = NextPtr(tail_ptr);
  elements_count++;
  return 0;
}

uint8_t FIFO::Remove(void* data_out_ptr)
{
  if(IsEmpty())
    return 1;

  memcpy(data_out_ptr, head_ptr, fifo_element_size);
  head_ptr = NextPtr(head_ptr);
  elements_count--;
  return 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
