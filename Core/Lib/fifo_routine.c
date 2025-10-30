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
#include "fifo_routine.h"

/*  Error Codes */
#define ERROR   1
#define OK      0


void FIFO_StructInit(struct fifo_s* fifo, void* fifo_ptr, uint32_t fifo_size, uint32_t fifo_element_size)
{
  fifo->fifo_bot_edge_ptr = fifo_ptr;
  fifo->fifo_top_edge_ptr = (void*)((uint32_t)fifo_ptr + (fifo_size-1) * fifo_element_size);
  fifo->head_ptr = fifo_ptr;
  fifo->tail_ptr = fifo_ptr;
  fifo->fifo_size = fifo_size;
  fifo->fifo_element_size = fifo_element_size;
  fifo->elements_count = 0;
}

static void FIFO_MovePtr(void* ptr_to_ptr, void* fifo_bot_edge_ptr, void* fifo_top_edge_ptr, uint32_t fifo_element_size)
{
	if(*((uint32_t*)ptr_to_ptr) < (uint32_t)fifo_top_edge_ptr)
		*((uint32_t*)ptr_to_ptr) += fifo_element_size;
	else
		*((uint32_t*)ptr_to_ptr) = (uint32_t)fifo_bot_edge_ptr;
}

uint8_t FIFO_Add(struct fifo_s* fifo, void* data_in_ptr)
{
  if((fifo->elements_count+1)==fifo->fifo_size)
    return ERROR;

  memcpy(fifo->tail_ptr, data_in_ptr, fifo->fifo_element_size);
  FIFO_MovePtr(&(fifo->tail_ptr), fifo->fifo_bot_edge_ptr, fifo->fifo_top_edge_ptr, fifo->fifo_element_size);
  fifo->elements_count++;
  return OK;
}

uint8_t FIFO_Remove(struct fifo_s* fifo, void* data_out_ptr)
{
  if(fifo->elements_count==0)
    return ERROR;

  memcpy(data_out_ptr, fifo->head_ptr, fifo->fifo_element_size);
  FIFO_MovePtr(&(fifo->head_ptr), fifo->fifo_bot_edge_ptr, fifo->fifo_top_edge_ptr, fifo->fifo_element_size);
  fifo->elements_count--;
  return OK;
}

uint32_t FIFO_GetElementsCount(struct fifo_s* fifo)
{
  return fifo->elements_count;
}

uint8_t FIFO_IsFull(struct fifo_s* fifo)
{
  return ((fifo->elements_count+1)==fifo->fifo_size) ? 1 : 0;
}

uint8_t FIFO_IsNotFull(struct fifo_s* fifo)
{
  return ((fifo->elements_count+1)==fifo->fifo_size) ? 0 : 1;
}

uint8_t FIFO_IsEmpty(struct fifo_s* fifo)
{
  return (fifo->elements_count==0) ? 1 : 0;
}

uint8_t FIFO_IsNotEmpty(struct fifo_s* fifo)
{
  return (fifo->elements_count==0) ? 0 : 1;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
