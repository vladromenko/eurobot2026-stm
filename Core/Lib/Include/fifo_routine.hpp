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
#ifndef __FIFO_ROUTINE_H
#define __FIFO_ROUTINE_H

#include "std_def.hpp"

/**
 * Ring buffer of fixed-size elements over external memory.
 * The memory is not owned: USART lines hand a part of their DMA buffer to it.
 * One slot is always kept free, so a FIFO of size N holds N-1 elements.
 */
class FIFO{
public:
  void Init(void* fifo_ptr, uint32_t fifo_size, uint32_t fifo_element_size);

  /*  Return 0 on success, 1 if the FIFO is full / empty  */
  uint8_t Add(const void* data_in_ptr);
  uint8_t Remove(void* data_out_ptr);

  uint32_t GetElementsCount() const { return elements_count; }
  uint8_t IsFull() const { return (elements_count+1)==fifo_size; }
  uint8_t IsNotFull() const { return !IsFull(); }
  uint8_t IsEmpty() const { return elements_count==0; }
  uint8_t IsNotEmpty() const { return !IsEmpty(); }

private:
  uint8_t* NextPtr(uint8_t* ptr) const;

  uint8_t* fifo_bot_edge_ptr;
  uint8_t* fifo_top_edge_ptr;
  uint8_t* head_ptr;
  uint8_t* tail_ptr;
  uint32_t fifo_size;
  uint32_t fifo_element_size;
  uint32_t elements_count;
};

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
