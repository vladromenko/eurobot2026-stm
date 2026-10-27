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

#include "os_tasks.h" 
#include "dynamixel_master.h"
#include "stm32f4xx_ll_gpio.h"
#include "board_gpio_defines.h"
#include "robot_gpio_defines.h"
#include "byte2word.h" 
#include "maxon.h" 
#include "dynamixel_ax.h" 
#include "stepper.h" 
#include "rc_servo.h" 
#include "stm32f4xx_ll_tim.h"
#include "terminal.h"
#include "h_bridge.h"
#include "serial.h"

#include "spi_gpo.h" 
#include "spi_gpo_defines.h"
#include "kinematics.h"
#include "odometry.h"
#include "ws2812b.h" 
#include "robot_behavior_routine.h"

// #define WS2812B_DEMO
#define SPI_GPO_DEMO
// #define STEPPER_DEMO
// #define DYNAMIXEL_DEMO_1
// #define DYNAMIXEL_DEMO_2
// #define MAXON_DEMO
// #define SERVO_DEMO


#define TASK_STACK_SIZE             512

static TaskHandle_t TaskHandle;
static StackType_t StackBuffer[TASK_STACK_SIZE];
static StaticTask_t TaskBuffer;

#ifdef WS2812B_DEMO
  #define WS2812B_LEDS_NUMBER   20
  static struct rgb_color_s ws_led[WS2812B_LEDS_NUMBER];
#endif

static void Task(void *arg)
{
  #if defined(DYNAMIXEL_DEMO_1) || defined(DYNAMIXEL_DEMO_2)
    vTaskDelay(100);
    uint8_t dynamixel_id;
  #endif

  #ifdef DYNAMIXEL_DEMO_1
    dynamixel_id = 4;
    Dynamixel_AX.TorqueEnable(USART3, dynamixel_id);
  #endif

  #ifdef DYNAMIXEL_DEMO_2
    dynamixel_id = 0xFE;
    while(dynamixel_id){
      dynamixel_id --;
      if(Dynamixel_AX.Ping(USART3, dynamixel_id) == 0)
        break;
    }
    Dynamixel_AX.TorqueEnable(USART3, dynamixel_id);
  #endif

  #ifdef MAXON_DEMO
    Kinematics.MatrixInit();
    MaxonsEnable();
  #endif

  #ifdef STEPPER_DEMO
    Stepper.Init(STEPPER_MOTOR_1);
    Stepper.Init(STEPPER_MOTOR_2);
    Stepper.Init(STEPPER_MOTOR_3);
    Stepper.Init(STEPPER_MOTOR_4);

    Stepper.Enable(STEPPER_MOTOR_1);
    Stepper.Enable(STEPPER_MOTOR_2);
    Stepper.Enable(STEPPER_MOTOR_3);
    Stepper.Enable(STEPPER_MOTOR_4);

    Stepper.SetZeroPosisition(STEPPER_MOTOR_1);
    Stepper.SetZeroPosisition(STEPPER_MOTOR_2);
    Stepper.SetZeroPosisition(STEPPER_MOTOR_3);
    Stepper.SetZeroPosisition(STEPPER_MOTOR_4);
  #endif

  #ifdef SERVO_DEMO
    RC_Servo.Enable(RC_SERVO_1);
    RC_Servo.Enable(RC_SERVO_2);
  #endif

  #ifdef WS2812B_DEMO
    for(uint8_t ik = 0; ik < WS2812B_LEDS_NUMBER; ik ++){
      ws_led[ik].b = 255 - ik*12;
      ws_led[ik].r = 0 + ik*12;
      ws_led[ik].g = 0;
    }
  #endif

  
  
	while(1)
	{
    #ifdef SPI_GPO_DEMO
      SPI_GPO_SetOutputPin(SPI_POWER_OUTPUT_1|SPI_POWER_OUTPUT_2|SPI_POWER_OUTPUT_3|SPI_POWER_OUTPUT_4);
      vTaskDelay(250);
      SPI_GPO_ResetOutputPin(SPI_POWER_OUTPUT_1|SPI_POWER_OUTPUT_2|SPI_POWER_OUTPUT_3|SPI_POWER_OUTPUT_4);
      vTaskDelay(250);
    #endif

    #if defined(DYNAMIXEL_DEMO_1) || defined(DYNAMIXEL_DEMO_2)
      uint16_t angle;
      uint8_t error_code;

      error_code = Dynamixel_AX.GetAngle(USART3, dynamixel_id, &angle);

      if(error_code == 0){
        angle += 10;
        if(angle > 300) angle = 0;
        Dynamixel_AX.SetAngle(USART3, dynamixel_id, angle);
        LL_GPIO_ResetOutputPin(GPIOD, PWM1_PD15);
      }else
        LL_GPIO_SetOutputPin(GPIOD, PWM1_PD15);

      if(angle < 100)
        vTaskDelay(250);

      if(angle < 200)
        vTaskDelay(125);

      vTaskDelay(125);

      if(angle == 0)
        vTaskDelay(1000);
    #endif

    #ifdef MAXON_DEMO
      struct speed_s speed;

      speed.dphi = 3.14f;
      speed.dx = 0;
      speed.dy = 0;
      Odometry.SetSpeed(&speed);

      vTaskDelay(2000);

      speed.dphi = -3.14f;
      Odometry.SetSpeed(&speed);

      vTaskDelay(2000);

      speed.dphi = 0;
      speed.dx = 0.3;
      Odometry.SetSpeed(&speed);

      vTaskDelay(2000);

      speed.dx = 0;
      speed.dy = 0.3;
      Odometry.SetSpeed(&speed);

      vTaskDelay(2000);

      speed.dx = -0.3;
      speed.dy = 0;
      Odometry.SetSpeed(&speed);

      vTaskDelay(2000);

      speed.dx = 0;
      speed.dy = -0.3;
      Odometry.SetSpeed(&speed);

      vTaskDelay(2000);
    #endif

    #ifdef SERVO_DEMO
      uint16_t servo_1_range = 120;
      uint16_t servo_2_range = 180;

      RC_Servo.SetServoAngle(RC_SERVO_1, 1100, 1900, servo_1_range, 0);
      RC_Servo.SetServoAngle(RC_SERVO_2, 500, 2500, servo_2_range, 0);

      vTaskDelay(2000);

      RC_Servo.SetServoAngle(RC_SERVO_1, 1100, 1900, servo_1_range, 90);
      RC_Servo.SetServoAngle(RC_SERVO_2, 500, 2500, servo_2_range, 90);

      vTaskDelay(1000);

      RC_Servo.SetServoAngle(RC_SERVO_1, 1100, 1900, servo_1_range, servo_1_range);
      RC_Servo.SetServoAngle(RC_SERVO_2, 500, 2500, servo_2_range, servo_2_range);

      vTaskDelay(1000);
    #endif

    #ifdef STEPPER_DEMO
      uint32_t stpper1_goal_position = 20000;
      uint32_t stpper2_goal_position = 8000;
      uint32_t stpper3_goal_position = 20000;
      uint32_t stpper4_goal_position = 20000;

      Stepper.StepToPosisition(STEPPER_MOTOR_1, stpper1_goal_position, 32, 6, 800);
      Stepper.StepToPosisition(STEPPER_MOTOR_2, stpper2_goal_position, 32, 4, 800);
      Stepper.StepToPosisition(STEPPER_MOTOR_3, stpper3_goal_position, 20, 1, 200);
      Stepper.StepToPosisition(STEPPER_MOTOR_4, stpper4_goal_position, 2, 2, 0);
      while((Stepper.GetPresentPosisition(STEPPER_MOTOR_1) != stpper1_goal_position)|
            (Stepper.GetPresentPosisition(STEPPER_MOTOR_2) != stpper2_goal_position)|
            (Stepper.GetPresentPosisition(STEPPER_MOTOR_3) != stpper3_goal_position)|
            (Stepper.GetPresentPosisition(STEPPER_MOTOR_4) != stpper4_goal_position)) taskYIELD();
      vTaskDelay(500);

      Stepper.StepToPosisition(STEPPER_MOTOR_1, 0, 32, 1, 800);
      Stepper.StepToPosisition(STEPPER_MOTOR_2, 0, 32, 4, 800);
      Stepper.StepToPosisition(STEPPER_MOTOR_3, 0, 20, 1, 200);
      Stepper.StepToPosisition(STEPPER_MOTOR_4, 0, 2, 2, 0);
      while((Stepper.GetPresentPosisition(STEPPER_MOTOR_1) != 0)|
            (Stepper.GetPresentPosisition(STEPPER_MOTOR_2) != 0)|
            (Stepper.GetPresentPosisition(STEPPER_MOTOR_3) != 0)|
            (Stepper.GetPresentPosisition(STEPPER_MOTOR_4) != 0)) taskYIELD();
      vTaskDelay(500);
    #endif

    #ifdef WS2812B_DEMO
      struct rgb_color_s temp_color = ws_led[WS2812B_LEDS_NUMBER - 1];

      for(uint8_t ik = (WS2812B_LEDS_NUMBER - 1); ik > 0; ik --)
        ws_led[ik] = ws_led[ik - 1];

      ws_led[0] = temp_color;
      WS2812B.Set_RGB_Color(ws_led, WS2812B_LEDS_NUMBER);
      vTaskDelay(50);
    #endif

	}
}

static TaskHandle_t GetTaskHandle(void)
{
    return TaskHandle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    TaskHandle = xTaskCreateStatic(Task, "DemoTask", TASK_STACK_SIZE,
                          NULL, uxPriority, StackBuffer, &TaskBuffer);
}


const struct os_task_base_s DemoTask = {.CreateTask = CreateTask,
                               .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
