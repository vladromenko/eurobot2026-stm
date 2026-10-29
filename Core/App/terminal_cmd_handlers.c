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

#include "terminal_cmd_handlers.h" 
#include "dynamixel_master.h"
#include "byte2word.h"

#include "board_gpio_defines.h"
#include "stm32f4xx_ll_gpio.h"
#include "terminal.h"

#include "dynamixel_ax.h" 
#include "dynamixel_mx.h" 
#include "stm32f4xx_ll_usart.h"
#include "os_tasks.h"
#include "kinematics.h"
#include "odometry.h" 
#include "bms_board.h"
#include "bus_device.h"
#include "spi_gpo.h"
#include "spi_gpo_defines.h"
#include "hw_instance.h"
#include "stepper.h"


static void Ping(uint8_t* data, uint8_t data_length)
{
  LL_GPIO_TogglePin(GPIOD, PWM2_PD14);

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_Ping;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void AX_GetAngle(uint8_t* data, uint8_t data_length)
{
  const struct hw_instance_s* hw = GetHwInstance();
  uint16_t angle;
  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_AX_GetAngle;
  terminal_response.data[0] = hw->dynamixel_ax->GetAngle(USART3, data[0], &angle);
  Word16_ToByte(&angle, &terminal_response.data[1]);
  terminal_response.data_length = 3;

  Terminal.SendResponse(&terminal_response);
}

static void AX_SetAngle(uint8_t* data, uint8_t data_length)
{
  const struct hw_instance_s* hw = GetHwInstance();
//  LL_GPIO_TogglePin(GPIOD, PWM1_PD15);
  uint8_t id = data[0];
  uint16_t angle;
  
  ByteTo_Word16(data+1,&angle);

  hw->dynamixel_ax->TorqueEnable(USART3,id);
  hw->dynamixel_ax->SetAngle(USART3,id,angle);

  struct terminal_response_s terminal_response;

  terminal_response.cmd = BOARD_AX_SetAngle;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void SetRobotSpeed(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct speed_s speed;
  
  ByteTo_Word32(data,&speed.dx);
  ByteTo_Word32(data+4,&speed.dy);
  ByteTo_Word32(data+8,&speed.dphi);

  Odometry.SetSpeed(&speed);

  terminal_response.cmd = BOARD_SetSpeed;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void GetRobotSpeed(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct speed_s speed;

  Odometry.GetSpeed(&speed);

  Word32_ToByte(&speed.dx,terminal_response.data);
  Word32_ToByte(&speed.dy,terminal_response.data+4);
  Word32_ToByte(&speed.dphi,terminal_response.data+8);

  terminal_response.cmd = BOARD_GetSpeed;
  terminal_response.data_length = 12;

  Terminal.SendResponse(&terminal_response);
}

static void SetRobotCoord(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct coord_s coord;
  
  ByteTo_Word32(data,&coord.x);
  ByteTo_Word32(data+4,&coord.y);
  ByteTo_Word32(data+8,&coord.phi);

  Odometry.SetCoord(&coord);

  terminal_response.cmd = BOARD_SetCoord;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void GetRobotCoord(uint8_t* data, uint8_t data_length)
{
  struct terminal_response_s terminal_response;

  struct coord_s coord;

  Odometry.GetCoord(&coord);

  Word32_ToByte(&coord.x,terminal_response.data);
  Word32_ToByte(&coord.y,terminal_response.data+4);
  Word32_ToByte(&coord.phi,terminal_response.data+8);

  terminal_response.cmd = BOARD_GetCoord;
  terminal_response.data_length = 12;
  
  Terminal.SendResponse(&terminal_response);
}

static void GetBatteryState(uint8_t* data, uint8_t data_length)
{
  const struct hw_instance_s* hw = GetHwInstance();
  struct terminal_response_s terminal_response;

  struct bms_battery_state_s battery_state;
  hw->bms->GetBatteryState(hw->bus_devices[BMS_BOARD].usart, hw->bus_devices[BMS_BOARD].id, &battery_state);
  memcpy(terminal_response.data, &battery_state, 18);

  terminal_response.cmd = BOARD_GetBatteryState;
  terminal_response.data_length = 18;
  
  Terminal.SendResponse(&terminal_response);
}

static void Set_BMS_CalibrationValue(uint8_t* data, uint8_t data_length)
{
  const struct hw_instance_s* hw = GetHwInstance();
  struct terminal_response_s terminal_response;

  uint16_t calibration_value;
  ByteTo_Word16(data+1, &calibration_value);

  hw->bms->SetCalibrationValue(hw->bus_devices[BMS_BOARD].usart, hw->bus_devices[BMS_BOARD].id, data[0], calibration_value);
  
  terminal_response.cmd = BOARD_Set_BMS_CalibrationValue;
  terminal_response.data_length = 0;
  
  Terminal.SendResponse(&terminal_response);
}

static void ToggleGripper(uint8_t* data, uint8_t data_length)
{
  extern uint8_t gripper_state;
  struct terminal_response_s terminal_response;

  gripper_state = data[0];
  vTaskResume(GripperTask.GetTaskHandle());

  // switch (data[0])
  // {
  //   case 0:
  //     Dynamixel_AX.SetAngle(bus_devices[GRIPPER_TOP].usart, bus_devices[GRIPPER_TOP].id, 0);
  //     Dynamixel_AX.SetAngle(bus_devices[GRIPPER_BOTTOM].usart, bus_devices[GRIPPER_BOTTOM].id, 0);
  //     break;
  //   case 1:
  //     Dynamixel_AX.SetAngle(bus_devices[GRIPPER_TOP].usart, bus_devices[GRIPPER_TOP].id, 100);
  //     Dynamixel_AX.SetAngle(bus_devices[GRIPPER_BOTTOM].usart, bus_devices[GRIPPER_BOTTOM].id, 100);
  //     break;
  // }
  
  terminal_response.cmd = BOARD_TOGGLE_GRIPPER;
  terminal_response.data_length = 0;
  
  Terminal.SendResponse(&terminal_response);
}

static void PumpOn(uint8_t* data, uint8_t data_length)
{
  const struct hw_instance_s* hw = GetHwInstance();
  struct terminal_response_s terminal_response;

  hw->spi_gpo->SetOutputPin(SPI_POWER_OUTPUT_1|SPI_POWER_OUTPUT_2|SPI_POWER_OUTPUT_3|SPI_POWER_OUTPUT_4);

  terminal_response.cmd = BOARD_POWER_ON_PUMP;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}

static void PumpOff(uint8_t* data, uint8_t data_length)
{
  const struct hw_instance_s* hw = GetHwInstance();
  struct terminal_response_s terminal_response;

  hw->spi_gpo->ResetOutputPin(SPI_POWER_OUTPUT_1|SPI_POWER_OUTPUT_2|SPI_POWER_OUTPUT_3|SPI_POWER_OUTPUT_4);

  terminal_response.cmd = BOARD_POWER_OFF_PUMP;
  terminal_response.data_length = 0;

  Terminal.SendResponse(&terminal_response);
}


static uint8_t steppers_ready = 0;

static void StepperTest(uint8_t* data, uint8_t data_length)
{
    const struct hw_instance_s* hw = GetHwInstance();

    struct terminal_response_s response = {0};
    response.cmd = BOARD_STEPPER_TEST;
    response.data_length = 1;

    // 0 — команда принята
    // 1 — неверная команда
    // 2 — двигатели не инициализированы
    response.data[0] = 0;

    if (data_length != 1)
    {
        response.data[0] = 1;
        Terminal.SendResponse(&response);
        return;
    }

    switch (data[0])
    {
        case 0: // INIT
        {
            if (!steppers_ready)
            {
                hw->stepper->Init(STEPPER_MOTOR_1);
                hw->stepper->Init(STEPPER_MOTOR_2);

                hw->stepper->SetZeroPosisition(STEPPER_MOTOR_1);
                hw->stepper->SetZeroPosisition(STEPPER_MOTOR_2);

                hw->stepper->Enable(STEPPER_MOTOR_1);
                hw->stepper->Enable(STEPPER_MOTOR_2);

                steppers_ready = 1;
            }
            break;
        }

        case 1: // FORWARD
        {
            if (!steppers_ready)
            {
                response.data[0] = 2;
                break;
            }

            hw->stepper->StepToPosisition(
                STEPPER_MOTOR_1, 20000, 32, 6, 800
            );

            hw->stepper->StepToPosisition(
                STEPPER_MOTOR_2, 20000, 32, 6, 800
            );
            break;
        }

        case 2: // BACKWARD
        {
            if (!steppers_ready)
            {
                response.data[0] = 2;
                break;
            }

            hw->stepper->StepToPosisition(
                STEPPER_MOTOR_1, -32, 32, 32, 0
            );

            hw->stepper->StepToPosisition(
                STEPPER_MOTOR_2, -32, 32, 32, 0
            );
            break;
        }

        case 3: // RETURN TO ZERO
        {
            if (!steppers_ready)
            {
                response.data[0] = 2;
                break;
            }

            hw->stepper->StepToPosisition(
                STEPPER_MOTOR_1, 0, 32, 32, 0
            );

            hw->stepper->StepToPosisition(
                STEPPER_MOTOR_2, 0, 32, 32, 0
            );
            break;
        }

        case 4: // STOP
        {
            if (!steppers_ready)
            {
                response.data[0] = 2;
                break;
            }

            hw->stepper->StopMotor(STEPPER_MOTOR_1);
            hw->stepper->StopMotor(STEPPER_MOTOR_2);
            break;
        }

        case 5: // DISABLE
        {
            if (!steppers_ready)
            {
                response.data[0] = 2;
                break;
            }

            hw->stepper->StopMotor(STEPPER_MOTOR_1);
            hw->stepper->StopMotor(STEPPER_MOTOR_2);

            hw->stepper->Disable(STEPPER_MOTOR_1);
            hw->stepper->Disable(STEPPER_MOTOR_2);

            steppers_ready = 0;
            break;
        }

        default:
        {
            response.data[0] = 1;
            break;
        }
    }

    Terminal.SendResponse(&response);
}


void (*terminal_cmd_handlers[TERMINAL_CMD_TABLE_SIZE])(uint8_t* data, uint8_t data_length) = {[BOARD_Ping] = Ping,
                                                                                              [BOARD_AX_GetAngle] = AX_GetAngle,
                                                                                              [BOARD_AX_SetAngle] = AX_SetAngle,
                                                                                              [BOARD_SetSpeed] = SetRobotSpeed,
                                                                                              [BOARD_GetSpeed] = GetRobotSpeed,
                                                                                              [BOARD_SetCoord] = SetRobotCoord,
                                                                                              [BOARD_GetCoord] = GetRobotCoord,
                                                                                              [BOARD_GetBatteryState] = GetBatteryState,
                                                                                              [BOARD_Set_BMS_CalibrationValue] = Set_BMS_CalibrationValue,
                                                                                              [BOARD_TOGGLE_GRIPPER] = ToggleGripper,
                                                                                              [BOARD_POWER_ON_PUMP] = PumpOn,
                                                                                              [BOARD_POWER_OFF_PUMP] = PumpOff,
                                                                                              [BOARD_STEPPER_TEST] = StepperTest
                                                                                            };

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
