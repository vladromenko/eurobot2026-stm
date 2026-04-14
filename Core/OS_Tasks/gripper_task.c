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
/*
#include "os_tasks.h"

#include "dynamixel_ax.h" 
#include "bus_device.h"
#include "spi_gpo.h" 
#include "spi_gpo_defines.h"
#include "robot_behavior_routine.h"
#include "h_bridge.h"
#include "stm32f4xx_ll_gpio.h"
*/
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
#include "debug_terminal.h"
#include "h_bridge.h"
#include "serial.h"

#include "spi_gpo.h" 
#include "spi_gpo_defines.h"
#include "kinematics.h"
#include "odometry.h"
#include "ws2812b.h" 
#include "robot_behavior_routine.h"

#include "bus_device.h"
#include "pca9685.h"


#define NO_CMD 0xFF // Idle task (idle command)

#define TASK_STACK_SIZE             512


static TaskHandle_t task_handle;
static StackType_t stack_buffer[TASK_STACK_SIZE];
static StaticTask_t task_buffer;

#define GRIPPER_INIT

extern volatile TaskHandle_t xTerminalTaskHandle;

#define cos30 	0.8660254f
#define cos60 	0.5f

#define MM_TO_STEPS 25

uint8_t move_cmd_speed = 0; // [mm/s]
uint8_t move_cmd_time = 0;  // [ms]

// Structure describing all pins and IDs for one gripper
typedef struct {
    // Dynamixels ID
    uint8_t dyn_main_1;
    uint8_t dyn_main_2;
    uint8_t dyn_side_1;
    uint8_t dyn_side_2;
	
	// Dynamixels params
	uint16_t dyn_speed;
	uint8_t start_angle;
	uint8_t end_angle;
	uint8_t cursor_angle;
	uint16_t dyn_side_speed;
	uint16_t side_on_angle_1;
	uint16_t side_off_angle_1;
	uint16_t side_on_angle_2;
	uint16_t side_off_angle_2;
    
    // SPI GPO (pumps and valves)
    uint32_t pumps_pins[4];
    uint32_t pumps_all_mask;
    uint32_t valves_pins[4];
    uint32_t valves_all_mask;
    
    // PCA9685 Servos
    enum pca9685_channel_e block_servos[4];
    enum pca9685_channel_e shift_servo_5;
    enum pca9685_channel_e shift_servo_6;

	// Servos params
	uint8_t shift_servo_5_start_angle;
	uint8_t shift_servo_5_end_angle;
	uint8_t shift_servo_6_start_angle;
	uint8_t shift_servo_6_end_angle;

} GripperHardware_t;

// configs for two grippers: right (index 0) and left (index 1)
const GripperHardware_t grippers[2] = {
    { // right (index 0) (id == 1)
        .dyn_main_1 = 98, .dyn_main_2 = 112,
        .dyn_side_1 = 13, .dyn_side_2 = 14,
		.start_angle = 149, .end_angle = 65, .cursor_angle = 99, // 149 71 99
		.side_on_angle_1 = 140, .side_off_angle_1 = 89, //140 69
		.side_on_angle_2 = 51, .side_off_angle_2 = 107, //51 127
		.dyn_speed = 300, .dyn_side_speed = 300, //150 300
        .pumps_pins = {SPI_POWER_OUTPUT_1, SPI_POWER_OUTPUT_3, SPI_POWER_OUTPUT_5, SPI_POWER_OUTPUT_7},
        .pumps_all_mask = SPI_POWER_OUTPUT_1 | SPI_POWER_OUTPUT_3 | SPI_POWER_OUTPUT_5 | SPI_POWER_OUTPUT_7,
        .valves_pins = {SPI_POWER_OUTPUT_2, SPI_POWER_OUTPUT_4, SPI_POWER_OUTPUT_6, SPI_POWER_OUTPUT_8},
        .valves_all_mask = SPI_POWER_OUTPUT_2 | SPI_POWER_OUTPUT_4 | SPI_POWER_OUTPUT_6 | SPI_POWER_OUTPUT_8,
        .block_servos = {PCA9685_CH1, PCA9685_CH2, PCA9685_CH3, PCA9685_CH4},
        .shift_servo_5 = PCA9685_CH5, .shift_servo_6 = PCA9685_CH6,
		.shift_servo_5_end_angle = 16, .shift_servo_5_start_angle = 100,
		.shift_servo_6_end_angle = 112, .shift_servo_6_start_angle = 21
    },
    { // left (index 1) (id == 2)
        .dyn_main_1 = 15, .dyn_main_2 = 16,
        .dyn_side_1 = 7,  .dyn_side_2 = 8,
		.start_angle = 148, .end_angle = 130, .cursor_angle = 99, // start 148; end 130 65
		.side_on_angle_1 = 156, .side_off_angle_1 = 89,
		.side_on_angle_2 = 55, .side_off_angle_2 = 107,
		.dyn_speed = 300, .dyn_side_speed = 300,
        .pumps_pins = {SPI_POWER_OUTPUT_9, SPI_POWER_OUTPUT_11, SPI_POWER_OUTPUT_13, SPI_POWER_OUTPUT_15},
        .pumps_all_mask = SPI_POWER_OUTPUT_9 | SPI_POWER_OUTPUT_11 | SPI_POWER_OUTPUT_13 | SPI_POWER_OUTPUT_15,
        .valves_pins = {SPI_POWER_OUTPUT_10, SPI_POWER_OUTPUT_12, SPI_POWER_OUTPUT_14, SPI_POWER_OUTPUT_16},
        .valves_all_mask = SPI_POWER_OUTPUT_10 | SPI_POWER_OUTPUT_12 | SPI_POWER_OUTPUT_14 | SPI_POWER_OUTPUT_16,
        .block_servos = {PCA9685_CH8, PCA9685_CH9, PCA9685_CH10, PCA9685_CH11},
        .shift_servo_5 = PCA9685_CH12, .shift_servo_6 = PCA9685_CH13,
		.shift_servo_5_end_angle = 16, .shift_servo_5_start_angle = 110,
		.shift_servo_6_end_angle = 112, .shift_servo_6_start_angle = 24
    }
};


/* Gripper params */
uint16_t dynamixel_speed = 150; //0-1024
uint8_t gripper_target_angle;
uint16_t angle_timeout_ms = 1500; // ms to wait for dynamixel to reach the target angle before timeout
uint8_t gripper_height = 0;

// I2C address of the PWM driver
uint8_t i2c_addr_1 = 0x40; 

uint8_t servo_target_angle = 0;

/* Cursor params */
uint8_t dynamixel_cursor_id = 0xFF; //15
uint16_t cursor_speed = 300; //0-1024
uint8_t cursor_hor_angle = 40; //0-300 60
uint8_t cursor_vert_angle = 130; //0-300 150 125
uint8_t cursor_cmd;

/* Side grips params */
uint16_t side_dynamixel_speed = 300; //0-1024
/*
uint8_t side_off_angle_1 = 69; //0-300 44 99
uint8_t side_on_angle_1 = 140; //0-300 132 140
uint8_t side_off_angle_2 = 127; //0-300 152 97
uint8_t side_on_angle_2 = 51; //0-300 59 51
*/
uint8_t grips_cmd;

/* buffer params */
uint16_t current_angle = 0;
int16_t diff = 0;
uint16_t tol_window = 10; // [deg] angle tolerance (+-) in which angle is considered as achieved
uint8_t pump_mask;
uint8_t valve_mask;
uint8_t block_mask;

uint8_t gripper_state;

uint8_t lift_cmd;
uint8_t cmd_isFinished;
uint8_t gripper_id;

uint8_t gripper_cmd = 0xFF; // idle task
uint8_t dir = 0; // diretion of movement in SimleMove command [0-forward, 1-backward]
uint16_t move_delay = 800;

uint8_t last_cmd_status = 1; // 1 - успех, 0 - авария/отмена

extern uint8_t stepper1_calibrate;
extern int16_t stepper1_goal_position;


/**
 * @brief Rotate two Dynamixels by a given angle synchronously.
 * @param usart USART bus of the Dynamixels
 * @param id_1 Dynamixel ID 1
 * @param id_2 Dynamixel ID 2
 * @param angle_1 Goal angle in deg (0-300) for Dynamixel_1
 * @param angle_2 Goal angle in deg (0-300) for Dynamixel_2
 */
static void RotateTwoDynamixels(USART_TypeDef* usart, uint8_t id_1, uint8_t id_2, uint16_t angle_1, uint16_t angle_2)
{
    // Dynamixels ID
    uint8_t device_ids[] = {id_1, id_2};
    uint8_t num_devices = 2;
    //uint8_t num_devices = sizeof(device_ids) / sizeof(device_ids[0]); // universal counter of devices

    // Transform angle from deg to Goal Position Register (0-1023)
    if (angle_1 > 300) angle_1 = 300; // max angle limit
    if (angle_2 > 300) angle_2 = 300;
    uint16_t goal_position_value_1 = (uint16_t)(((float)angle_1 * 1023.0f) / 300.0f);
    uint16_t goal_position_value_2 = (uint16_t)(((float)angle_2 * 1023.0f) / 300.0f);

    // Prepare data for SyncWrite
    // 2 bytes for each servo (Low byte (LB), then High byte (HB))
    uint8_t data_to_write[4];

    // Data for 1st Dynamixel
    data_to_write[0] = (uint8_t)(goal_position_value_1 & 0xFF); // LB
    data_to_write[1] = (uint8_t)(goal_position_value_1 >> 8);  // HB

    // Data for 2nd Dynamixel
    data_to_write[2] = (uint8_t)(goal_position_value_2 & 0xFF); // LB
    data_to_write[3] = (uint8_t)(goal_position_value_2 >> 8);  // HB

    // Call the SyncWrite command
    DynamixelMaster.SyncWrite(usart, device_ids, num_devices, AX_GoalPosition, data_to_write, 2, NULL);
}


static void Gripper_SetPumps(const GripperHardware_t* hw, uint8_t mask) {
    // turn on all pumps
    SPI_GPO_SetOutputPin(hw->pumps_all_mask);
    
    // turn off pumps according to the mask (0 - turn off, 1 - keep on)
    for (int i = 0; i < 4; i++) {
        if ((mask & (1 << i)) == 0) {
            SPI_GPO_ResetOutputPin(hw->pumps_pins[i]);
            vTaskDelay(50);
        }
    }
}

static void Gripper_SetValves(const GripperHardware_t* hw, uint8_t mask) {
    SPI_GPO_SetOutputPin(hw->valves_all_mask);
    for (int i = 0; i < 4; i++) {
        if ((mask & (1 << i)) == 0) {
            SPI_GPO_ResetOutputPin(hw->valves_pins[i]);
            vTaskDelay(50);
        }
    }
}

/**
 * @brief Ожидание достижения угла динамикселем с защитой от зависания (тайм-аутом)
 * @param id ID динамикселя
 * @param target_angle Целевой угол
 * @param timeout_ms Максимальное время ожидания в миллисекундах
 * @return 1 если угол достигнут, 0 если сработал таймаут
 */
static uint8_t WaitDynamixelAngle(uint8_t id, uint8_t target_angle, uint32_t timeout_ms) {
    uint16_t current_angle = 0;
    int16_t diff = 0;
    uint32_t start_time = xTaskGetTickCount(); // Засекаем время старта (FreeRTOS)
    
    while (1) {
        Dynamixel_AX.GetAngle(USART3, id, &current_angle);
        diff = (int16_t)current_angle - (int16_t)target_angle;
        if (diff < 0) diff = -diff;
        
        // Угол достигнут с учетом допуска
        if (diff < tol_window) {
            return 1; 
        }
        
        // Проверка таймаута
        if ((xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS >= timeout_ms) {
            return 0; // Механизм заклинило или он не успел, выходим из цикла
        }
        
        vTaskDelay(20);
    }
}

/**
 * @brief Ожидание достижения позиции stepper'а с защитой от зависания (тайм-аутом)
 * @param target_pos Целевая позиция
 * @param timeout_ms Максимальное время ожидания в миллисекундах
 * @return 1 если позиция достигнута, 0 если сработал таймаут
 */
static uint8_t WaitStepper1Angle(int16_t target_pos, uint32_t timeout_ms) {
    int16_t current_pos = 0;
    uint32_t start_time = xTaskGetTickCount(); // Засекаем время старта (FreeRTOS)
    
    while (1) {
        current_pos = Stepper.GetPresentPosisition(STEPPER_MOTOR_1);
        // Позиция достигнута
        if (current_pos == target_pos) {
            return 1; 
        }
        // Проверка таймаута
        if ((xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS >= timeout_ms) {
            return 0; // Механизм заклинило или он не успел, выходим из цикла
        }
        
        vTaskDelay(20);
    }
}


static void Task(void *arg)
{
  	vTaskDelay(50);

	/*
	ValveDisable(1);
	ValveDisable(2);
	ValveDisable(3);
	ValveDisable(4);
	*/

	
	// Set custom id to single dynamixel (DISCONNECT ANY OTHER DYNAMIXELS!)
	/*
	uint8_t dynamixel_id = 0xFE;
	uint8_t dynamixel_new_id = 8;
	while(dynamixel_id > 0)
	{
		dynamixel_id --;
		if(Dynamixel_AX.Ping(USART3, dynamixel_id) == 0)
		{
			Dynamixel_AX.Set_ID(USART3, dynamixel_id, dynamixel_new_id);
			break;
		}
	}
	Dynamixel_AX.TorqueEnable(USART3, dynamixel_new_id);
	Dynamixel_AX.SetAngle(USART3, dynamixel_new_id, 0);
	vTaskDelay(1000);
	Dynamixel_AX.SetAngle(USART3, dynamixel_new_id, 180);
	*/
	

	/*
	uint8_t maxon_id = 0xFE;
	
	// Set custom id to single maxon (DISCONNECT ANY OTHER maxons!)
	
	while(maxon_id > 0)
	{
		maxon_id --;
		if(Maxon.Ping(USART6, maxon_id) == 0)
		{
			Maxon.Set_ID(USART6, maxon_id, 0x22);
			break;
		}
	}
	*/

	//Maxon.Set_ID(USART6, 0x21, 0x22);

	#ifdef GRIPPER_INIT
	
        MaxonsDisable();
        Kinematics.MatrixInit();
        MaxonsEnable();
        vTaskDelay(100);

        struct speed_s speed;
        speed.dphi = 0;
        speed.dx = 0;
        speed.dy = 0;
        Odometry.SetSpeed(&speed);
        vTaskDelay(100);

        // cursor init
        Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_cursor_id, cursor_speed);
        Dynamixel_AX.TorqueEnable(USART3, dynamixel_cursor_id);
        Dynamixel_AX.SetAngle(USART3, dynamixel_cursor_id, cursor_vert_angle);
        vTaskDelay(100);

        stepper1_calibrate = 1; // set the stepper into zero position

        // Инициализация обоих грипперов в цикле
        for (int i = 0; i < 2; i++) {
            // Берем указатель на конфигурацию текущего гриппера
            const GripperHardware_t* hw = &grippers[i];

            // Настройка скоростей динамикселей
            Dynamixel_AX.SetMovingSpeed(USART3, hw->dyn_main_1, hw->dyn_speed);
            Dynamixel_AX.SetMovingSpeed(USART3, hw->dyn_main_2, hw->dyn_speed);
            Dynamixel_AX.SetMovingSpeed(USART3, hw->dyn_side_1, hw->dyn_side_speed);
            Dynamixel_AX.SetMovingSpeed(USART3, hw->dyn_side_2, hw->dyn_side_speed);

            // Включение удержания (Torque Enable)
            Dynamixel_AX.TorqueEnable(USART3, hw->dyn_main_1);
            Dynamixel_AX.TorqueEnable(USART3, hw->dyn_main_2);
            Dynamixel_AX.TorqueEnable(USART3, hw->dyn_side_1);
            Dynamixel_AX.TorqueEnable(USART3, hw->dyn_side_2);

            // Сводим боковые захваты в начальное положение
            RotateTwoDynamixels(USART3, hw->dyn_side_1, hw->dyn_side_2, hw->side_on_angle_1, hw->side_on_angle_2);
            vTaskDelay(500);

            // Инициализация сервоприводов блоков (каналы 1-4 или 8-11)
            for (int j = 0; j < 4; j++) {
                PCA9685.SetServoAngleWithAddr(i2c_addr_1, hw->block_servos[j], 0);
                vTaskDelay(200);
            }

            // Инициализация сервоприводов сдвига
            PCA9685.SetServoAngleWithAddr(i2c_addr_1, hw->shift_servo_5, hw->shift_servo_5_start_angle);
            vTaskDelay(500);
            PCA9685.SetServoAngleWithAddr(i2c_addr_1, hw->shift_servo_6, hw->shift_servo_6_start_angle);
            vTaskDelay(500);

            // Выключаем все насосы и клапаны этого гриппера
            SPI_GPO_ResetOutputPin(hw->pumps_all_mask | hw->valves_all_mask);
            vTaskDelay(100);

            // Поднимаем основной механизм в стартовое положение
            //RotateTwoDynamixels(USART3, hw->dyn_main_1, hw->dyn_main_2, hw->end_angle, hw->end_angle);
			RotateTwoDynamixels(USART3, hw->dyn_main_1, hw->dyn_main_2, 65, 65);
            vTaskDelay(200);
        }
    #endif

	while(1)
	{
		vTaskDelay(100);

		#ifdef GRIPPER_INIT
			vTaskDelay(50);

			// Skip the cycle if there are no command to execute
			if (gripper_cmd == 0xFF) {
				continue; 
			}

			const GripperHardware_t* active_hw = &grippers[gripper_id - 1];

			switch(gripper_cmd){
			case 0: // toggle cursor
			{
				//cmd_isFinished = 0;
				
			  	Dynamixel_AX.SetMovingSpeed(USART3, dynamixel_cursor_id, cursor_speed);
  				Dynamixel_AX.TorqueEnable(USART3, dynamixel_cursor_id);

				// switch the state of the cursor
				uint8_t cursor_target_angle = (cursor_cmd == 1)?cursor_hor_angle:cursor_vert_angle;

				Dynamixel_AX.SetAngle(USART3, dynamixel_cursor_id, cursor_target_angle);
				vTaskDelay(100);
				
				// checking the achievement of a set angle
				Dynamixel_AX.GetAngle(USART3, dynamixel_cursor_id, &current_angle);
				diff = (int16_t)current_angle - (int16_t)cursor_target_angle;
				if (diff < 0) diff = -diff;
				
				while(diff >= tol_window){
					vTaskDelay(20);
					Dynamixel_AX.GetAngle(USART3, dynamixel_cursor_id, &current_angle);
					diff = (int16_t)current_angle - (int16_t)cursor_target_angle;
					if (diff < 0) diff = -diff;
				}

				// send the "wake up!" notification to TerminalTask
				if (xTerminalTaskHandle != NULL) {
					xTaskNotifyGive(xTerminalTaskHandle);
				}
				gripper_cmd = 0xFF; // go to idle task
				break;
			}

			case 1: // SideGrips
			{
                /*
				// Определяем целевой угол для боковых створок из структуры
                uint16_t target_angle_1 = (grips_cmd == 1) ? active_hw->side_on_angle_1 : active_hw->side_off_angle_1;
                uint16_t target_angle_2 = (grips_cmd == 1) ? active_hw->side_on_angle_2 : active_hw->side_off_angle_2;

                // Настраиваем скорость и включаем удержание
                Dynamixel_AX.SetMovingSpeed(USART3, active_hw->dyn_side_1, active_hw->dyn_side_speed);
                Dynamixel_AX.SetMovingSpeed(USART3, active_hw->dyn_side_2, active_hw->dyn_side_speed);
                Dynamixel_AX.TorqueEnable(USART3, active_hw->dyn_side_1);
                Dynamixel_AX.TorqueEnable(USART3, active_hw->dyn_side_2);

                // Запускаем синхронный поворот (обе створки получают одинаковый угол target_angle)
                RotateTwoDynamixels(USART3, active_hw->dyn_side_1, active_hw->dyn_side_2, target_angle_1, target_angle_2);
                
                vTaskDelay(200); 

                // Ждем достижения угла с таймаутом
                WaitDynamixelAngle(active_hw->dyn_side_1, target_angle_1, angle_timeout_ms);

                // Будим терминал
                if (xTerminalTaskHandle != NULL) {
                    xTaskNotifyGive(xTerminalTaskHandle);
                }
                gripper_cmd = NO_CMD;
                break;
                */


                uint16_t target_angle_1 = (grips_cmd == 1) ? active_hw->side_on_angle_1 : active_hw->side_off_angle_1;
                uint16_t target_angle_2 = (grips_cmd == 1) ? active_hw->side_on_angle_2 : active_hw->side_off_angle_2;

                Dynamixel_AX.SetMovingSpeed(USART3, active_hw->dyn_side_1, active_hw->dyn_side_speed);
                Dynamixel_AX.SetMovingSpeed(USART3, active_hw->dyn_side_2, active_hw->dyn_side_speed);
                Dynamixel_AX.TorqueEnable(USART3, active_hw->dyn_side_1);
                Dynamixel_AX.TorqueEnable(USART3, active_hw->dyn_side_2);

                if (grips_cmd == 1) 
                {
                    // === УМНОЕ СЖАТИЕ С ЦЕНТРИРОВАНИЕМ ===
                    
                    // 1. Начинаем движение к центру
                    RotateTwoDynamixels(USART3, active_hw->dyn_side_1, active_hw->dyn_side_2, target_angle_1, target_angle_2);
                    
                    // Даем сервам время тронуться с места (чтобы пропустить пик пускового тока/момента)
                    vTaskDelay(150); 

                    uint8_t done_1 = 0;
                    uint8_t done_2 = 0;
                    uint32_t start_time = xTaskGetTickCount();
                    
                    // Порог момента касания (от 0 до 1023). Подбери экспериментально на роботе!
                    int16_t LOAD_THRESHOLD = 150; 

                    // 2. Ждем касания обоих захватов
                    while (!done_1 || !done_2) 
                    {
                        // Защита от зависания по таймауту (используем твой angle_timeout_ms или жестко 1500)
                        if ((xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS > 1500) {
                            break;
                        }

                        // Проверяем 1-й захват
                        if (!done_1) {
                            int16_t load_1 = 0;
                            Dynamixel_AX.GetPresentLoad(USART3, active_hw->dyn_side_1, &load_1);
                            if (load_1 < 0) load_1 = -load_1; // Берем момент по модулю

                            uint16_t cur_angle_1 = 0;
                            Dynamixel_AX.GetAngle(USART3, active_hw->dyn_side_1, &cur_angle_1);
                            
                            int16_t diff_1 = (int16_t)cur_angle_1 - (int16_t)target_angle_1;
                            if (diff_1 < 0) diff_1 = -diff_1;

                            // Если коснулись блока ИЛИ дошли до конца вхолостую
                            if (load_1 > LOAD_THRESHOLD || diff_1 < tol_window) {
                                // Тормозим захват в текущей точке
                                Dynamixel_AX.SetAngle(USART3, active_hw->dyn_side_1, cur_angle_1);
                                done_1 = 1;
                            }
                        }

                        // Проверяем 2-й захват
                        if (!done_2) {
                            int16_t load_2 = 0;
                            Dynamixel_AX.GetPresentLoad(USART3, active_hw->dyn_side_2, &load_2);
                            if (load_2 < 0) load_2 = -load_2; // Берем момент по модулю

                            uint16_t cur_angle_2 = 0;
                            Dynamixel_AX.GetAngle(USART3, active_hw->dyn_side_2, &cur_angle_2);

                            int16_t diff_2 = (int16_t)cur_angle_2 - (int16_t)target_angle_2;
                            if (diff_2 < 0) diff_2 = -diff_2;

                            // Если коснулись блока ИЛИ дошли до конца вхолостую
                            if (load_2 > LOAD_THRESHOLD || diff_2 < tol_window) {
                                // Тормозим захват в текущей точке
                                Dynamixel_AX.SetAngle(USART3, active_hw->dyn_side_2, cur_angle_2);
                                done_2 = 1;
                            }
                        }

                        vTaskDelay(10); // Небольшая задержка, чтобы не забивать шину USART
                    }

                    // 3. Финальное дожатие и центрирование
                    // Оба коснулись (или вышли по таймауту). Теперь даем команду на полное сжатие
                    RotateTwoDynamixels(USART3, active_hw->dyn_side_1, active_hw->dyn_side_2, target_angle_1, target_angle_2);
                    vTaskDelay(300); // Даем время продавить блоки к центру
                } 
                else 
                {
                    // === ОБЫЧНОЕ РАЗЖАТИЕ ===
                    RotateTwoDynamixels(USART3, active_hw->dyn_side_1, active_hw->dyn_side_2, target_angle_1, target_angle_2);
                    vTaskDelay(200);
                    
                    // Ждем достижения угла (при разжатии ничего не мешает)
                    WaitDynamixelAngle(active_hw->dyn_side_1, target_angle_1, 1500);
                }

                // Будим терминал
                if (xTerminalTaskHandle != NULL) {
                    xTaskNotifyGive(xTerminalTaskHandle);
                }
                gripper_cmd = NO_CMD;
                break;
            }
			
			case 2: // gripper_lift
			{	
                uint16_t dyn_speed = 150;
				// Определяем целевой угол подъема из структуры
                if (lift_cmd == 0) {
                    gripper_target_angle = active_hw->end_angle;
                    dyn_speed = 150;
                } else if (lift_cmd == 1) {
                    gripper_target_angle = active_hw->start_angle;
                    dyn_speed = 150;
                } else if (lift_cmd == 2) {
                    gripper_target_angle = active_hw->cursor_angle;
                }

                // Настраиваем основные динамиксели подъема
                //Dynamixel_AX.SetMovingSpeed(USART3, active_hw->dyn_main_1, active_hw->dyn_speed);
                //Dynamixel_AX.SetMovingSpeed(USART3, active_hw->dyn_main_2, active_hw->dyn_speed);
                Dynamixel_AX.SetMovingSpeed(USART3, active_hw->dyn_main_1, dyn_speed);
                Dynamixel_AX.SetMovingSpeed(USART3, active_hw->dyn_main_2, dyn_speed);
                Dynamixel_AX.TorqueEnable(USART3, active_hw->dyn_main_1);
                Dynamixel_AX.TorqueEnable(USART3, active_hw->dyn_main_2);

                // Запускаем движение
                RotateTwoDynamixels(USART3, active_hw->dyn_main_1, active_hw->dyn_main_2, gripper_target_angle, gripper_target_angle);
                
                vTaskDelay(200);

                // Ждем с таймаутом
                WaitDynamixelAngle(active_hw->dyn_main_1, gripper_target_angle, angle_timeout_ms);

                // Будим терминал
                if (xTerminalTaskHandle != NULL) {
                    xTaskNotifyGive(xTerminalTaskHandle);
                }
                gripper_cmd = NO_CMD;
                break;
			} 

            /*case 2: // gripper_lift
            {   
                uint8_t lift_success = 1; 

                // Определяем целевой угол
                if (lift_cmd == 0) {
                    gripper_target_angle = active_hw->end_angle;   // Вверх
                } else if (lift_cmd == 1) {
                    gripper_target_angle = active_hw->start_angle; // Вниз
                } else if (lift_cmd == 2) {
                    gripper_target_angle = active_hw->cursor_angle; // Курсор
                }

                // Настраиваем моторы
                Dynamixel_AX.SetMovingSpeed(USART3, active_hw->dyn_main_1, active_hw->dyn_speed);
                Dynamixel_AX.SetMovingSpeed(USART3, active_hw->dyn_main_2, active_hw->dyn_speed);
                Dynamixel_AX.TorqueEnable(USART3, active_hw->dyn_main_1);
                Dynamixel_AX.TorqueEnable(USART3, active_hw->dyn_main_2);

                // Запускаем движение
                RotateTwoDynamixels(USART3, active_hw->dyn_main_1, active_hw->dyn_main_2, gripper_target_angle, gripper_target_angle);
                
                // УВЕЛИЧЕННАЯ ЗАДЕРЖКА: даем механизму 250 мс на разгон, 
                // чтобы полностью пропустить стартовый скачок тока
                vTaskDelay(250);

                if (lift_cmd == 1) 
                {
                    // === ОПУСКАНИЕ (С ЗАЩИТОЙ) ===
                    uint32_t start_time = xTaskGetTickCount();
                    int16_t LIFT_LOAD_THRESHOLD = 300; // Порог удара
                    uint8_t hit_counter = 0; // Счётчик для фильтрации помех
                    
                    while (1) 
                    {
                        // 1. Защита по таймауту
                        if ((xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS > angle_timeout_ms) {
                            break; 
                        }

                        // 2. Читаем нагрузку
                        int16_t current_load = 0;
                        GetPresentLoad(USART3, active_hw->dyn_main_1, &current_load);
                        
                        // Твоя функция уже переводит значения в формат от -1023 до +1023.
                        // Нам нужен только модуль силы:
                        if (current_load < 0) current_load = -current_load;

                        // 3. ПРОВЕРКА НА УДАР С ФИЛЬТРАЦИЕЙ
                        if (current_load > LIFT_LOAD_THRESHOLD) {
                            hit_counter++; // Нагрузка высока - копим счетчик
                        } else {
                            hit_counter = 0; // Нагрузка упала - это был случайный скачок, сбрасываем
                        }

                        // Если нагрузка стабильно высока 3 цикла подряд (~30 мс) - это точно препятствие
                        if (hit_counter >= 3) {
                            lift_success = 0; 
                            
                            // Спасаем гриппер - отправляем наверх
                            RotateTwoDynamixels(USART3, active_hw->dyn_main_1, active_hw->dyn_main_2, active_hw->end_angle, active_hw->end_angle);
                            WaitDynamixelAngle(active_hw->dyn_main_1, active_hw->end_angle, 1500);
                            break;
                        }

                        // 4. Проверяем, достигли ли низа
                        uint16_t cur_angle = 0;
                        Dynamixel_AX.GetAngle(USART3, active_hw->dyn_main_1, &cur_angle);
                        int16_t diff = (int16_t)cur_angle - (int16_t)gripper_target_angle;
                        if (diff < 0) diff = -diff;
                        
                        if (diff < tol_window) {
                            break; // Успешно доехали до низа
                        }

                        vTaskDelay(10); // Период опроса
                    }
                } 
                else 
                {
                    // === ОБЫЧНОЕ ДВИЖЕНИЕ ВВЕРХ ИЛИ КУРСОР ===
                    WaitDynamixelAngle(active_hw->dyn_main_1, gripper_target_angle, angle_timeout_ms);
                }

                // Будим терминал
                if (xTerminalTaskHandle != NULL) {
                    xTaskNotifyGive(xTerminalTaskHandle);
                }
                
                gripper_cmd = NO_CMD;
                break;
            }*/

			case 3: // toggle_pumps
			{
				Gripper_SetPumps(active_hw, pump_mask);
				vTaskDelay(100);
				if (xTerminalTaskHandle != NULL) xTaskNotifyGive(xTerminalTaskHandle);
				gripper_cmd = NO_CMD;
				break;
			}

			case 4: // toggle_valves
			{
				Gripper_SetValves(active_hw, valve_mask);
				vTaskDelay(100);
				if (xTerminalTaskHandle != NULL) xTaskNotifyGive(xTerminalTaskHandle);
				gripper_cmd = NO_CMD;
				break;
			}

			case 5: // rotate
			{
				Dynamixel_AX.SetMovingSpeed(USART3, active_hw->dyn_side_1, side_dynamixel_speed);
				Dynamixel_AX.SetMovingSpeed(USART3, active_hw->dyn_side_2, side_dynamixel_speed);

				// Разводим боковые захваты
				RotateTwoDynamixels(USART3, active_hw->dyn_side_1, active_hw->dyn_side_2, active_hw->side_off_angle_1, active_hw->side_off_angle_2);
				vTaskDelay(500);

				PCA9685.SetServoAngleWithAddr(i2c_addr_1, active_hw->shift_servo_5, active_hw->shift_servo_5_end_angle);
				vTaskDelay(500);
				PCA9685.SetServoAngleWithAddr(i2c_addr_1, active_hw->shift_servo_6, active_hw->shift_servo_6_end_angle);
				vTaskDelay(500);

				// Крутим блоки
				for (int i = 0; i < 4; i++) {
					servo_target_angle = ((block_mask & (1 << (3 - i))) == 0) ? 0 : 180;
					PCA9685.SetServoAngleWithAddr(i2c_addr_1, active_hw->block_servos[i], servo_target_angle); 
					vTaskDelay(350);
				}

				// Возвращаем на место
				PCA9685.SetServoAngleWithAddr(i2c_addr_1, active_hw->shift_servo_5, active_hw->shift_servo_5_start_angle);
				vTaskDelay(500);
				PCA9685.SetServoAngleWithAddr(i2c_addr_1, active_hw->shift_servo_6, active_hw->shift_servo_6_start_angle);
				vTaskDelay(500);

				gripper_cmd = NO_CMD;
				break;
			}
			
			case 6: // move
			{
				struct speed_s speed;
                speed.dphi = 0;
                speed.dx = 0;
                speed.dy = 0;

                float move_speed;
                
                if (dir == 0) {
                    move_speed = (float)(move_cmd_speed * 0.01f);
                } 
                else if (dir == 1) {
                    move_speed = (float)(move_cmd_speed * (-1) * 0.01f);
                }

                uint32_t delay_ms = (uint32_t)(move_cmd_time * 500.0f);

                // Расчет проекций скоростей на оси базы
                if (gripper_id == 1) {
                    // Гриппер 1 (смотрит под углом -60 градусов от оси X робота)
                    speed.dx = move_speed * cos60;
                    speed.dy = -move_speed * cos30;
                } 
                else if (gripper_id == 2) {
                    // Гриппер 2 (смотрит под углом +60 градусов)
                    speed.dx = move_speed * cos60;
                    speed.dy = move_speed * cos30;
                }

                Odometry.SetSpeed(&speed);
                vTaskDelay(delay_ms);
                

                // stop the robot after movement
                speed.dphi = 0;
                speed.dx = 0;
                speed.dy = 0;
                Odometry.SetSpeed(&speed);
                vTaskDelay(100);
                
                /*
				if (xTerminalTaskHandle != NULL) {
                    xTaskNotifyGive(xTerminalTaskHandle);
                }
                */
                gripper_cmd = 0xFF;
                break;
            }
                
            case 7: // gripper_vertical
			{	
                stepper1_goal_position = -1 * (int16_t)gripper_height * (int16_t)MM_TO_STEPS;
                Stepper.Enable(STEPPER_MOTOR_1);
                // Даем 10 мс на то, чтобы драйвер L6474 подал ток и жестко зафиксировал вал
                vTaskDelay(10); 
                Stepper.StepToPosisition(STEPPER_MOTOR_1, stepper1_goal_position, 32, 10, 300);
                
                WaitStepper1Angle(stepper1_goal_position, 2000); 
                Stepper.Disable(STEPPER_MOTOR_1);
                
                // (Альтернатива для п.5: если без питания гриппер падает вниз под своим весом, 
                // закомментируй Disable и вместо него аппаратно снизь ток:
                // L6474H.SetCurrentLimit_mA(STEPPER_MOTOR_1, 300); )

                // Будим терминал и отправляем ответ на HL
                if (xTerminalTaskHandle != NULL) {
                    xTaskNotifyGive(xTerminalTaskHandle);
                }
                gripper_cmd = NO_CMD;
                break;
			}
        }
		#endif
	}
	
}

static TaskHandle_t GetTaskHandle(void)
{
    return task_handle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    task_handle = xTaskCreateStatic(Task, "GripperTask", TASK_STACK_SIZE,
                              NULL, uxPriority, stack_buffer, &task_buffer);
}


const struct os_task_base_s GripperTask = { .CreateTask = CreateTask,
                                          .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
