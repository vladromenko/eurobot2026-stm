/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         grail_gripper_task.c
  * @brief        Grail pick (PCA9685 CH13) and shelf (PCA9685 CH14) servo grippers,
  *               grail lift on Stepper 2.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "os_tasks.h"
#include "pca9685.h"
#include "stepper.h"
#include "robot_gpio_defines.h"
#include "stm32f4xx_ll_gpio.h"

/* Set by the terminal commands, cleared by the task */
volatile uint8_t grail_pick_toggle = 0;
volatile uint8_t grail_shelf_toggle = 0;
volatile uint8_t grail_lift_cmd = 0;     // GRAIL_LIFT_UP or GRAIL_LIFT_DOWN
volatile uint8_t grail_lift_step = 0;    // mm
volatile uint8_t grail_lift_status = 0;  // 1 - moved, 0 - driver fault, timeout or bottom switch


#define TASK_STACK_SIZE             512

/* PCA9685 on I2C2, pulse range 500..2500 us over 180 degrees is set in main.c */
#define GRAIL_PCA9685_ADDR          0x40
#define GRAIL_PICK_SERVO_CH         PCA9685_CH13
#define GRAIL_SHELF_SERVO_CH        PCA9685_CH14

#define GRAIL_PICK_CLOSED_ANGLE     0
#define GRAIL_PICK_OPEN_ANGLE       180
#define GRAIL_SHELF_CLOSED_ANGLE    0
#define GRAIL_SHELF_OPEN_ANGLE      180

/* The same as the gripper vertical on Stepper 1 (gripper_task.c): full step (StepperMotor_2_Init),
   25 steps per mm, 19200 Hz / 10 = 1920 steps/s. NOT CALIBRATED for the grail lift */
#define GRAIL_MM_TO_STEPS           25
#define GRAIL_NORMAL_CURRENT_MA     1100.0f
#define GRAIL_DOWN_CURRENT_MA       320.0f
#define GRAIL_STEPPER_EDGE_DIV      64
#define GRAIL_STEPPER_WORKING_DIV   10
#define GRAIL_STEPPER_RAMP_MS       200
#define GRAIL_LIFT_TIMEOUT_MS       4000    // the terminal handler waits a bit longer


static TaskHandle_t task_handle;
static StackType_t stack_buffer[TASK_STACK_SIZE];
static StaticTask_t task_buffer;


/* Set in main.c. If the PCA9685 did not answer at startup (e.g. powered after the MCU),
   it stays in SLEEP and outputs no PWM, so Init is repeated until it succeeds. */
extern volatile int PCA9685_InitStatus;

static uint8_t PCA9685_Ready(void)
{
    if (PCA9685_InitStatus != 0) {
        PCA9685_InitStatus = PCA9685.Init(GRAIL_PCA9685_ADDR, 50.0f, 500, 2500, 180);
    }
    return PCA9685_InitStatus == 0;
}

/* PCA9685 keeps the channel off after reset, so the first toggle turns the servo to the open angle.
   The angle changes only if the I2C write succeeds, so a failed toggle is repeated by the next one. */
static void ToggleServo(enum pca9685_channel_e channel, uint8_t closed_angle, uint8_t open_angle, uint8_t *angle)
{
    uint8_t new_angle = (*angle == open_angle) ? closed_angle : open_angle;

    if (PCA9685_Ready() && PCA9685.SetServoAngleWithAddr(GRAIL_PCA9685_ADDR, channel, new_angle) == 0) {
        *angle = new_angle;
    }
}

static uint8_t BottomSwitchPressed(void)
{
    return LL_GPIO_IsInputPinSet(GRAIL_BOTTOM_SWITCH_GPIO_PORT, GRAIL_BOTTOM_SWITCH_GPIO28_PC14);
}

/* As the gripper vertical: lowering uses a lower current, the driver is disabled after the move.
   Moving down also stops at the bottom switch. */
static uint8_t MoveLift(uint8_t direction, uint8_t step_mm)
{
    int32_t step = (int32_t)step_mm * GRAIL_MM_TO_STEPS;
    int32_t target_position = Stepper.GetPresentPosisition(STEPPER_MOTOR_2) + ((direction == GRAIL_LIFT_UP) ? step : -step);
    uint8_t moving_down = (direction == GRAIL_LIFT_DOWN);
    TickType_t start_time;
    uint8_t reached;

    if (step == 0 || (moving_down && BottomSwitchPressed())) {
        return 0;
    }

    L6474H.SetCurrentLimit_mA(STEPPER_MOTOR_2, moving_down ? GRAIL_DOWN_CURRENT_MA : GRAIL_NORMAL_CURRENT_MA);
    // Every L6474 reset latches UVLO: without GetStatus the ENABLE command is ignored
    L6474H.GetStatus(STEPPER_MOTOR_2);
    Stepper.Enable(STEPPER_MOTOR_2);
    vTaskDelay(10);
    Stepper.StepToPosisition(STEPPER_MOTOR_2, target_position, GRAIL_STEPPER_EDGE_DIV,
                             GRAIL_STEPPER_WORKING_DIV, GRAIL_STEPPER_RAMP_MS);

    start_time = xTaskGetTickCount();
    while (Stepper.GetPresentPosisition(STEPPER_MOTOR_2) != target_position &&
           !(moving_down && BottomSwitchPressed()) &&
           xTaskGetTickCount() - start_time < pdMS_TO_TICKS(GRAIL_LIFT_TIMEOUT_MS)) {
        vTaskDelay(1);
    }
    reached = (Stepper.GetPresentPosisition(STEPPER_MOTOR_2) == target_position);

    Stepper.StopMotor(STEPPER_MOTOR_2);
    Stepper.Disable(STEPPER_MOTOR_2);
    L6474H.SetCurrentLimit_mA(STEPPER_MOTOR_2, GRAIL_NORMAL_CURRENT_MA);
    return reached;
}

static void Task(void *arg)
{
    uint8_t pick_angle = GRAIL_PICK_CLOSED_ANGLE;
    uint8_t shelf_angle = GRAIL_SHELF_CLOSED_ANGLE;

    vTaskDelay(50);
    Stepper.Init(STEPPER_MOTOR_2);
    L6474H.GetStatus(STEPPER_MOTOR_2);
    Stepper.SetZeroPosisition(STEPPER_MOTOR_2);
    Stepper.Disable(STEPPER_MOTOR_2);

    while(1)
    {
        vTaskDelay(10);

        if (grail_lift_cmd) {
            // Busy while the gripper vertical (Stepper 1) is moving
            grail_lift_status = 0;
            if (Stepper.TryLock(STEPPER_MOTOR_2)) {
                grail_lift_status = MoveLift(grail_lift_cmd, grail_lift_step);
                Stepper.Unlock(STEPPER_MOTOR_2);
            }
            grail_lift_cmd = 0;
        }

        if (grail_pick_toggle) {
            grail_pick_toggle = 0;
            ToggleServo(GRAIL_PICK_SERVO_CH, GRAIL_PICK_CLOSED_ANGLE, GRAIL_PICK_OPEN_ANGLE, &pick_angle);
        }

        if (grail_shelf_toggle) {
            grail_shelf_toggle = 0;
            ToggleServo(GRAIL_SHELF_SERVO_CH, GRAIL_SHELF_CLOSED_ANGLE, GRAIL_SHELF_OPEN_ANGLE, &shelf_angle);
        }
    }
}

static TaskHandle_t GetTaskHandle(void)
{
    return task_handle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    task_handle = xTaskCreateStatic(Task, "GrailGripperTask", TASK_STACK_SIZE,
                              NULL, uxPriority, stack_buffer, &task_buffer);
}


const struct os_task_base_s GrailGripperTask = { .CreateTask = CreateTask,
                                          .GetTaskHandle = GetTaskHandle};




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
