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

/* Set by the terminal commands, cleared by the task */
volatile uint8_t grail_pick_toggle = 0;
volatile uint8_t grail_shelf_toggle = 0;
volatile uint8_t grail_lift_cmd = 0;
volatile uint8_t grail_lift_height = 0;  // mm from the power-on position
volatile uint8_t grail_lift_status = 0;  // 1 - reached, 0 - driver fault or timeout


#define TASK_STACK_SIZE             512

/* PCA9685 on I2C2, pulse range 500..2500 us over 180 degrees is set in main.c */
#define GRAIL_PCA9685_ADDR          0x40
#define GRAIL_PICK_SERVO_CH         PCA9685_CH13
#define GRAIL_SHELF_SERVO_CH        PCA9685_CH14

#define GRAIL_PICK_CLOSED_ANGLE     0
#define GRAIL_PICK_OPEN_ANGLE       180
#define GRAIL_SHELF_CLOSED_ANGLE    0
#define GRAIL_SHELF_OPEN_ANGLE      180

/* Stepper 2 runs at 1/16 microstep (StepperMotor_2_Init): 25 full steps per mm
   as on the gripper lift, NOT CALIBRATED for the grail lift */
#define GRAIL_MM_TO_STEPS           (25 * 16)
#define GRAIL_STEPPER_EDGE_DIV      32
#define GRAIL_STEPPER_WORKING_DIV   8       // 19200 Hz / 8 = 2400 microsteps/s = 6 mm/s
#define GRAIL_STEPPER_RAMP_MS       300
#define GRAIL_LIFT_TIMEOUT_MS       10000   // the terminal handler waits a bit longer


static TaskHandle_t task_handle;
static StackType_t stack_buffer[TASK_STACK_SIZE];
static StaticTask_t task_buffer;


/* PCA9685 keeps the channel off after reset, so the first toggle turns the servo to the open angle.
   The angle changes only if the I2C write succeeds, so a failed toggle is repeated by the next one. */
static void ToggleServo(enum pca9685_channel_e channel, uint8_t closed_angle, uint8_t open_angle, uint8_t *angle)
{
    uint8_t new_angle = (*angle == open_angle) ? closed_angle : open_angle;

    if (PCA9685.SetServoAngleWithAddr(GRAIL_PCA9685_ADDR, channel, new_angle) == 0) {
        *angle = new_angle;
    }
}

static uint8_t StepperFault(void)
{
    return (L6474H.GetParam(STEPPER_MOTOR_2, L6474_STATUS, L6474_STATUS_LENGTH) & L6474_HiZ_MASK) != 0;
}

/* The driver keeps holding the lift after the move, it is disabled only on a failure */
static uint8_t MoveLift(uint8_t height_mm)
{
    int32_t target_position = (int32_t)height_mm * GRAIL_MM_TO_STEPS;
    TickType_t start_time;

    if (target_position == Stepper.GetPresentPosisition(STEPPER_MOTOR_2)) {
        return 1;
    }

    // Every L6474 reset latches UVLO: without GetStatus the ENABLE command is ignored
    L6474H.GetStatus(STEPPER_MOTOR_2);
    Stepper.Enable(STEPPER_MOTOR_2);
    vTaskDelay(10);
    if (StepperFault()) {
        Stepper.Disable(STEPPER_MOTOR_2);
        return 0;
    }

    Stepper.StepToPosisition(STEPPER_MOTOR_2, target_position, GRAIL_STEPPER_EDGE_DIV,
                             GRAIL_STEPPER_WORKING_DIV, GRAIL_STEPPER_RAMP_MS);

    start_time = xTaskGetTickCount();
    while (Stepper.GetPresentPosisition(STEPPER_MOTOR_2) != target_position) {
        if (xTaskGetTickCount() - start_time >= pdMS_TO_TICKS(GRAIL_LIFT_TIMEOUT_MS)) {
            break;
        }
        vTaskDelay(10);
    }
    Stepper.StopMotor(STEPPER_MOTOR_2);

    // Overcurrent/thermal shutdown puts the bridge in HiZ while STEP pulses are still counted
    if (Stepper.GetPresentPosisition(STEPPER_MOTOR_2) != target_position || StepperFault()) {
        Stepper.Disable(STEPPER_MOTOR_2);
        return 0;
    }
    return 1;
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
            grail_lift_status = MoveLift(grail_lift_height);
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
