#include "cup_lift.h"
#include "os_tasks.h"
#include "robot_gpio_defines.h"
#include "stepper.h"
#include "stm32f4xx_ll_gpio.h"
#include "terminal_cmd_handlers.h"

#define LIFT_MOTOR           STEPPER_MOTOR_2
#define MICROSTEPS_PER_MM    (25 * 16)
#define EDGE_DIV             32
#define WORKING_DIV          8
#define RAMP_MS              300
#define MOVE_TIMEOUT_MS      10000
#define HOME_TIMEOUT_MS      30000
#define HOME_MAX_MM          150
#define LIFT_FAULTS          (L6474_OCD_MASK | L6474_TH_SD_MASK | L6474_UVLO_MASK)
#define TASK_STACK_SIZE      512

static TaskHandle_t task_handle;
static StackType_t stack_buffer[TASK_STACK_SIZE];
static StaticTask_t task_buffer;

/* Written by TerminalTask and consumed by CupLiftTask. */
static uint8_t requested_down;
static uint8_t requested_mm;
static volatile uint8_t requested;
static volatile uint8_t move_ok;
static volatile uint8_t ready;
static uint8_t homed;

static uint8_t BottomSwitchPressed(void)
{
    return LL_GPIO_IsInputPinSet(CUP_BOTTOM_SWITCH_PORT, CUP_BOTTOM_SWITCH_PIN);
}

static void SetBottomZero(void)
{
    Stepper.SetZeroPosisition(LIFT_MOTOR);
    homed = 1;
}

static uint8_t DriverEnabled(void)
{
    uint32_t status = L6474H.GetParam(LIFT_MOTOR, L6474_STATUS, L6474_STATUS_LENGTH);
    /* OCD, TH_SD and UVLO are active low. HiZ is active high. */
    return (status & (LIFT_FAULTS | L6474_HiZ_MASK)) == LIFT_FAULTS;
}

static uint8_t MoveTo(int32_t target, uint8_t down, uint32_t timeout_ms)
{
    if (down && BottomSwitchPressed()) {
        SetBottomZero();
        return 1;
    }
    if (Stepper.GetPresentPosisition(LIFT_MOTOR) == target) {
        return 1;
    }

    /* Clear the reset latch before enabling the L6474 power stage. */
    L6474H.GetStatus(LIFT_MOTOR);
    Stepper.Enable(LIFT_MOTOR);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t start_move = DriverEnabled();
    vTaskSuspendAll();
    start_move = start_move && COMM_ENABLE;
    if (start_move) {
        Stepper.StepToPosisition(LIFT_MOTOR, target, EDGE_DIV, WORKING_DIV, RAMP_MS);
    }
    xTaskResumeAll();

    uint8_t reached = 0;
    if (start_move) {
        TickType_t start = xTaskGetTickCount();
        while (COMM_ENABLE &&
               Stepper.GetPresentPosisition(LIFT_MOTOR) != target &&
               !(down && BottomSwitchPressed()) &&
               xTaskGetTickCount() - start < pdMS_TO_TICKS(timeout_ms)) {
            vTaskDelay(1);
        }
        Stepper.StopMotor(LIFT_MOTOR);
        if (DriverEnabled()) {
            reached = COMM_ENABLE && Stepper.GetPresentPosisition(LIFT_MOTOR) == target;
        } else {
            homed = 0;
        }
    } else {
        homed = 0;
    }
    Stepper.Disable(LIFT_MOTOR);

    if (down && BottomSwitchPressed()) {
        SetBottomZero();
        reached = COMM_ENABLE;
    }
    return reached;
}

static void Task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(100));
    while (!Stepper.TryLock(LIFT_MOTOR)) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Stepper.Init(LIFT_MOTOR);
    Stepper.SetZeroPosisition(LIFT_MOTOR);
    Stepper.Disable(LIFT_MOTOR);
    if (COMM_ENABLE) {
        MoveTo(-HOME_MAX_MM * MICROSTEPS_PER_MM, 1, HOME_TIMEOUT_MS);
    }
    Stepper.Unlock(LIFT_MOTOR);
    ready = 1;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (BottomSwitchPressed()) {
            SetBottomZero();
        }
        if (!requested) {
            continue;
        }

        move_ok = 0;
        if (COMM_ENABLE && Stepper.TryLock(LIFT_MOTOR)) {
            int32_t step = (int32_t)requested_mm * MICROSTEPS_PER_MM;
            int32_t target = Stepper.GetPresentPosisition(LIFT_MOTOR) + (requested_down ? -step : step);
            if (requested_down && homed && target < 0) {
                target = 0;
            }
            move_ok = MoveTo(target, requested_down, MOVE_TIMEOUT_MS);
            Stepper.Unlock(LIFT_MOTOR);
        }
        requested = 0;
    }
}

uint8_t CupLiftMove(uint8_t down, uint8_t travel_mm)
{
    if (!COMM_ENABLE || !ready || requested) {
        return 0;
    }
    requested_down = down;
    requested_mm = travel_mm;
    requested = 1;

    TickType_t start = xTaskGetTickCount();
    while (requested && xTaskGetTickCount() - start < pdMS_TO_TICKS(MOVE_TIMEOUT_MS + 1000)) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return !requested && move_ok;
}

static void CreateTask(UBaseType_t priority)
{
    task_handle = xTaskCreateStatic(Task, "CupLiftTask", TASK_STACK_SIZE,
                                   NULL, priority, stack_buffer, &task_buffer);
}

static TaskHandle_t GetTaskHandle(void)
{
    return task_handle;
}

const struct os_task_base_s CupLiftTask = {
    .CreateTask = CreateTask,
    .GetTaskHandle = GetTaskHandle
};
