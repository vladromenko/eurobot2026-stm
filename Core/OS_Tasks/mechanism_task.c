#include "os_tasks.h"
#include "mechanism_task.h"
#include "rc_servo.h"
#include "stepper.h"

#define TASK_STACK_SIZE                 512
#define COMMAND_QUEUE_LENGTH            8
#define STEPPER_MIN_POSITION_STEPS      (-200000)
#define STEPPER_MAX_POSITION_STEPS      200000
#define STEPPER_EDGE_DIV                32
#define STEPPER_ACCELERATION_MS         300
#define SERVO_MIN_PULSE_US              1000
#define SERVO_MAX_PULSE_US              2000
#define SERVO_ANGLE_RANGE               180
#define SPEED_LEVEL_MIN                 1
#define SPEED_LEVEL_MAX                 8
#define DEFAULT_STEPPER_SPEED_LEVEL     1
#define DEFAULT_SERVO_SPEED_LEVEL       1
#define DEFAULT_SERVO_ANGLE             90

enum mechanism_command_type_e {
    MECHANISM_COMMAND_SET_POSITION,
    MECHANISM_COMMAND_SET_SERVOS,
    MECHANISM_COMMAND_SET_ZERO,
    MECHANISM_COMMAND_SET_SPEED
};

struct mechanism_command_s {
    enum mechanism_command_type_e type;
    int32_t position;
    uint8_t left_angle;
    uint8_t right_angle;
    uint8_t stepper_speed_level;
    uint8_t servo_speed_level;
};

static const uint8_t stepper_working_div[SPEED_LEVEL_MAX] = {
    32, 28, 24, 20, 16, 12, 10, 8
};

static const uint8_t servo_step_degrees[SPEED_LEVEL_MAX] = {
    1, 1, 1, 1, 1, 1, 2, 4
};

static const uint8_t servo_update_period_ms[SPEED_LEVEL_MAX] = {
    100, 80, 60, 40, 30, 20, 20, 20
};

static TaskHandle_t task_handle;
static StackType_t stack_buffer[TASK_STACK_SIZE];
static StaticTask_t task_buffer;
static StaticQueue_t command_queue_buffer;
static uint8_t command_queue_storage[COMMAND_QUEUE_LENGTH * sizeof(struct mechanism_command_s)];
static QueueHandle_t command_queue;
static struct mechanism_state_s mechanism_state;
static volatile uint8_t emergency_stop_requested;

static void StopOutputs(void)
{
    int32_t current_position;

    Stepper.StopMotor(STEPPER_MOTOR_2);
    Stepper.Disable(STEPPER_MOTOR_2);
    RC_Servo.Disable(RC_SERVO_5);
    RC_Servo.Disable(RC_SERVO_6);
    current_position = Stepper.GetPresentPosisition(STEPPER_MOTOR_2);

    taskENTER_CRITICAL();
    mechanism_state.flags &= (uint8_t)~(MECHANISM_FLAG_STEPPER_BUSY | MECHANISM_FLAG_SERVOS_ENABLED);
    mechanism_state.current_position = current_position;
    mechanism_state.goal_position = current_position;
    mechanism_state.left_target_angle = mechanism_state.left_angle;
    mechanism_state.right_target_angle = mechanism_state.right_angle;
    taskEXIT_CRITICAL();
}

static uint8_t MoveAngleToward(uint8_t current, uint8_t target, uint8_t step)
{
    if (current < target) {
        uint16_t next = (uint16_t)current + step;
        return (next < target) ? (uint8_t)next : target;
    }

    if (current > target) {
        return ((uint16_t)target + step < current) ? (uint8_t)(current - step) : target;
    }

    return current;
}

static void UpdateServos(TickType_t *last_update)
{
    TickType_t now = xTaskGetTickCount();
    uint8_t speed_index;
    uint8_t step;
    uint8_t left_angle;
    uint8_t right_angle;
    uint8_t left_target;
    uint8_t right_target;

    if ((mechanism_state.flags & MECHANISM_FLAG_SERVOS_ENABLED) == 0U) {
        *last_update = now;
        return;
    }

    speed_index = mechanism_state.servo_speed_level - SPEED_LEVEL_MIN;
    if ((now - *last_update) < pdMS_TO_TICKS(servo_update_period_ms[speed_index])) {
        return;
    }
    *last_update = now;

    step = servo_step_degrees[speed_index];
    left_target = mechanism_state.left_target_angle;
    right_target = mechanism_state.right_target_angle;
    left_angle = MoveAngleToward(mechanism_state.left_angle, left_target, step);
    right_angle = MoveAngleToward(mechanism_state.right_angle, right_target, step);

    if ((left_angle != mechanism_state.left_angle) ||
        (right_angle != mechanism_state.right_angle)) {
        RC_Servo.SetServoAngle(RC_SERVO_5, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US,
                               SERVO_ANGLE_RANGE, left_angle);
        RC_Servo.SetServoAngle(RC_SERVO_6, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US,
                               SERVO_ANGLE_RANGE, right_angle);

        taskENTER_CRITICAL();
        mechanism_state.left_angle = left_angle;
        mechanism_state.right_angle = right_angle;
        taskEXIT_CRITICAL();
    }
}

static void ProcessCommand(const struct mechanism_command_s *command)
{
    if (command->type == MECHANISM_COMMAND_SET_POSITION) {
        uint8_t speed_index = mechanism_state.stepper_speed_level - SPEED_LEVEL_MIN;
        Stepper.StopMotor(STEPPER_MOTOR_2);
        Stepper.Enable(STEPPER_MOTOR_2);
        Stepper.StepToPosisition(STEPPER_MOTOR_2, command->position,
                                STEPPER_EDGE_DIV, stepper_working_div[speed_index],
                                STEPPER_ACCELERATION_MS);

        taskENTER_CRITICAL();
        mechanism_state.goal_position = command->position;
        mechanism_state.flags |= MECHANISM_FLAG_STEPPER_BUSY;
        taskEXIT_CRITICAL();
    } else if (command->type == MECHANISM_COMMAND_SET_SERVOS) {
        if ((mechanism_state.flags & MECHANISM_FLAG_SERVOS_ENABLED) == 0U) {
            RC_Servo.SetServoAngle(RC_SERVO_5, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US,
                                   SERVO_ANGLE_RANGE, mechanism_state.left_angle);
            RC_Servo.SetServoAngle(RC_SERVO_6, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US,
                                   SERVO_ANGLE_RANGE, mechanism_state.right_angle);
            RC_Servo.Enable(RC_SERVO_5);
            RC_Servo.Enable(RC_SERVO_6);
        }

        taskENTER_CRITICAL();
        mechanism_state.left_target_angle = command->left_angle;
        mechanism_state.right_target_angle = command->right_angle;
        mechanism_state.flags |= MECHANISM_FLAG_SERVOS_ENABLED;
        taskEXIT_CRITICAL();
    } else if (command->type == MECHANISM_COMMAND_SET_ZERO) {
        if ((mechanism_state.flags & MECHANISM_FLAG_STEPPER_BUSY) == 0U) {
            Stepper.SetZeroPosisition(STEPPER_MOTOR_2);
            taskENTER_CRITICAL();
            mechanism_state.current_position = 0;
            mechanism_state.goal_position = 0;
            taskEXIT_CRITICAL();
        }
    } else if (command->type == MECHANISM_COMMAND_SET_SPEED) {
        taskENTER_CRITICAL();
        mechanism_state.stepper_speed_level = command->stepper_speed_level;
        mechanism_state.servo_speed_level = command->servo_speed_level;
        taskEXIT_CRITICAL();
    } else {
        StopOutputs();
    }
}

static uint8_t SendCommand(const struct mechanism_command_s *command)
{
    uint8_t initialized;

    if (command_queue == NULL) {
        return 0;
    }

    taskENTER_CRITICAL();
    initialized = mechanism_state.flags & MECHANISM_FLAG_INITIALIZED;
    taskEXIT_CRITICAL();

    if (initialized == 0U) {
        return 0;
    }

    return (xQueueSend(command_queue, command, 0) == pdPASS) ? 1U : 0U;
}

uint8_t MechanismSetStepperPosition(int32_t position)
{
    struct mechanism_command_s command = {0};

    if ((position < STEPPER_MIN_POSITION_STEPS) || (position > STEPPER_MAX_POSITION_STEPS)) {
        return 0;
    }

    command.type = MECHANISM_COMMAND_SET_POSITION;
    command.position = position;
    return SendCommand(&command);
}

uint8_t MechanismSetServoAngles(uint8_t left_angle, uint8_t right_angle)
{
    struct mechanism_command_s command = {0};

    if ((left_angle > SERVO_ANGLE_RANGE) || (right_angle > SERVO_ANGLE_RANGE)) {
        return 0;
    }

    command.type = MECHANISM_COMMAND_SET_SERVOS;
    command.left_angle = left_angle;
    command.right_angle = right_angle;
    return SendCommand(&command);
}

uint8_t MechanismSetSpeedLevels(uint8_t stepper_level, uint8_t servo_level)
{
    struct mechanism_command_s command = {0};

    if ((stepper_level < SPEED_LEVEL_MIN) || (stepper_level > SPEED_LEVEL_MAX) ||
        (servo_level < SPEED_LEVEL_MIN) || (servo_level > SPEED_LEVEL_MAX)) {
        return 0;
    }

    command.type = MECHANISM_COMMAND_SET_SPEED;
    command.stepper_speed_level = stepper_level;
    command.servo_speed_level = servo_level;
    return SendCommand(&command);
}

uint8_t MechanismSetStepperZero(void)
{
    struct mechanism_command_s command = {0};
    uint8_t stepper_busy;

    taskENTER_CRITICAL();
    stepper_busy = mechanism_state.flags & MECHANISM_FLAG_STEPPER_BUSY;
    taskEXIT_CRITICAL();

    if (stepper_busy != 0U) {
        return 0;
    }

    command.type = MECHANISM_COMMAND_SET_ZERO;
    return SendCommand(&command);
}

uint8_t MechanismStop(void)
{
    if (command_queue == NULL) {
        return 0;
    }

    MechanismEmergencyStop();
    return 1;
}

void MechanismEmergencyStop(void)
{
    emergency_stop_requested = 1U;
}

void MechanismGetState(struct mechanism_state_s *state)
{
    if (state != NULL) {
        taskENTER_CRITICAL();
        *state = mechanism_state;
        taskEXIT_CRITICAL();
    }
}

static void Task(void *arg)
{
    struct mechanism_command_s command;
    int32_t current_position;
    uint8_t position_reached;
    TickType_t last_servo_update;

    (void)arg;
    vTaskDelay(150);

    Stepper.Init(STEPPER_MOTOR_2);
    Stepper.SetZeroPosisition(STEPPER_MOTOR_2);
    Stepper.Disable(STEPPER_MOTOR_2);
    RC_Servo.Disable(RC_SERVO_5);
    RC_Servo.Disable(RC_SERVO_6);
    last_servo_update = xTaskGetTickCount();

    taskENTER_CRITICAL();
    mechanism_state.left_angle = DEFAULT_SERVO_ANGLE;
    mechanism_state.right_angle = DEFAULT_SERVO_ANGLE;
    mechanism_state.left_target_angle = DEFAULT_SERVO_ANGLE;
    mechanism_state.right_target_angle = DEFAULT_SERVO_ANGLE;
    mechanism_state.stepper_speed_level = DEFAULT_STEPPER_SPEED_LEVEL;
    mechanism_state.servo_speed_level = DEFAULT_SERVO_SPEED_LEVEL;
    mechanism_state.flags = MECHANISM_FLAG_INITIALIZED;
    taskEXIT_CRITICAL();

    while (1) {
        if (emergency_stop_requested != 0U) {
            emergency_stop_requested = 0U;
            xQueueReset(command_queue);
            StopOutputs();
        }

        if (xQueueReceive(command_queue, &command, pdMS_TO_TICKS(10)) == pdPASS) {
            ProcessCommand(&command);
        }

        current_position = Stepper.GetPresentPosisition(STEPPER_MOTOR_2);
        position_reached = 0U;
        taskENTER_CRITICAL();
        mechanism_state.current_position = current_position;
        if (((mechanism_state.flags & MECHANISM_FLAG_STEPPER_BUSY) != 0U) &&
            (current_position == mechanism_state.goal_position)) {
            mechanism_state.flags &= (uint8_t)~MECHANISM_FLAG_STEPPER_BUSY;
            position_reached = 1U;
        }
        taskEXIT_CRITICAL();

        if (position_reached != 0U) {
            Stepper.StopMotor(STEPPER_MOTOR_2);
            Stepper.Disable(STEPPER_MOTOR_2);
        }

        UpdateServos(&last_servo_update);
    }
}

static TaskHandle_t GetTaskHandle(void)
{
    return task_handle;
}

static void CreateTask(UBaseType_t uxPriority)
{
    command_queue = xQueueCreateStatic(COMMAND_QUEUE_LENGTH,
                                       sizeof(struct mechanism_command_s),
                                       command_queue_storage,
                                       &command_queue_buffer);
    task_handle = xTaskCreateStatic(Task, "MechanismTask", TASK_STACK_SIZE,
                                   NULL, uxPriority, stack_buffer, &task_buffer);
}

const struct os_task_base_s MechanismTask = {
    .CreateTask = CreateTask,
    .GetTaskHandle = GetTaskHandle
};
