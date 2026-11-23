#include "os_tasks.h"
#include "mechanism_task.h"
#include "rc_servo.h"
#include "stepper.h"

#define TASK_STACK_SIZE                 512
#define COMMAND_QUEUE_LENGTH            8
#define STEPPER_MIN_POSITION_STEPS      (-200000)
#define STEPPER_MAX_POSITION_STEPS      200000
#define STEPPER_EDGE_DIV                32
#define STEPPER_WORKING_DIV             16
#define STEPPER_ACCELERATION_MS         300
#define SERVO_MIN_PULSE_US              1000
#define SERVO_MAX_PULSE_US              2000
#define SERVO_ANGLE_RANGE               180

enum mechanism_command_type_e {
    MECHANISM_COMMAND_SET_POSITION,
    MECHANISM_COMMAND_SET_SERVOS,
    MECHANISM_COMMAND_SET_ZERO
};

struct mechanism_command_s {
    enum mechanism_command_type_e type;
    int32_t position;
    uint8_t left_angle;
    uint8_t right_angle;
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
    taskEXIT_CRITICAL();
}

static void ProcessCommand(const struct mechanism_command_s *command)
{
    if (command->type == MECHANISM_COMMAND_SET_POSITION) {
        Stepper.StopMotor(STEPPER_MOTOR_2);
        Stepper.Enable(STEPPER_MOTOR_2);
        Stepper.StepToPosisition(STEPPER_MOTOR_2, command->position,
                                STEPPER_EDGE_DIV, STEPPER_WORKING_DIV,
                                STEPPER_ACCELERATION_MS);

        taskENTER_CRITICAL();
        mechanism_state.goal_position = command->position;
        mechanism_state.flags |= MECHANISM_FLAG_STEPPER_BUSY;
        taskEXIT_CRITICAL();
    } else if (command->type == MECHANISM_COMMAND_SET_SERVOS) {
        RC_Servo.SetServoAngle(RC_SERVO_5, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US,
                               SERVO_ANGLE_RANGE, command->left_angle);
        RC_Servo.SetServoAngle(RC_SERVO_6, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US,
                               SERVO_ANGLE_RANGE, command->right_angle);
        RC_Servo.Enable(RC_SERVO_5);
        RC_Servo.Enable(RC_SERVO_6);

        taskENTER_CRITICAL();
        mechanism_state.left_angle = command->left_angle;
        mechanism_state.right_angle = command->right_angle;
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
    } else {
        StopOutputs();
    }
}

static uint8_t SendCommand(const struct mechanism_command_s *command)
{
    if (command_queue == NULL) {
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

    (void)arg;
    vTaskDelay(150);

    Stepper.Init(STEPPER_MOTOR_2);
    Stepper.SetZeroPosisition(STEPPER_MOTOR_2);
    Stepper.Disable(STEPPER_MOTOR_2);
    RC_Servo.Disable(RC_SERVO_5);
    RC_Servo.Disable(RC_SERVO_6);

    taskENTER_CRITICAL();
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
