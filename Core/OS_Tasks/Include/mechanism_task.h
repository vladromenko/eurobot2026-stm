#ifndef __MECHANISM_TASK_H
#define __MECHANISM_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std_def.h"

enum mechanism_state_flags_e {
    MECHANISM_FLAG_INITIALIZED = (1U << 0),
    MECHANISM_FLAG_STEPPER_BUSY = (1U << 1),
    MECHANISM_FLAG_PWM5_ENABLED = (1U << 2),
    MECHANISM_FLAG_PWM6_ENABLED = (1U << 3),
    MECHANISM_FLAG_SERVOS_ENABLED = MECHANISM_FLAG_PWM5_ENABLED | MECHANISM_FLAG_PWM6_ENABLED
};

enum mechanism_servo_mask_e {
    MECHANISM_SERVO_PWM5 = (1U << 0),
    MECHANISM_SERVO_PWM6 = (1U << 1),
    MECHANISM_SERVO_BOTH = MECHANISM_SERVO_PWM5 | MECHANISM_SERVO_PWM6
};

struct mechanism_state_s {
    int32_t current_position;
    int32_t goal_position;
    uint8_t left_angle;
    uint8_t right_angle;
    uint8_t left_target_angle;
    uint8_t right_target_angle;
    uint8_t stepper_speed_level;
    uint8_t servo_speed_level;
    uint8_t flags;
};

uint8_t MechanismSetStepperPosition(int32_t position);
uint8_t MechanismSetServoAngles(uint8_t left_angle, uint8_t right_angle, uint8_t servo_mask);
uint8_t MechanismSetSpeedLevels(uint8_t stepper_level, uint8_t servo_level);
uint8_t MechanismSetStepperZero(void);
uint8_t MechanismStop(void);
void MechanismEmergencyStop(void);
void MechanismGetState(struct mechanism_state_s *state);

#ifdef __cplusplus
}
#endif

#endif
