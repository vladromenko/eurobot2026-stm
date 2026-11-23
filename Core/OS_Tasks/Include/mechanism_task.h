#ifndef __MECHANISM_TASK_H
#define __MECHANISM_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std_def.h"

enum mechanism_state_flags_e {
    MECHANISM_FLAG_INITIALIZED = (1U << 0),
    MECHANISM_FLAG_STEPPER_BUSY = (1U << 1),
    MECHANISM_FLAG_SERVOS_ENABLED = (1U << 2)
};

struct mechanism_state_s {
    int32_t current_position;
    int32_t goal_position;
    uint8_t left_angle;
    uint8_t right_angle;
    uint8_t flags;
};

uint8_t MechanismSetStepperPosition(int32_t position);
uint8_t MechanismSetServoAngles(uint8_t left_angle, uint8_t right_angle);
uint8_t MechanismSetStepperZero(void);
uint8_t MechanismStop(void);
void MechanismEmergencyStop(void);
void MechanismGetState(struct mechanism_state_s *state);

#ifdef __cplusplus
}
#endif

#endif
