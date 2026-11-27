#ifndef GRIPPER_ADDONS_H
#define GRIPPER_ADDONS_H

#include <stdint.h>

enum gripper_addon_e {
    GRIPPER_ADDON_IDLE,
    GRIPPER_ADDON_CUP_PICK,
    GRIPPER_ADDON_CUP_SHELF,
    GRIPPER_ADDON_AX_PAIR,
    GRIPPER_ADDON_PUMP_GROUP
};

uint8_t RunGripperAddon(enum gripper_addon_e command, uint8_t argument);

#endif
