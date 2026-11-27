#ifndef CUP_LIFT_H
#define CUP_LIFT_H

#include <stdint.h>

/* Relative travel, in mm. Returns 1 at the target or lower limit. */
uint8_t CupLiftMove(uint8_t down, uint8_t travel_mm);

#endif
