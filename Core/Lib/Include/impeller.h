/* RESET 2026: PWM8/PA3, TIM9_CH2, active-high 3.3 V PWM at 1 kHz. */
#ifndef __IMPELLER_H
#define __IMPELLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std_def.h"

/* Values are part of the four-byte UART status reply. */
enum impeller_state_e {
  IMPELLER_OFF = 0,
  IMPELLER_ON = 2
};

struct impeller_s {
  void (*Init)(void);
  void (*Update)(void);
  void (*TurnOn)(void);
  void (*TurnOff)(void);
  void (*SetDuty)(uint8_t percent);
  enum impeller_state_e (*GetState)(void);
  uint8_t (*GetDuty)(void);
  uint8_t (*GetTargetDuty)(void);
};

extern const struct impeller_s Impeller;

#ifdef __cplusplus
}
#endif
#endif
