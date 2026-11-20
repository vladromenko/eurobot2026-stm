/**
  ******************************************************************************
  * @file    	impeller.h
  * @brief   High-speed impeller (brushless fan "Mode 03, resistance speed").
  *          Speed signal: PWM8/PA3 driven through the RC_Servo API at the
  *          servo frequency (50 Hz, pulse 0..20000 us = duty 0..100 %)
  *          + external RC filter (10 kOhm, 47 uF) to the potentiometer wiper input.
  *          The only setting is the duty: wiper voltage = 3.3 V x duty
  *          (push-pull; IMPELLER_SPEED_OPEN_DRAIN 1 + external pull-up to +5 V for 0..5 V).
  *          On/off: GPIO5/PA8 open-drain wired straight to the button's
  *          signal contact (other contact = GND): LOW = pressed, Hi-Z = released.
  *
  *          TIM5 timing is not changed: PWM5..PWM7 still drive RC servos.
  *
  *          The button toggles the impeller controller and there is no
  *          feedback: the state below is only what the firmware believes.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IMPELLER_H
#define __IMPELLER_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "std_def.h"

#define IMPELLER_INVERT               0       /* 1 if the controller treats 0 V as maximum */
#define IMPELLER_SPEED_OPEN_DRAIN     0       /* 1: PA3 open-drain, needs an external pull-up to +5 V; 0: push-pull 0..3.3 V */
#define IMPELLER_PRESS_MS             200     /* button press length, NOT VERIFIED */
#define IMPELLER_SIGNAL_SETTLE_MS     2500    /* RC filter settles before a press: 5 x 10 kOhm x 47 uF */
#define IMPELLER_BOOT_DELAY_MS        1000    /* pause after the press that turns it on */
#define IMPELLER_DEFAULT_DUTY_PCT     75      /* duty after reset, used by TurnOn() until SetDuty(); it starts from 75 % */
#define IMPELLER_MAX_DUTY_PCT         100     /* limits every duty command; was 30 for the first tests */
#define IMPELLER_PWM_PERIOD_US        20000   /* RC_Servo period (50 Hz): pulse 0..20000 us = duty 0..100 % */
#define IMPELLER_DEBUG_PRESS_MAX_MS   2000    /* limit for Impeller.PressButton() */

enum impeller_state_e{
  IMPELLER_OFF,
  IMPELLER_STARTING,
  IMPELLER_ON,
  IMPELLER_STOPPING
};

struct impeller_s{
  void (*Init)(void);
  void (*Update)(uint32_t dt_ms);           // state machine step, called by ImpellerTask only
  void (*TurnOn)(void);                     // request: apply the duty, wait for the filter, press the button
  void (*TurnOff)(void);                    // request: duty 0, wait for the filter, press the button
  void (*SetDuty)(uint8_t percent);         // 0..100, limited by IMPELLER_MAX_DUTY_PCT; applied while STARTING/ON
  void (*PressButton)(uint16_t ms);         // debug: single press, does not change the state
  void (*SyncState)(uint8_t on);            // set the believed state to the real one, no press (in OFF/ON)
  enum impeller_state_e (*GetState)(void);
  uint8_t (*GetDuty)(void);                 // duty on the pin now, %
  uint8_t (*GetTargetDuty)(void);           // duty applied while STARTING/ON, %
};

extern const struct impeller_s Impeller;

#ifdef __cplusplus
}
#endif

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
