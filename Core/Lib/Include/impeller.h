/**
  ******************************************************************************
  * @file    	impeller.h
  * @brief   High-speed impeller (brushless fan "Mode 03, resistance speed").
  *          Speed signal: PWM8/PA3 (TIM5_CH4) at 20 kHz + external RC filter
  *          (10 kOhm, 1..10 uF) to the potentiometer wiper input.
  *          On/off: short press of the native button through a 2N7000
  *          on GPIO5/PA8 (HIGH = pressed).
  *
  *          After Impeller.Init() TIM5 runs with prescaler 0 at 20 kHz,
  *          so PWM5..PWM8 can no longer drive RC servos (RC_Servo 5..8).
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

/*  Calibration, NOT VERIFIED yet: replace with values from the ESP32 bench  */
#define IMPELLER_V_MIN_MV             500     /* signal at which the motor starts to spin */
#define IMPELLER_V_MAX_MV             3300    /* signal above which the speed does not grow */
#define IMPELLER_V_STOP_MV            0       /* signal for 0 % */
#define IMPELLER_INVERT               0       /* 1 if the controller treats 0 V as maximum */
#define IMPELLER_PRESS_MS             200     /* button press length */

#define IMPELLER_BOOT_DELAY_MS        1000    /* pause after the press before the speed is applied */
#define IMPELLER_STOP_DELAY_MS        300     /* pause at 0 % before the press that turns it off */
#define IMPELLER_MAX_SPEED_PCT        30      /* limits every speed command; 30 for the first tests */
#define IMPELLER_RAMP_PCT_PER_S       40      /* acceleration/deceleration */
#define IMPELLER_PWM_FREQ_HZ          20000
#define IMPELLER_PWM_VREF_MV          3300    /* PWM high level = 100 % duty */
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
  void (*TurnOn)(void);                     // request: press the button, then apply the speed
  void (*TurnOff)(void);                    // request: ramp to 0, then press the button
  void (*SetSpeed)(uint8_t percent);        // 0..100, limited by IMPELLER_MAX_SPEED_PCT
  void (*SetRawVolts_mV)(uint16_t mv);      // calibration: direct signal voltage, in OFF/ON only
  void (*PressButton)(uint16_t ms);         // debug: single press, does not change the state
  enum impeller_state_e (*GetState)(void);
  uint8_t (*GetSpeed)(void);                // current speed after the ramp, %
};

extern const struct impeller_s Impeller;

#ifdef __cplusplus
}
#endif

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
