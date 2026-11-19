/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         impeller.c
  * @brief        Impeller control: speed signal through PWM + RC filter,
  *               on/off through a 2N7000 parallel to the native button.
  *               No delays and no FreeRTOS here: ImpellerTask calls Update()
  *               with a fixed period, other tasks only post requests.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "impeller.h"
#include "pwm.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#include "robot_gpio_defines.h"

#define MPCT_PER_PCT                  1000    /* ramp is kept in 0.001 % steps */

enum impeller_cmd_e{
  IMPELLER_CMD_NONE,
  IMPELLER_CMD_ON,
  IMPELLER_CMD_OFF,
  IMPELLER_CMD_PRESS,
  IMPELLER_CMD_SYNC_ON,
  IMPELLER_CMD_SYNC_OFF
};

enum impeller_phase_e{
  IMPELLER_PHASE_IDLE,
  IMPELLER_PHASE_SIGNAL_SETTLE,
  IMPELLER_PHASE_PRESS,
  IMPELLER_PHASE_BOOT,
  IMPELLER_PHASE_RAMP_DOWN,
  IMPELLER_PHASE_SETTLE
};

/*  Requests, written by any task  */
static volatile uint8_t pending_cmd = IMPELLER_CMD_NONE;
static volatile uint16_t pending_arg;
static volatile uint8_t target_pct;
static volatile uint16_t calibration_mv = IMPELLER_CALIBRATION_MV;  /* signal while ON, 0: speed control */
static volatile uint8_t calibration_any_state;  /* set by SetRawVolts_mV(), cleared by TurnOff() */

/*  State, written by Update() only  */
static volatile enum impeller_state_e state = IMPELLER_OFF;
static volatile uint32_t current_mpct;
static enum impeller_phase_e phase = IMPELLER_PHASE_IDLE;
static uint32_t phase_timer_ms;
static uint32_t debug_press_ms;
static uint32_t pwm_period;                   /* ARR + 1 */



static uint32_t Lock(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void Unlock(uint32_t primask)
{
  __set_PRIMASK(primask);
}

static void PostCmd(enum impeller_cmd_e cmd, uint16_t arg)
{
  uint32_t primask = Lock();
  pending_arg = arg;
  pending_cmd = cmd;
  Unlock(primask);
}

static enum impeller_cmd_e TakeCmd(uint16_t* arg)
{
  uint32_t primask = Lock();
  enum impeller_cmd_e cmd = (enum impeller_cmd_e)pending_cmd;
  *arg = pending_arg;
  pending_cmd = IMPELLER_CMD_NONE;
  Unlock(primask);
  return cmd;
}

static void SetButton(uint8_t pressed)
{
  if(pressed)
    LL_GPIO_SetOutputPin(IMPELLER_BTN_GPIO_PORT, IMPELLER_BTN_GPIO5_PA8);
  else
    LL_GPIO_ResetOutputPin(IMPELLER_BTN_GPIO_PORT, IMPELLER_BTN_GPIO5_PA8);
}

static void SetSignal_mV(uint32_t mv)
{
  if(mv > IMPELLER_PWM_VREF_MV)
    mv = IMPELLER_PWM_VREF_MV;
#if IMPELLER_INVERT
  mv = IMPELLER_PWM_VREF_MV - mv;
#endif
  PWM.SetOutput_CC(IMPELLER_SPEED_PWM_OUTPUT, (mv*pwm_period)/IMPELLER_PWM_VREF_MV);
}

static uint32_t Speed_mV(uint32_t mpct)
{
  if(mpct == 0)
    return IMPELLER_V_STOP_MV;
  return IMPELLER_V_MIN_MV + ((IMPELLER_V_MAX_MV - IMPELLER_V_MIN_MV)*mpct)/(100*MPCT_PER_PCT);
}

static uint8_t TimerElapsed(uint32_t* timer_ms, uint32_t dt_ms)
{
  if(*timer_ms > dt_ms){
    *timer_ms -= dt_ms;
    return 0;
  }
  *timer_ms = 0;
  return 1;
}



static void Init(void)
{
  LL_GPIO_InitTypeDef GPIO_Struct;

  /*  Button released before the pin becomes an output (100 kOhm pull-down holds the gate until then)  */
  LL_GPIO_ResetOutputPin(IMPELLER_BTN_GPIO_PORT, IMPELLER_BTN_GPIO5_PA8);

  GPIO_Struct.Pin = IMPELLER_BTN_GPIO5_PA8;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_LOW;

	LL_GPIO_Init(IMPELLER_BTN_GPIO_PORT, &GPIO_Struct);

  /*  TIM5 tick 84 MHz instead of 1 MHz: ARR = 4199 at 20 kHz, 4200 duty steps.
   *  RC servos cannot use PWM5..PWM8 after this.  */
  LL_TIM_SetPrescaler(TIM5, 0);
  PWM.SetGroupFrequency(IMPELLER_SPEED_PWM_GROUP, IMPELLER_PWM_FREQ_HZ);
  pwm_period = PWM.GetGroupTimAutoreload(IMPELLER_SPEED_PWM_GROUP) + 1;

  SetSignal_mV(IMPELLER_V_STOP_MV);
  PWM.Enable(IMPELLER_SPEED_PWM_OUTPUT); // update event loads PSC, ARR and CCR
}

static void Update(uint32_t dt_ms)
{
  enum impeller_cmd_e cmd;
  uint16_t arg;

  if(debug_press_ms && TimerElapsed(&debug_press_ms, dt_ms))
    SetButton(0);

  switch(state)
  {
    case IMPELLER_OFF:
    case IMPELLER_ON:
      /*  Requests are taken only here, so a press sequence is never interrupted  */
      if(debug_press_ms)
        break;
      cmd = TakeCmd(&arg);
      if((cmd == IMPELLER_CMD_ON) && (state == IMPELLER_OFF)){
        current_mpct = 0;
        if(calibration_mv){
          /*  Calibration voltage is applied below; press once the RC filter has settled  */
          phase = IMPELLER_PHASE_SIGNAL_SETTLE;
          phase_timer_ms = IMPELLER_SIGNAL_SETTLE_MS;
        }
        else{
          phase = IMPELLER_PHASE_PRESS;
          phase_timer_ms = IMPELLER_PRESS_MS;
          SetButton(1);
        }
        state = IMPELLER_STARTING;
      }
      else if((cmd == IMPELLER_CMD_OFF) && (state == IMPELLER_ON)){
        phase = IMPELLER_PHASE_RAMP_DOWN;
        state = IMPELLER_STOPPING;
      }
      else if(cmd == IMPELLER_CMD_PRESS){
        debug_press_ms = arg;
        SetButton(1);
      }
      /*  The real state is known only to the user: no press, the ramp starts from 0 or goes to 0  */
      else if(cmd == IMPELLER_CMD_SYNC_ON){
        state = IMPELLER_ON;
      }
      else if(cmd == IMPELLER_CMD_SYNC_OFF){
        state = IMPELLER_OFF;
      }
      break;

    case IMPELLER_STARTING:
      if(!TimerElapsed(&phase_timer_ms, dt_ms))
        break;
      if(phase == IMPELLER_PHASE_SIGNAL_SETTLE){
        phase = IMPELLER_PHASE_PRESS;
        phase_timer_ms = IMPELLER_PRESS_MS;
        SetButton(1);
      }
      else if(phase == IMPELLER_PHASE_PRESS){
        SetButton(0);
        phase = IMPELLER_PHASE_BOOT;
        phase_timer_ms = IMPELLER_BOOT_DELAY_MS;
      }
      else{
        phase = IMPELLER_PHASE_IDLE;
        state = IMPELLER_ON;
      }
      break;

    case IMPELLER_STOPPING:
      if(phase == IMPELLER_PHASE_RAMP_DOWN){
        if(current_mpct == 0){
          phase = IMPELLER_PHASE_SETTLE;
          phase_timer_ms = IMPELLER_STOP_DELAY_MS;
        }
        break;
      }
      if(!TimerElapsed(&phase_timer_ms, dt_ms))
        break;
      if(phase == IMPELLER_PHASE_SETTLE){
        phase = IMPELLER_PHASE_PRESS;
        phase_timer_ms = IMPELLER_PRESS_MS;
        SetButton(1);
      }
      else{
        SetButton(0);
        phase = IMPELLER_PHASE_IDLE;
        state = IMPELLER_OFF;
      }
      break;
  }

  /*  Ramp: only ON follows the target, every other state goes to 0  */
  uint32_t goal_mpct = (state == IMPELLER_ON) ? (uint32_t)target_pct*MPCT_PER_PCT : 0;
  uint32_t step_mpct = IMPELLER_RAMP_PCT_PER_S*dt_ms;

  if(current_mpct < goal_mpct)
    current_mpct = ((goal_mpct - current_mpct) > step_mpct) ? (current_mpct + step_mpct) : goal_mpct;
  else
    current_mpct = ((current_mpct - goal_mpct) > step_mpct) ? (current_mpct - step_mpct) : goal_mpct;

  uint16_t signal_mv = calibration_mv;
  /*  A voltage set by command acts in any state (the impeller may be turned on by its own button)  */
  if((signal_mv != 0) && (calibration_any_state || (state == IMPELLER_STARTING) || (state == IMPELLER_ON)))
    SetSignal_mV(signal_mv);
  else
    SetSignal_mV(Speed_mV(current_mpct));
}

static void TurnOn(void)
{
  PostCmd(IMPELLER_CMD_ON, 0);
}

static void TurnOff(void)
{
  calibration_any_state = 0;
  PostCmd(IMPELLER_CMD_OFF, 0);
}

static void SetSpeed(uint8_t percent)
{
  if(percent > IMPELLER_MAX_SPEED_PCT)
    percent = IMPELLER_MAX_SPEED_PCT;
  target_pct = percent;
}

static uint8_t SetRawVolts_mV(uint16_t mv)
{
  /*  Same limit as the speed commands; raise IMPELLER_MAX_SPEED_PCT to calibrate V_MAX  */
  if(mv > Speed_mV(IMPELLER_MAX_SPEED_PCT*MPCT_PER_PCT))
    return 0;
  calibration_mv = mv;
  calibration_any_state = (mv != 0);
  return 1;
}

static uint16_t GetRawVolts_mV(void)
{
  return calibration_mv;
}

static void PressButton(uint16_t ms)
{
  if(ms == 0)
    return;
  if(ms > IMPELLER_DEBUG_PRESS_MAX_MS)
    ms = IMPELLER_DEBUG_PRESS_MAX_MS;
  PostCmd(IMPELLER_CMD_PRESS, ms);
}

static void SyncState(uint8_t on)
{
  PostCmd(on ? IMPELLER_CMD_SYNC_ON : IMPELLER_CMD_SYNC_OFF, 0);
}

static enum impeller_state_e GetState(void)
{
  return state;
}

static uint8_t GetSpeed(void)
{
  return (uint8_t)((current_mpct + MPCT_PER_PCT/2)/MPCT_PER_PCT);
}

const struct impeller_s Impeller = {.Init = Init,
                                    .Update = Update,
                                    .TurnOn = TurnOn,
                                    .TurnOff = TurnOff,
                                    .SetSpeed = SetSpeed,
                                    .SetRawVolts_mV = SetRawVolts_mV,
                                    .GetRawVolts_mV = GetRawVolts_mV,
                                    .PressButton = PressButton,
                                    .SyncState = SyncState,
                                    .GetState = GetState,
                                    .GetSpeed = GetSpeed};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
