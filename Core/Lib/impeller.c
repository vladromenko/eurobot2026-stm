/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         impeller.c
  * @brief        Impeller control: PWM duty through RC_Servo + RC filter,
  *               on/off by shorting the native button's signal contact to
  *               GND with PA8 (open-drain).
  *               No delays and no FreeRTOS here: ImpellerTask calls Update()
  *               with a fixed period, other tasks only post requests.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "impeller.h"
#include "rc_servo.h"
#include "stm32f4xx_ll_gpio.h"
#include "robot_gpio_defines.h"

enum impeller_cmd_e{
  IMPELLER_CMD_NONE,
  IMPELLER_CMD_ON,
  IMPELLER_CMD_OFF
};

enum impeller_phase_e{
  IMPELLER_PHASE_IDLE,
  IMPELLER_PHASE_SETTLE,
  IMPELLER_PHASE_PRESS,
  IMPELLER_PHASE_BOOT
};

/*  Requests, written by any task  */
static volatile uint8_t pending_cmd = IMPELLER_CMD_NONE;
static volatile uint8_t target_pct = IMPELLER_DEFAULT_DUTY_PCT;

/*  State, written by Update() only  */
static volatile enum impeller_state_e state = IMPELLER_OFF;
static volatile uint8_t duty_pct;             /* on the pin now */
static enum impeller_phase_e phase = IMPELLER_PHASE_IDLE;
static uint32_t phase_timer_ms;



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

static void PostCmd(enum impeller_cmd_e cmd)
{
  uint32_t primask = Lock();
  pending_cmd = cmd;
  Unlock(primask);
}

static enum impeller_cmd_e TakeCmd(void)
{
  uint32_t primask = Lock();
  enum impeller_cmd_e cmd = (enum impeller_cmd_e)pending_cmd;
  pending_cmd = IMPELLER_CMD_NONE;
  Unlock(primask);
  return cmd;
}

static void SetButton(uint8_t pressed)
{
  /*  Open-drain: LOW shorts the button to GND, HIGH = Hi-Z (released)  */
  if(pressed)
    LL_GPIO_ResetOutputPin(IMPELLER_BTN_GPIO_PORT, IMPELLER_BTN_GPIO5_PA8);
  else
    LL_GPIO_SetOutputPin(IMPELLER_BTN_GPIO_PORT, IMPELLER_BTN_GPIO5_PA8);
}

static void SetDuty_Pin(uint8_t percent)
{
#if IMPELLER_INVERT
  percent = 100 - percent;
#endif
  /*  "Angle" in % over a 0..100 range: pulse = percent of the period  */
  RC_Servo.SetServoAngle(IMPELLER_SPEED_RC_SERVO, 0, IMPELLER_PWM_PERIOD_US, 100, percent);
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

static void StartPhase(enum impeller_phase_e new_phase, uint32_t ms)
{
  phase = new_phase;
  phase_timer_ms = ms;
  SetButton(new_phase == IMPELLER_PHASE_PRESS);
}



static void Init(void)
{
  LL_GPIO_InitTypeDef GPIO_Struct;

  /*  ODR resets to 0 = pressed: release (Hi-Z) before the pin becomes an output  */
  LL_GPIO_SetOutputPin(IMPELLER_BTN_GPIO_PORT, IMPELLER_BTN_GPIO5_PA8);

  GPIO_Struct.Pin = IMPELLER_BTN_GPIO5_PA8;
	GPIO_Struct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_Struct.Pull = LL_GPIO_PULL_NO;
	GPIO_Struct.Speed = LL_GPIO_SPEED_FREQ_LOW;

	LL_GPIO_Init(IMPELLER_BTN_GPIO_PORT, &GPIO_Struct);

  /*  Servo frequency (50 Hz, 1 us steps), the group keeps serving RC servos  */
  RC_Servo.Enable(IMPELLER_SPEED_RC_SERVO);
#if IMPELLER_SPEED_OPEN_DRAIN
  /*  PWM.Enable() sets push-pull: high level comes from the 5 V pull-up instead  */
  LL_GPIO_SetPinOutputType(IMPELLER_SPEED_GPIO_PORT, IMPELLER_SPEED_PWM8_PA3, LL_GPIO_OUTPUT_OPENDRAIN);
#endif
  SetDuty_Pin(0);
}

static void Update(uint32_t dt_ms)
{
  enum impeller_cmd_e cmd;

  switch(state)
  {
    case IMPELLER_OFF:
    case IMPELLER_ON:
      /*  Requests are taken only here, so a press sequence is never interrupted  */
      cmd = TakeCmd();
      /*  On and off: the duty (or 0) is applied below, press once the RC filter has settled  */
      if((cmd == IMPELLER_CMD_ON) && (state == IMPELLER_OFF)){
        StartPhase(IMPELLER_PHASE_SETTLE, IMPELLER_SIGNAL_SETTLE_MS);
        state = IMPELLER_STARTING;
      }
      else if((cmd == IMPELLER_CMD_OFF) && (state == IMPELLER_ON)){
        StartPhase(IMPELLER_PHASE_SETTLE, IMPELLER_SIGNAL_SETTLE_MS);
        state = IMPELLER_STOPPING;
      }
      break;

    case IMPELLER_STARTING:
    case IMPELLER_STOPPING:
      if(!TimerElapsed(&phase_timer_ms, dt_ms))
        break;
      if(phase == IMPELLER_PHASE_SETTLE)
        StartPhase(IMPELLER_PHASE_PRESS, IMPELLER_PRESS_MS);
      else if((phase == IMPELLER_PHASE_PRESS) && (state == IMPELLER_STARTING))
        StartPhase(IMPELLER_PHASE_BOOT, IMPELLER_BOOT_DELAY_MS);
      else{
        StartPhase(IMPELLER_PHASE_IDLE, 0);
        state = (state == IMPELLER_STARTING) ? IMPELLER_ON : IMPELLER_OFF;
      }
      break;
  }

  /*  The duty is on the pin only while starting or on, otherwise 0  */
  duty_pct = ((state == IMPELLER_STARTING) || (state == IMPELLER_ON)) ? target_pct : 0;
  SetDuty_Pin(duty_pct);
}

static void TurnOn(void)
{
  PostCmd(IMPELLER_CMD_ON);
}

static void TurnOff(void)
{
  PostCmd(IMPELLER_CMD_OFF);
}

static void SetDuty(uint8_t percent)
{
  if(percent > IMPELLER_MAX_DUTY_PCT)
    percent = IMPELLER_MAX_DUTY_PCT;
  target_pct = percent;
}

static enum impeller_state_e GetState(void)
{
  return state;
}

static uint8_t GetDuty(void)
{
  return duty_pct;
}

static uint8_t GetTargetDuty(void)
{
  return target_pct;
}

const struct impeller_s Impeller = {.Init = Init,
                                    .Update = Update,
                                    .TurnOn = TurnOn,
                                    .TurnOff = TurnOff,
                                    .SetDuty = SetDuty,
                                    .GetState = GetState,
                                    .GetDuty = GetDuty,
                                    .GetTargetDuty = GetTargetDuty};

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
