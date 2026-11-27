/* Direct PWM input on the 2026 impeller controller. */
#include "impeller.h"
#include "robot_gpio_defines.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"

#define IMPELLER_PERIOD_US 1000U

static volatile enum impeller_state_e requested_state = IMPELLER_OFF;
static volatile enum impeller_state_e applied_state = IMPELLER_OFF;
static volatile uint8_t target_pct = 75;  /* Existing UART start-without-argument default. */
static volatile uint8_t applied_pct;

static void Init(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM9);
  /* APB2 runs at HCLK/2 on RESET, so TIM9 runs at HCLK (168 MHz). */
  LL_TIM_SetPrescaler(TIM9, SystemCoreClock / 1000000U - 1U);
  LL_TIM_SetAutoReload(TIM9, IMPELLER_PERIOD_US - 1U);
  LL_TIM_OC_SetMode(TIM9, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_EnablePreload(TIM9, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_SetCompareCH2(TIM9, 0);
  LL_TIM_GenerateEvent_UPDATE(TIM9);
  LL_TIM_CC_EnableChannel(TIM9, LL_TIM_CHANNEL_CH2);
  LL_TIM_EnableCounter(TIM9);

  /* Switch PA3 to the timer only after the timer is producing a steady low. */
  LL_GPIO_SetPinOutputType(IMPELLER_SPEED_GPIO_PORT, IMPELLER_SPEED_PWM8_PA3, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(IMPELLER_SPEED_GPIO_PORT, IMPELLER_SPEED_PWM8_PA3, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_0_7(IMPELLER_SPEED_GPIO_PORT, IMPELLER_SPEED_PWM8_PA3, LL_GPIO_AF_3);
  LL_GPIO_SetPinMode(IMPELLER_SPEED_GPIO_PORT, IMPELLER_SPEED_PWM8_PA3, LL_GPIO_MODE_ALTERNATE);
}

static void Update(void)
{
  /* Publish one consistent command even if the game or terminal task preempts us. */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  applied_state = requested_state;
  applied_pct = (applied_state == IMPELLER_ON) ? target_pct : 0;
  LL_TIM_OC_SetCompareCH2(TIM9, IMPELLER_PERIOD_US * applied_pct / 100U);
  __set_PRIMASK(primask);
}

static void TurnOn(void)
{
  requested_state = IMPELLER_ON;
}

static void TurnOff(void)
{
  requested_state = IMPELLER_OFF;
}

static void SetDuty(uint8_t percent)
{
  target_pct = percent > 100 ? 100 : percent;
}

static enum impeller_state_e GetState(void)
{
  return applied_state;
}

static uint8_t GetDuty(void)
{
  return applied_pct;
}

static uint8_t GetTargetDuty(void)
{
  return target_pct;
}

const struct impeller_s Impeller = {
  .Init = Init,
  .Update = Update,
  .TurnOn = TurnOn,
  .TurnOff = TurnOff,
  .SetDuty = SetDuty,
  .GetState = GetState,
  .GetDuty = GetDuty,
  .GetTargetDuty = GetTargetDuty
};
