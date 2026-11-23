#!/usr/bin/env python3
"""Run the actual impeller state machine on the host with simulated pins and PWM.

No serial ports or debug probes are opened. Requires a host C compiler (CC/cc).
"""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Core/Lib/impeller.c"

STUBS = r"""
#ifndef TEST_STUBS_H
#define TEST_STUBS_H
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>

static uint32_t primask, irq_disabled;
static uint32_t __get_PRIMASK(void) { return primask; }
static void __disable_irq(void) { primask = 1; ++irq_disabled; }
static void __set_PRIMASK(uint32_t value) { primask = value; }

typedef struct { int unused; } GPIO_TypeDef;
static GPIO_TypeDef gpioa;
#define GPIOA (&gpioa)
#define LL_GPIO_PIN_3 (1u << 3)
#define LL_GPIO_PIN_8 (1u << 8)
#define LL_GPIO_MODE_OUTPUT 1
#define LL_GPIO_OUTPUT_PUSHPULL 0
#define LL_GPIO_OUTPUT_OPENDRAIN 1
#define LL_GPIO_PULL_NO 0
#define LL_GPIO_SPEED_FREQ_LOW 0
typedef struct { uint32_t Pin, Mode, Speed, OutputType, Pull, Alternate; } LL_GPIO_InitTypeDef;

/* Open-drain to GND: ODR = 0 (reset value) pulls the button LOW = pressed. */
static int odr, button_is_output, button_released_before_output, presses;
#define button (button_is_output && !odr)
static void LL_GPIO_SetOutputPin(GPIO_TypeDef *port, uint32_t pin) {
    assert(port == GPIOA && pin == LL_GPIO_PIN_8);
    odr = 1;
}
static void LL_GPIO_ResetOutputPin(GPIO_TypeDef *port, uint32_t pin) {
    assert(port == GPIOA && pin == LL_GPIO_PIN_8);
    if (!button) ++presses;
    odr = 0;
}
static void LL_GPIO_Init(GPIO_TypeDef *port, LL_GPIO_InitTypeDef *init) {
    assert(port == GPIOA && init->Pin == LL_GPIO_PIN_8);
    assert(init->Mode == LL_GPIO_MODE_OUTPUT && init->OutputType == LL_GPIO_OUTPUT_OPENDRAIN);
    button_released_before_output = odr;
    button_is_output = 1;
}

/* TIM5 as set up by TIM5_PWM_Group5678_Init(): 1 MHz tick, 50 Hz. */
static uint32_t arr = 19999, cc, pwm_enabled, servo_freq_hz, speed_open_drain;
enum rc_servo_e { RC_SERVO_1, RC_SERVO_2, RC_SERVO_3, RC_SERVO_4,
                  RC_SERVO_5, RC_SERVO_6, RC_SERVO_7, RC_SERVO_8 };
/* Same arithmetic as rc_servo.c */
static void SetServoAngle(enum rc_servo_e servo, uint32_t pulse_min_us, uint32_t pulse_max_us, uint32_t angle_range, uint32_t angle) {
    assert(servo == RC_SERVO_8 && pwm_enabled && angle <= angle_range);
    uint32_t pulse_us = pulse_min_us + (angle * (pulse_max_us - pulse_min_us)) / angle_range;
    cc = (pulse_us * (arr + 1)) / 20000;
    assert(cc <= arr + 1);
}
/* PWM.Enable() puts the pin into push-pull alternate function */
static void Enable(enum rc_servo_e servo) { assert(servo == RC_SERVO_8); servo_freq_hz = 50; pwm_enabled = 1; speed_open_drain = 0; }
static const struct { void (*SetServoAngle)(enum rc_servo_e, uint32_t, uint32_t, uint32_t, uint32_t);
                      void (*Enable)(enum rc_servo_e); }
    RC_Servo = { SetServoAngle, Enable };
__attribute__((unused)) static void LL_GPIO_SetPinOutputType(GPIO_TypeDef *port, uint32_t pin, uint32_t type) {
    assert(port == GPIOA && pin == LL_GPIO_PIN_3 && pwm_enabled);
    speed_open_drain = (type == LL_GPIO_OUTPUT_OPENDRAIN);
}

#define IMPELLER_SPEED_RC_SERVO RC_SERVO_8
#define IMPELLER_SPEED_GPIO_PORT GPIOA
#define IMPELLER_SPEED_PWM8_PA3 LL_GPIO_PIN_3
#define IMPELLER_BTN_GPIO_PORT GPIOA
#define IMPELLER_BTN_GPIO5_PA8 LL_GPIO_PIN_8
#endif
"""

HARNESS = r"""
#include IMPELLER_SOURCE

#define DT 10
static uint32_t ms, button_ms, press_start_ms;
static int was_pressed;

static void run(uint32_t duration_ms) {
    for (uint32_t t = 0; t < duration_ms; t += DT) {
        Impeller.Update(DT);
        ms += DT;
        if (button && !was_pressed) { press_start_ms = ms; button_ms = 0; }
        was_pressed = button;
        if (button) button_ms += DT;
        assert(primask == 0);
        /* Never above the limit, and only while starting or on. */
        assert(cc <= IMPELLER_MAX_DUTY_PCT * (arr + 1) / 100);
        if (Impeller.GetState() == IMPELLER_OFF || Impeller.GetState() == IMPELLER_STOPPING)
            assert(cc == 0);
    }
}

/* Run until the state changes, return how long it took. */
static uint32_t run_until(enum impeller_state_e expected, uint32_t limit_ms) {
    uint32_t start = ms;
    while (Impeller.GetState() != expected) {
        assert(ms - start < limit_ms);
        run(DT);
    }
    return ms - start;
}

static uint32_t duty_cc(uint32_t pct) { return pct * (arr + 1) / 100; }

int main(void) {
    uint32_t t, start;

    /* Reset: button released before the pin becomes an output, PA3 open-drain, duty 0. */
    Impeller.Init();
    assert(button_is_output && button_released_before_output && !button);
    assert(servo_freq_hz == 50 && arr == 19999 && speed_open_drain == IMPELLER_SPEED_OPEN_DRAIN);
    run(5000);
    assert(Impeller.GetState() == IMPELLER_OFF && presses == 0 && cc == 0 && Impeller.GetDuty() == 0);
    printf("PASS: reset -> OFF, no press, %u steps at %u Hz, open-drain %u\n", arr + 1, servo_freq_hz, speed_open_drain);

    /* Default duty after reset: TurnOn without SetDuty uses it. */
    assert(Impeller.GetTargetDuty() == IMPELLER_DEFAULT_DUTY_PCT);
    Impeller.TurnOn();
    run(DT);
    assert(Impeller.GetState() == IMPELLER_STARTING && cc == duty_cc(IMPELLER_DEFAULT_DUTY_PCT));
    run_until(IMPELLER_ON, 10000);
    Impeller.TurnOff();
    run_until(IMPELLER_OFF, 10000);
    assert(presses == 2 && cc == 0);
    presses = 0;
    printf("PASS: default duty %u%% after reset\n", IMPELLER_DEFAULT_DUTY_PCT);

    /* Duty set in OFF stays off the pin. */
    Impeller.SetDuty(60);
    run(100);
    assert(cc == 0 && presses == 0);

    /* TurnOn: duty at once, press after the filter settled, boot pause, ON. */
    start = ms;
    Impeller.TurnOn();
    run(DT);
    assert(Impeller.GetState() == IMPELLER_STARTING && cc == duty_cc(60) && Impeller.GetDuty() == 60 && !button);
    t = run_until(IMPELLER_ON, 10000) + DT;
    assert(presses == 1 && button_ms == IMPELLER_PRESS_MS && !button);
    assert(press_start_ms - start == IMPELLER_SIGNAL_SETTLE_MS + DT);
    assert(t == IMPELLER_SIGNAL_SETTLE_MS + IMPELLER_PRESS_MS + IMPELLER_BOOT_DELAY_MS + DT);
    assert(cc == duty_cc(60));
    printf("PASS: TurnOn duty 60%% at once, press %u ms after it for %u ms, ON after %u ms\n",
        press_start_ms - start - DT, button_ms, t);

    /* In ON: repeated TurnOn ignored, the duty changes at once, limit applies. */
    Impeller.SetDuty(30);
    Impeller.TurnOn();
    run(DT);
    assert(cc == duty_cc(30) && Impeller.GetState() == IMPELLER_ON);
    Impeller.SetDuty(150);
    run(DT);
    assert(Impeller.GetDuty() == IMPELLER_MAX_DUTY_PCT);
    run(1000);
    assert(presses == 1);
    printf("PASS: duty change in ON without a press, 150%% -> %u%%\n", IMPELLER_MAX_DUTY_PCT);

    /* TurnOff: duty 0 at once, press after the filter discharged. */
    start = ms;
    Impeller.TurnOff();
    run(DT);
    assert(Impeller.GetState() == IMPELLER_STOPPING && cc == 0);
    t = run_until(IMPELLER_OFF, 10000) + DT;
    assert(presses == 2 && button_ms == IMPELLER_PRESS_MS && !button);
    assert(press_start_ms - start == IMPELLER_SIGNAL_SETTLE_MS + DT);
    assert(t == IMPELLER_SIGNAL_SETTLE_MS + IMPELLER_PRESS_MS + DT);
    Impeller.TurnOff();
    run(5000);
    assert(presses == 2 && Impeller.GetState() == IMPELLER_OFF);
    printf("PASS: TurnOff duty 0 at once, OFF after %u ms, TurnOff in OFF ignored\n", t);

    Impeller.SetDuty(40);

    /* TurnOff during STARTING: the start is finished, then it is turned off. */
    Impeller.TurnOn();
    run(DT);
    Impeller.TurnOff();
    run_until(IMPELLER_ON, 10000);
    run_until(IMPELLER_OFF, 10000);
    assert(presses == 4 && !button && cc == 0);
    printf("PASS: TurnOff during STARTING -> ON -> OFF, 2 presses\n");

    /* TurnOn during STOPPING. */
    Impeller.TurnOn();
    run_until(IMPELLER_ON, 10000);
    Impeller.TurnOff();
    run(DT);
    Impeller.TurnOn();
    run_until(IMPELLER_OFF, 10000);
    run_until(IMPELLER_ON, 10000);
    assert(presses == 7 && cc == duty_cc(40));
    printf("PASS: TurnOn during STOPPING -> OFF -> ON, presses=%u\n", presses);
    assert(irq_disabled > 0);
    return 0;
}
"""


def main():
    with tempfile.TemporaryDirectory(prefix="eurobot-impeller-") as temp:
        folder = Path(temp)
        # Supply only hardware declarations; compile impeller.c and impeller.h unmodified.
        for header in re.findall(r'^#include "([^"]+)"', SOURCE.read_text(), re.M) + ["std_def.h"]:
            if header != "impeller.h":
                (folder / header).write_text('#include "test_stubs.h"\n')
        (folder / "test_stubs.h").write_text(STUBS)
        (folder / "test.c").write_text(HARNESS)
        binary = folder / "test"
        subprocess.run([
            *shlex.split(os.environ.get("CC", "cc")), "-std=c99", "-O0",
            "-Wall", "-Wextra", "-I", str(folder), "-I", str(ROOT / "Core/Lib/Include"),
            f'-DIMPELLER_SOURCE="{SOURCE}"',
            str(folder / "test.c"), "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
