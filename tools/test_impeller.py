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
typedef struct { int unused; } TIM_TypeDef;
static GPIO_TypeDef gpioa;
static TIM_TypeDef tim5;
#define GPIOA (&gpioa)
#define TIM5 (&tim5)
#define LL_GPIO_PIN_8 (1u << 8)
#define LL_GPIO_MODE_OUTPUT 1
#define LL_GPIO_OUTPUT_PUSHPULL 0
#define LL_GPIO_PULL_NO 0
#define LL_GPIO_SPEED_FREQ_LOW 0
typedef struct { uint32_t Pin, Mode, Speed, OutputType, Pull, Alternate; } LL_GPIO_InitTypeDef;

static int button, button_is_output, button_low_before_output, presses;
static void LL_GPIO_SetOutputPin(GPIO_TypeDef *port, uint32_t pin) {
    assert(port == GPIOA && pin == LL_GPIO_PIN_8);
    if (!button) ++presses;
    button = 1;
}
static void LL_GPIO_ResetOutputPin(GPIO_TypeDef *port, uint32_t pin) {
    assert(port == GPIOA && pin == LL_GPIO_PIN_8);
    button = 0;
}
static void LL_GPIO_Init(GPIO_TypeDef *port, LL_GPIO_InitTypeDef *init) {
    assert(port == GPIOA && init->Pin == LL_GPIO_PIN_8);
    assert(init->Mode == LL_GPIO_MODE_OUTPUT && init->OutputType == LL_GPIO_OUTPUT_PUSHPULL);
    button_low_before_output = !button;
    button_is_output = 1;
}

static uint32_t psc = 83, arr = 19999, cc, cc_max, pwm_enabled;
static void LL_TIM_SetPrescaler(TIM_TypeDef *tim, uint32_t value) { assert(tim == TIM5); psc = value; }

enum pwm_output_e { PWM_OUTPUT_1, PWM_OUTPUT_2, PWM_OUTPUT_3, PWM_OUTPUT_4,
                    PWM_OUTPUT_5, PWM_OUTPUT_6, PWM_OUTPUT_7, PWM_OUTPUT_8 };
enum pwm_group_e { PWM_GROUP_1234, PWM_GROUP_5678 };
static uint32_t GetGroupTimAutoreload(enum pwm_group_e group) { assert(group == PWM_GROUP_5678); return arr; }
static void SetOutput_CC(enum pwm_output_e output, uint32_t value) {
    assert(output == PWM_OUTPUT_8 && value <= arr + 1);
    cc = value;
    if (value > cc_max) cc_max = value;
}
static void SetGroupFrequency(enum pwm_group_e group, uint32_t hz) {
    assert(group == PWM_GROUP_5678);
    arr = 84000000 / (psc + 1) / hz - 1; /* same formula as pwm.c */
}
static void Enable(enum pwm_output_e output) { assert(output == PWM_OUTPUT_8); pwm_enabled = 1; }
static const struct { uint32_t (*GetGroupTimAutoreload)(enum pwm_group_e);
                      void (*SetOutput_CC)(enum pwm_output_e, uint32_t);
                      void (*SetGroupFrequency)(enum pwm_group_e, uint32_t);
                      void (*Enable)(enum pwm_output_e); }
    PWM = { GetGroupTimAutoreload, SetOutput_CC, SetGroupFrequency, Enable };

#define IMPELLER_SPEED_PWM_OUTPUT PWM_OUTPUT_8
#define IMPELLER_SPEED_PWM_GROUP PWM_GROUP_5678
#define IMPELLER_BTN_GPIO_PORT GPIOA
#define IMPELLER_BTN_GPIO5_PA8 LL_GPIO_PIN_8
#endif
"""

HARNESS = r"""
#include IMPELLER_SOURCE

#define DT 10
static uint32_t ms, button_ms;

static void run(uint32_t duration_ms) {
    for (uint32_t t = 0; t < duration_ms; t += DT) {
        Impeller.Update(DT);
        ms += DT;
        if (button) button_ms += DT;
        assert(primask == 0);
        /* The speed signal never exceeds the voltage of IMPELLER_MAX_SPEED_PCT. */
        if (!IMPELLER_CALIBRATION_MV)
            assert(cc_max <= Speed_mV(IMPELLER_MAX_SPEED_PCT * MPCT_PER_PCT) * (arr + 1) / IMPELLER_PWM_VREF_MV);
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

static uint32_t expected_cc(uint32_t mv) { return mv * (arr + 1) / IMPELLER_PWM_VREF_MV; }

int main(void) {
    const uint32_t max_mv = IMPELLER_V_MIN_MV
        + (IMPELLER_V_MAX_MV - IMPELLER_V_MIN_MV) * IMPELLER_MAX_SPEED_PCT / 100;
    const uint32_t ramp_ms = IMPELLER_MAX_SPEED_PCT * 1000 / IMPELLER_RAMP_PCT_PER_S;
    uint32_t t;

#if IMPELLER_CALIBRATION_MV
    /* Calibration mode: V_STOP outside ON, exactly IMPELLER_CALIBRATION_MV while ON. */
    Impeller.Init();
    run(1000);
    assert(Impeller.GetState() == IMPELLER_OFF && cc == expected_cc(IMPELLER_V_STOP_MV));
    Impeller.SetSpeed(10);
    Impeller.TurnOn();
    /* The voltage is applied first, the button is pressed only after it has settled. */
    t = ms;
    button_ms = 0;
    while (!button) {
        run(DT);
        assert(ms - t <= IMPELLER_SIGNAL_SETTLE_MS + DT);
        assert(Impeller.GetState() == IMPELLER_STARTING && cc == expected_cc(IMPELLER_CALIBRATION_MV));
    }
    assert(ms - t == IMPELLER_SIGNAL_SETTLE_MS + DT);
    while (Impeller.GetState() == IMPELLER_STARTING) {
        assert(cc == expected_cc(IMPELLER_CALIBRATION_MV));
        run(DT);
    }
    assert(Impeller.GetState() == IMPELLER_ON && button_ms == IMPELLER_PRESS_MS);
    assert(ms - t == IMPELLER_SIGNAL_SETTLE_MS + IMPELLER_PRESS_MS + IMPELLER_BOOT_DELAY_MS + DT);
    assert(cc == expected_cc(IMPELLER_CALIBRATION_MV) && presses == 1);
    run(5000);
    assert(cc == expected_cc(IMPELLER_CALIBRATION_MV));
    Impeller.TurnOff();
    run_until(IMPELLER_OFF, 5000);
    assert(cc == expected_cc(IMPELLER_V_STOP_MV) && presses == 2 && !button);
    printf("PASS: calibration mode, %u mV (CCR %u of %u) %u ms before the press and while ON, OFF -> %u mV\n",
        IMPELLER_CALIBRATION_MV, expected_cc(IMPELLER_CALIBRATION_MV), arr + 1,
        IMPELLER_SIGNAL_SETTLE_MS, IMPELLER_V_STOP_MV);
    return 0;
#endif

    /* Reset: button released before the pin becomes an output, signal at V_STOP. */
    Impeller.Init();
    assert(button_is_output && button_low_before_output && !button);
    assert(psc == 0 && arr == 84000000 / IMPELLER_PWM_FREQ_HZ - 1 && arr + 1 >= 256);
    assert(pwm_enabled && cc == expected_cc(IMPELLER_V_STOP_MV));
    run(5000);
    assert(Impeller.GetState() == IMPELLER_OFF && presses == 0 && cc == expected_cc(IMPELLER_V_STOP_MV));
    printf("PASS: reset -> OFF, no press, PWM %u steps at %u Hz\n", arr + 1, IMPELLER_PWM_FREQ_HZ);

    /* TurnOn: one press of IMPELLER_PRESS_MS, boot pause, then ramp to the limited speed. */
    Impeller.SetSpeed(100);
    Impeller.TurnOn();
    t = run_until(IMPELLER_ON, 5000);
    assert(presses == 1 && button_ms == IMPELLER_PRESS_MS && !button);
    assert(t == IMPELLER_PRESS_MS + IMPELLER_BOOT_DELAY_MS + DT);
    /* The first ramp step is made on the tick that enters ON. */
    assert(Impeller.GetSpeed() == 0 && cc > expected_cc(IMPELLER_V_MIN_MV) - 1 && cc < expected_cc(max_mv));
    run(ramp_ms - 2 * DT);
    assert(cc < expected_cc(max_mv));
    run(DT);
    assert(Impeller.GetSpeed() == IMPELLER_MAX_SPEED_PCT && cc == expected_cc(max_mv));
    printf("PASS: TurnOn press=%u ms, ON after %u ms, ramp to %u%% in %u ms, signal %u mV\n",
        button_ms, t, IMPELLER_MAX_SPEED_PCT, ramp_ms, max_mv);

    /* Repeated TurnOn in ON and a speed change do not press the button. */
    Impeller.TurnOn();
    Impeller.SetSpeed(10);
    run((IMPELLER_MAX_SPEED_PCT - 10) * 1000 / IMPELLER_RAMP_PCT_PER_S + 500);
    assert(presses == 1 && Impeller.GetState() == IMPELLER_ON && Impeller.GetSpeed() == 10);
    assert(cc == expected_cc(IMPELLER_V_MIN_MV + (IMPELLER_V_MAX_MV - IMPELLER_V_MIN_MV) / 10));
    printf("PASS: TurnOn in ON ignored, speed 10%%\n");

    /* TurnOff: ramp to 0, pause, one press, OFF. */
    button_ms = 0;
    Impeller.TurnOff();
    t = run_until(IMPELLER_OFF, 5000);
    assert(presses == 2 && button_ms == IMPELLER_PRESS_MS && !button);
    assert(t >= 10 * 1000 / IMPELLER_RAMP_PCT_PER_S + IMPELLER_STOP_DELAY_MS + IMPELLER_PRESS_MS);
    assert(Impeller.GetSpeed() == 0 && cc == expected_cc(IMPELLER_V_STOP_MV));
    Impeller.TurnOff();
    run(2000);
    assert(presses == 2 && Impeller.GetState() == IMPELLER_OFF);
    printf("PASS: TurnOff -> OFF after %u ms, TurnOff in OFF ignored\n", t);

    /* Debug press: exact length, limited, state unchanged. */
    button_ms = 0;
    Impeller.PressButton(150);
    run(1000);
    assert(presses == 3 && button_ms == 150 && Impeller.GetState() == IMPELLER_OFF);
    button_ms = 0;
    Impeller.PressButton(60000);
    run(5000);
    assert(presses == 4 && button_ms == IMPELLER_DEBUG_PRESS_MAX_MS);
    Impeller.PressButton(0);
    run(1000);
    assert(presses == 4);
    printf("PASS: debug press 150 ms, limited to %u ms, state unchanged\n", IMPELLER_DEBUG_PRESS_MAX_MS);

    /* Sync: the believed state changes without a press, ramp from/to 0. */
    Impeller.SetSpeed(20);
    Impeller.SyncState(1);
    run(DT);
    assert(Impeller.GetState() == IMPELLER_ON && presses == 4);
    run(ramp_ms + 1000);
    assert(Impeller.GetSpeed() == 20);
    Impeller.SyncState(0);
    run(DT);
    assert(Impeller.GetState() == IMPELLER_OFF && presses == 4);
    run(ramp_ms + 1000);
    assert(Impeller.GetSpeed() == 0 && cc == expected_cc(IMPELLER_V_STOP_MV));
    printf("PASS: sync ON / OFF without a press\n");

    /* Calibration voltage: applied only while ON, kept across on/off, 0 = speed control. */
    /* Set by command: at once even in OFF (impeller turned on by its own button), TurnOff returns 0 V. */
    assert(Impeller.SetRawVolts_mV(1000) == 1 && Impeller.GetRawVolts_mV() == 1000);
    assert(Impeller.SetRawVolts_mV(max_mv + 1) == 0 && Impeller.GetRawVolts_mV() == 1000);
    run(DT);
    assert(Impeller.GetState() == IMPELLER_OFF && cc == expected_cc(1000));
    Impeller.TurnOff();
    run(1000);
    assert(Impeller.GetState() == IMPELLER_OFF && cc == expected_cc(IMPELLER_V_STOP_MV));
    assert(Impeller.GetRawVolts_mV() == 1000 && presses == 4);
    Impeller.SetSpeed(20);
    Impeller.TurnOn();
    run_until(IMPELLER_STARTING, 100);
    assert(cc == expected_cc(1000) && !button);
    run_until(IMPELLER_ON, 5000);
    assert(cc == expected_cc(1000));
    Impeller.TurnOff();
    run_until(IMPELLER_OFF, 5000);
    assert(cc == expected_cc(IMPELLER_V_STOP_MV));
    Impeller.TurnOn();
    run_until(IMPELLER_ON, 5000);
    assert(cc == expected_cc(1000) && Impeller.GetRawVolts_mV() == 1000);
    assert(Impeller.SetRawVolts_mV(0) == 1);
    run(ramp_ms + 1000);
    assert(Impeller.GetSpeed() == 20 && cc == expected_cc(IMPELLER_V_MIN_MV + (IMPELLER_V_MAX_MV - IMPELLER_V_MIN_MV) / 5));
    Impeller.TurnOff();
    run_until(IMPELLER_OFF, 5000);
    assert(presses == 8);
    printf("PASS: calibration 1000 mV at once in OFF, 0 V after TurnOff, then only in ON, %u+1 mV rejected, 0 -> speed\n", max_mv);

    /* TurnOff during start: the start finishes, then the impeller is stopped. */
    Impeller.TurnOn();
    run(DT);
    assert(Impeller.GetState() == IMPELLER_STARTING);
    Impeller.TurnOff();
    run_until(IMPELLER_ON, 5000);
    run_until(IMPELLER_OFF, 5000);
    assert(presses == 10 && !button && Impeller.GetSpeed() == 0);
    printf("PASS: TurnOff during STARTING -> ON -> OFF, 2 presses\n");

    /* TurnOn during stop: the stop finishes, then the impeller starts again. */
    Impeller.TurnOn();
    run_until(IMPELLER_ON, 5000);
    Impeller.TurnOff();
    run(DT);
    assert(Impeller.GetState() == IMPELLER_STOPPING);
    Impeller.TurnOn();
    run_until(IMPELLER_OFF, 5000);
    run_until(IMPELLER_ON, 5000);
    assert(presses == 13);
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
        # Normal speed control, then the calibration voltage from impeller.h.
        for calibration in ("-DIMPELLER_CALIBRATION_MV=0", None):
            binary = folder / "test"
            subprocess.run([
                *shlex.split(os.environ.get("CC", "cc")), "-std=c99", "-O0",
                "-Wall", "-Wextra", "-I", str(folder), "-I", str(ROOT / "Core/Lib/Include"),
                f'-DIMPELLER_SOURCE="{SOURCE}"', *([calibration] if calibration else []),
                str(folder / "test.c"), "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
