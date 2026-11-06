#!/usr/bin/env python3
"""Run the actual GameTask on the host with simulated RTOS time and cord input.

No serial ports or debug probes are opened. Requires a host C compiler (CC/cc).
"""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Core/OS_Tasks/game_task.c"

STUBS = r"""
#ifndef TEST_STUBS_H
#define TEST_STUBS_H
#include <stdint.h>
#include <stddef.h>
#include <setjmp.h>
#include <assert.h>
#include <stdio.h>
typedef uint32_t TickType_t;
typedef uint32_t StackType_t;
typedef unsigned UBaseType_t;
typedef int StaticTask_t;
typedef void *TaskHandle_t;
enum { STARTING_CORD_ATTACHED, STARTING_CORD_RELEASED };
enum { GAME_MODE, DEBUG_MODE, RESETED };
struct { int starting_cord, robot_mode, reset_state; } robot_ctrl;
struct os_task_base_s {
    void (*CreateTask)(UBaseType_t);
    TaskHandle_t (*GetTaskHandle)(void);
};
uint8_t COMM_ENABLE, gripper_cmd, gripper_id, grips_cmd;
uint8_t dir, move_cmd_speed, move_cmd_time;
static TickType_t ticks, first_shutdown;
static unsigned shutdowns, stops, enables;
static int scenario;
static jmp_buf finished;

static void advance(TickType_t delta) {
    ticks += delta;
    /* Simulate the button callback, including an initially released cord. */
    robot_ctrl.starting_cord = (scenario == 0 || (scenario == 2 && ticks < 1000))
        ? STARTING_CORD_ATTACHED : STARTING_CORD_RELEASED;
    if (ticks >= 180000) longjmp(finished, 1);
}
static void vTaskDelay(TickType_t delay) { advance(delay); }
#define taskYIELD() advance(1)
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms)) /* Same 1000 Hz as the firmware. */
static TickType_t xTaskGetTickCount(void) { return ticks; }
#define IWDG NULL
static void LL_IWDG_ReloadCounter(void *unused) { (void)unused; }
static void MaxonsStop(void) { ++stops; }
static void MaxonsEnable(void) { ++enables; }
static void StopAllActuators(void) {
    if (!shutdowns) first_shutdown = ticks;
    ++shutdowns;
}
static TaskHandle_t xTaskCreateStatic(void (*fn)(void *), const char *name,
        unsigned depth, void *arg, UBaseType_t priority,
        StackType_t *stack, StaticTask_t *buffer) {
    (void)fn; (void)name; (void)depth; (void)arg;
    (void)priority; (void)stack; (void)buffer;
    return NULL;
}
#endif
"""

HARNESS = r"""
#include GAME_TASK_SOURCE

int main(void) {
    for (scenario = 0; scenario < 3; ++scenario) {
        ticks = first_shutdown = 0;
        shutdowns = stops = enables = 0;
        COMM_ENABLE = 1;
        gripper_cmd = 0; /* A busy gripper avoids scheduling unrelated work. */
        if (setjmp(finished) == 0) Task(NULL);
        assert(ticks == 180000);
        assert(enables == 1);
#if EUROBOT_MATCH_TIMEOUT
        if (scenario != 0) {
            assert(shutdowns == 1 && stops == 2 && COMM_ENABLE == 0);
            assert(first_shutdown == (scenario == 1 ? 99100 : 100000));
        } else {
            assert(shutdowns == 0 && stops == 1 && COMM_ENABLE == 1);
        }
#else
        assert(shutdowns == 0 && stops == 1 && COMM_ENABLE == 1);
#endif
        printf("PASS: timeout=%d, cord=%s, 180 seconds, shutdowns=%u, COMM_ENABLE=%u\n",
            EUROBOT_MATCH_TIMEOUT,
            scenario == 0 ? "attached" : scenario == 1 ? "released at boot" : "released after 1 s",
            shutdowns, COMM_ENABLE);
    }
    return 0;
}
"""


def main():
    with tempfile.TemporaryDirectory(prefix="eurobot-game-timeout-") as temp:
        folder = Path(temp)
        # Supply only hardware/RTOS declarations; compile GameTask unmodified.
        for header in re.findall(r'^#include "([^"]+)"', SOURCE.read_text(), re.M):
            (folder / header).write_text('#include "test_stubs.h"\n')
        (folder / "test_stubs.h").write_text(STUBS)
        (folder / "test.c").write_text(HARNESS)
        for enabled in (1, 0):
            binary = folder / f"test-{enabled}"
            subprocess.run([
                *shlex.split(os.environ.get("CC", "cc")), "-std=c99", "-O0",
                "-Wall", "-Wextra", "-Wno-unused-parameter", "-I", str(folder),
                f'-DGAME_TASK_SOURCE="{SOURCE}"',
                f"-DEUROBOT_MATCH_TIMEOUT={enabled}",
                str(folder / "test.c"), "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
