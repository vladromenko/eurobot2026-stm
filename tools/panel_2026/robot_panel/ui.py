"""Full-screen curses UI and keyboard mapping."""

from __future__ import annotations

import curses
import time
from collections import deque
from typing import Callable

from .controller import Robot2026
from .dispatcher import CommandDispatcher, WorkItem
from .guided_tests import FieldSpec, GUIDED_TESTS, GuidedTest
from .logs import TestResultLog

def safe_addstr(screen, row: int, column: int, text: str, style: int = 0) -> None:
    height, width = screen.getmaxyx()
    if row < 0 or row >= height or column >= width:
        return
    try:
        screen.addnstr(row, column, text, max(0, width - column - 1), style)
    except curses.error:
        pass


class Teleop:
    def __init__(
        self,
        screen,
        robot: Robot2026,
        test_results: TestResultLog,
        device: str,
        args,
        initial_pwm,
    ):
        self.screen = screen
        self.robot = robot
        self.test_results = test_results
        self.device = device
        self.pwm_status = initial_pwm
        self.pwm_selection = initial_pwm.target
        self.args = args
        self.selected_gripper = 1
        self.suction_mask = 15
        self.vertical_mm = args.vertical_start
        self.slide_mm = args.slide_start
        self.motors_enabled = False
        self.side_grips = {1: None, 2: None}
        self.cursor = None
        self.blocks_spread = {1: None, 2: None}
        self.notices: deque[str] = deque(maxlen=6)
        self.motion = (0.0, 0.0, 0.0)
        self.last_sent_motion = (0.0, 0.0, 0.0)
        self.motion_deadline = 0.0
        self.last_motion_tx = 0.0
        self.motion_label = "Base stopped"
        self.full_log_view = False
        self.mode = "manual"
        self.test_index = 0
        self.dispatcher: CommandDispatcher | None = None
        self.running = True
        self.colors: dict[str, int] = {}

    def init_screen(self) -> None:
        curses.curs_set(0)
        self.screen.nodelay(True)
        self.screen.timeout(25)
        if curses.has_colors():
            curses.start_color()
            try:
                curses.use_default_colors()
            except curses.error:
                pass
            for number, color in enumerate(
                (curses.COLOR_CYAN, curses.COLOR_GREEN, curses.COLOR_YELLOW, curses.COLOR_RED),
                start=1,
            ):
                try:
                    curses.init_pair(number, color, -1)
                except curses.error:
                    curses.init_pair(number, color, curses.COLOR_BLACK)
            self.colors = {
                "title": curses.color_pair(1) | curses.A_BOLD,
                "ok": curses.color_pair(2) | curses.A_BOLD,
                "warn": curses.color_pair(3) | curses.A_BOLD,
                "error": curses.color_pair(4) | curses.A_BOLD,
            }

    def style(self, name: str, fallback: int = 0) -> int:
        return self.colors.get(name, fallback)

    def notice(self, message: str) -> None:
        self.notices.appendleft(time.strftime("%H:%M:%S") + "  " + message)

    def render(self) -> None:
        self.screen.erase()
        height, width = self.screen.getmaxyx()
        if self.mode == "tests":
            self.render_test_menu(height, width)
        elif self.full_log_view:
            self.render_full_log(height, width)
        elif width >= 104 and height >= 34:
            self.render_wide(height, width)
        else:
            self.render_compact(height, width)
        self.screen.refresh()

    def render_header(self, width: int) -> None:
        safe_addstr(
            self.screen,
            0,
            0,
            " EUROBOT 2026  |  SSH TERMINAL CONTROL ".ljust(width - 1),
            self.style("title", curses.A_BOLD),
        )
        source = "DEMO (no hardware)" if self.args.demo else f"STM32 {self.device} @ {self.args.baud}"
        safe_addstr(self.screen, 1, 1, source)
        safe_addstr(
            self.screen,
            2,
            1,
            "F2 = GUIDED TESTS    SPACE = STOP BASE    0 = EMERGENCY STOP    ESC = SAFE EXIT",
            self.style("warn", curses.A_BOLD),
        )

    def render_test_menu(self, height: int, width: int) -> None:
        safe_addstr(
            self.screen,
            0,
            0,
            " GUIDED HARDWARE TESTS  |  F1 manual control  |  ESC safe exit ".ljust(width - 1),
            self.style("title", curses.A_BOLD),
        )
        safe_addstr(
            self.screen,
            2,
            1,
            "UP/DOWN or J/K: select    ENTER: open test    Every run asks for RUN and records your verdict.",
        )
        safe_addstr(self.screen, 4, 1, "AVAILABLE TESTS", curses.A_BOLD)
        max_rows = max(1, height - 11)
        start = min(
            max(0, self.test_index - max_rows + 1),
            max(0, len(GUIDED_TESTS) - max_rows),
        )
        for row_offset, index in enumerate(range(start, min(len(GUIDED_TESTS), start + max_rows))):
            test = GUIDED_TESTS[index]
            style = curses.A_REVERSE | curses.A_BOLD if index == self.test_index else 0
            marker = ">" if index == self.test_index else " "
            safe_addstr(
                self.screen,
                5 + row_offset,
                1,
                f"{marker} {index + 1:02d}. {test.title}",
                style,
            )

        selected = GUIDED_TESTS[self.test_index]
        detail_row = max(7, height - 5)
        safe_addstr(self.screen, detail_row, 1, selected.title, curses.A_BOLD)
        safe_addstr(self.screen, detail_row + 1, 1, selected.instruction)
        if selected.fields:
            ranges = " | ".join(self.field_hint(field) for field in selected.fields)
        else:
            ranges = "No values required"
        safe_addstr(self.screen, detail_row + 2, 1, "Inputs: " + ranges)
        safe_addstr(
            self.screen,
            height - 1,
            1,
            f"Results: {self.test_results.path.name}  |  Command log: {self.robot.trace.path.name}",
            curses.A_DIM,
        )

    @staticmethod
    def field_hint(field: FieldSpec) -> str:
        if field.choices:
            allowed = "/".join(field.choices)
        else:
            allowed = f"{field.minimum:g}..{field.maximum:g}"
        return f"{field.label} [{allowed}]"

    def render_state(self, row: int, column: int = 1) -> int:
        motor_text = "ON" if self.motors_enabled else "OFF"
        motor_style = self.style("ok") if self.motors_enabled else self.style("warn")
        safe_addstr(self.screen, row, column, "STATE", curses.A_BOLD)
        safe_addstr(self.screen, row, column + 7, f"MOTORS {motor_text}", motor_style)
        safe_addstr(
            self.screen,
            row + 1,
            column,
            f"gripper={self.selected_gripper}  mask={self.suction_mask:04b}  "
            f"vertical={self.vertical_mm}mm  slide={self.slide_mm}mm  "
            f"cursor-jog={self.args.cursor_jog_step}deg",
        )
        safe_addstr(
            self.screen,
            row + 2,
            column,
            f"linear={self.args.linear_speed:.2f}m/s  yaw={self.args.angular_speed:.2f}rad/s  "
            f"watchdog={self.args.drive_timeout:.2f}s",
        )
        busy, queued = self.dispatcher.status() if self.dispatcher else ("startup", 0)
        safe_addstr(
            self.screen,
            row + 3,
            column,
            f"controller={busy}  queued={queued}",
            self.style("warn") if busy != "idle" else 0,
        )
        return row + 4

    def render_impeller(self, row: int) -> int:
        status = self.pwm_status
        state = "ON" if status.enabled else "OFF"
        setting = "Target" if status.enabled else "Next start"
        safe_addstr(self.screen, row, 1, f"IMPELLER  {state}", self.style("ok" if status.enabled else "title", curses.A_BOLD))
        safe_addstr(self.screen, row + 1, 1, f" PWM output {status.applied}%   {setting} {self.pwm_selection}%")
        safe_addstr(self.screen, row + 2, 1, " 9 start   Backspace stop   ? status")
        safe_addstr(self.screen, row + 3, 1, " ↑/↓ duty +/−5%   </> duty −/+1%")
        return row + 4

    def render_wide(self, height: int, width: int) -> None:
        self.render_header(width)
        divider = max(55, min(68, width // 2))
        for row in range(3, height - 1):
            safe_addstr(self.screen, row, divider, "|", curses.A_DIM)

        row = self.render_impeller(self.render_state(4) + 1)
        controls = (
            ("DRIVE", curses.A_BOLD),
            (" W/S   forward/back       A/D   left/right", 0),
            (" Q/E   rotate +/-         hold key; release = stop", 0),
            (" M     motors on/off      V     release wheel holding", 0),
            (" -/=   linear speed -/+   ,/.   angular speed -/+", 0),
            (" SPACE stop base", 0),
            ("MECHANISMS", curses.A_BOLD),
            (" 1/2   select gripper     P/O   suction sequence on/off", 0),
            (" 3/4   pumps on/off       5/6   valves open/close", 0),
            (" [/]   suction mask -/+   Z/X/C lift up/mid/down", 0),
            (" Y/H   side grips close/open", 0),
            (" R/F   vertical +/− step  T/G   slide +/− step", 0),
            (" 7/8   cursor motor jog −/+ (small angle step)", 0),
            (" U/J   cursor jog +/−  Shift+U/J full horizontal/vertical", 0),
            (" B/N   blocks spread/close", 0),
            (" I/K   rotate selected blocks / return", 0),
            (" Shift+K spread + rotate + close", 0),
            (" L     full diagnostics: motion, BMS, wheels, switches", 0),
        )
        for text, style in controls:
            safe_addstr(self.screen, row, 1, text, style)
            row += 1

        notice_row = max(row + 1, height - 5)
        if notice_row < height - 1:
            safe_addstr(self.screen, notice_row, 1, "LAST RESULT", curses.A_BOLD)
            for offset, message in enumerate(self.notices):
                if notice_row + 1 + offset >= height - 1:
                    break
                safe_addstr(self.screen, notice_row + 1 + offset, 1, message)

        safe_addstr(
            self.screen,
            4,
            divider + 2,
            "LIVE COMMAND LOG  (TAB = full screen)",
            curses.A_BOLD,
        )
        self.render_trace_entries(6, divider + 2, height - 2, width - divider - 3)
        safe_addstr(
            self.screen,
            height - 1,
            1,
            f"JSONL: {self.robot.trace.path.name}",
            curses.A_DIM,
        )

    def render_compact(self, height: int, width: int) -> None:
        self.render_header(width)
        row = self.render_impeller(self.render_state(4) + 1)
        safe_addstr(
            self.screen,
            row,
            1,
            "Terminal is small: use >=104x34 for controls + live log.",
            self.style("warn", curses.A_BOLD),
        )
        row += 2
        for line in (
            "MOVE: W/S A/D Q/E | M motors | V free wheels | SPACE stop",
            "GRIP: 1/2 | P/O suction | 3/4 pumps | 5/6 valves",
            "AXES: R/F vertical | T/G slide",
            "CURSOR: U/J or 8/7 jog +/- | Shift+U/J full endpoints",
            "BLOCKS: B/N spread/close | I/K rotate/return | Shift+K cycle",
            "INFO: L telemetry | TAB full log | F2 guided tests | ESC exit",
        ):
            safe_addstr(self.screen, row, 1, line)
            row += 1
        row += 1
        safe_addstr(self.screen, row, 1, "LATEST TRANSACTIONS", curses.A_BOLD)
        self.render_trace_entries(row + 1, 1, height - 2, width - 2, compact=True)

    def render_trace_entries(
        self,
        start_row: int,
        column: int,
        end_row: int,
        pane_width: int,
        compact: bool = False,
    ) -> None:
        row = start_row
        for entry in self.robot.trace.entries:
            if row >= end_row:
                break
            clock = entry.timestamp[11:23]
            status_style = self.style("ok") if entry.result == "OK" else self.style("error")
            safe_addstr(
                self.screen,
                row,
                column,
                f"{clock} {entry.result:<5} {entry.command_code} {entry.command} {entry.latency_ms:.1f}ms",
                status_style,
            )
            row += 1
            if compact or row >= end_row:
                continue
            for prefix, value in (("action", entry.action), ("TX", entry.tx), ("RX", entry.rx)):
                if row >= end_row:
                    break
                safe_addstr(self.screen, row, column, f"  {prefix}: {value}"[:pane_width])
                row += 1
            if entry.detail and row < end_row:
                safe_addstr(self.screen, row, column, f"  result: {entry.detail}"[:pane_width])
                row += 1
            row += 1

    def render_full_log(self, height: int, width: int) -> None:
        safe_addstr(
            self.screen,
            0,
            0,
            " FULL TRANSACTION LOG  |  TAB return  |  ESC safe exit ".ljust(width - 1),
            self.style("title", curses.A_BOLD),
        )
        safe_addstr(self.screen, 1, 1, f"Saved continuously to: {self.robot.trace.path}")
        self.render_trace_entries(3, 1, height - 1, width - 2)

    def stop_base(self, quiet: bool = False, action: str = "Stop base") -> None:
        self.motion = (0.0, 0.0, 0.0)
        self.motion_deadline = 0.0
        if self.last_sent_motion != self.motion or not quiet:
            if self.dispatcher is None:
                self.robot.set_speed(0.0, 0.0, 0.0, action=action)
            else:
                self.dispatcher.submit(
                    "drive",
                    action,
                    lambda: self.robot.set_speed(0.0, 0.0, 0.0, action=action),
                    replace_pending=True,
                    front=True,
                )
            self.last_sent_motion = self.motion
            self.last_motion_tx = time.monotonic()
        self.motion_label = "Base stopped"
        if not quiet:
            self.notice("Base stopped")

    def discrete_action(
        self,
        label: str,
        action: Callable[[], None],
        channel: str = "mechanism",
        replace_pending: bool = False,
    ) -> None:
        self.stop_base(quiet=True, action="Stop before mechanism command")
        if self.dispatcher is None:
            action()
            self.notice(label + " OK")
            return
        self.dispatcher.submit(
            channel,
            label,
            action,
            replace_pending=replace_pending,
        )
        self.notice(label + " queued")

    def drive_key(self, key: str) -> None:
        if not self.motors_enabled:
            self.notice("Drive blocked: press M to enable motors")
            return
        if self.dispatcher is not None and self.dispatcher.mechanism_pending():
            self.notice("Drive blocked: controller is executing a mechanism command")
            return
        linear = self.args.linear_speed
        angular = self.args.angular_speed
        directions = {
            "w": ((linear, 0.0, 0.0), "W forward"),
            "s": ((-linear, 0.0, 0.0), "S backward"),
            "a": ((0.0, linear, 0.0), "A left"),
            "d": ((0.0, -linear, 0.0), "D right"),
            "q": ((0.0, 0.0, angular), "Q rotate +"),
            "e": ((0.0, 0.0, -angular), "E rotate -"),
        }
        self.motion, self.motion_label = directions[key]
        self.motion_deadline = time.monotonic() + self.args.drive_timeout

    def update_drive(self) -> None:
        now = time.monotonic()
        action = self.motion_label
        if self.motion != (0.0, 0.0, 0.0) and now >= self.motion_deadline:
            self.motion = (0.0, 0.0, 0.0)
            action = "Key released -> watchdog stop"
            self.motion_label = "Base stopped"
        if self.motion != self.last_sent_motion and now - self.last_motion_tx >= 0.04:
            if self.dispatcher is None:
                self.robot.set_speed(*self.motion, action=action)
            else:
                self.dispatcher.submit(
                    "drive",
                    action,
                    lambda motion=self.motion, label=action: self.robot.set_speed(
                        *motion, action=label
                    ),
                    replace_pending=True,
                    front=True,
                )
            self.last_sent_motion = self.motion
            self.last_motion_tx = now

    def toggle_motors(self) -> None:
        self.stop_base(quiet=True, action="Stop before motor toggle")
        target = not self.motors_enabled
        if self.dispatcher is None:
            self.robot.set_motors(target)
        else:
            label = "Enable drive motors" if target else "Disable drive motors"
            self.dispatcher.submit(
                "motors",
                label,
                lambda value=target, text=label: self.robot.set_motors(
                    value, action=text
                ),
                replace_pending=True,
            )
        self.motors_enabled = target
        self.notice(
            "Drive motors " + ("enable queued" if target else "disable queued")
        )

    def emergency_stop(self) -> None:
        self.motors_enabled = False
        self.motion = self.last_sent_motion = (0.0, 0.0, 0.0)
        self.motion_deadline = 0.0
        self.motion_label = "Base stopped"
        items = [
            WorkItem(
                "drive",
                "E-STOP / stop base",
                lambda: self.robot.set_speed(
                    0.0, 0.0, 0.0, action="E-STOP / stop base"
                ),
            ),
            WorkItem(
                "motors",
                "E-STOP / disable motors",
                lambda: self.robot.set_motors(
                    False, action="E-STOP / disable motors"
                ),
            ),
            WorkItem("impeller", "E-STOP / PWM8 off", self.stop_impeller),
            WorkItem(
                "suction-1",
                "E-STOP / gripper 1 suction off",
                lambda: self.robot.suction(
                    1, 0, False, action="E-STOP / gripper 1 off"
                ),
            ),
            WorkItem(
                "suction-2",
                "E-STOP / gripper 2 suction off",
                lambda: self.robot.suction(
                    2, 0, False, action="E-STOP / gripper 2 off"
                ),
            ),
        ]
        if self.dispatcher is None:
            for item in items:
                item.action()
        else:
            self.dispatcher.replace_queue(items)
        self.notice("FULL STOP queued: base, impeller and suction off")

    def stop_impeller(self) -> None:
        self.pwm_status = self.robot.impeller_stop()

    def impeller_key(self, code: int) -> None:
        steps = {curses.KEY_F7: -5, curses.KEY_F8: 5, ord("<"): -1, ord(">"): 1}
        if code in steps:
            step = steps[code]
            self.pwm_selection = max(0, min(100, self.pwm_selection + step))
        duty = self.pwm_selection

        def apply() -> None:
            if code == curses.KEY_F5:
                self.pwm_status = self.robot.impeller_start(duty)
            elif code == curses.KEY_F6:
                self.stop_impeller()
            elif code == curses.KEY_F9:
                self.pwm_status = self.robot.impeller_status()
            else:
                self.pwm_status = self.robot.impeller_duty(duty)

        label = {curses.KEY_F5: f"Start impeller at {duty}%", curses.KEY_F6: "Stop impeller",
                 curses.KEY_F9: "Read impeller status"}.get(code, f"Set impeller duty to {duty}%")
        if self.dispatcher is None:
            apply()
        else:
            stop = code == curses.KEY_F6
            self.dispatcher.submit("impeller", label, apply, replace_pending=stop, front=stop)

    def mechanism_key(self, key: str, shifted: bool = False) -> bool:
        grip = self.selected_gripper
        if key == "p":
            mask = self.suction_mask
            self.discrete_action(
                f"Gripper {grip}: suction ON mask {mask:04b}",
                lambda g=grip, m=mask: self.robot.suction(g, m, True),
                channel=f"suction-{grip}",
            )
        elif key == "o":
            self.discrete_action(
                f"Gripper {grip}: suction OFF",
                lambda g=grip: self.robot.suction(g, 0, False),
                channel=f"suction-{grip}",
            )
        elif key in "34":
            mask = self.suction_mask if key == "3" else 0
            self.discrete_action(
                f"Gripper {grip}: pumps mask {mask:04b}",
                lambda g=grip, m=mask: self.robot.pumps(g, m),
                channel=f"suction-{grip}",
            )
        elif key in "56":
            mask = self.suction_mask if key == "5" else 0
            self.discrete_action(
                f"Gripper {grip}: valves mask {mask:04b}",
                lambda g=grip, m=mask: self.robot.valves(g, m),
                channel=f"suction-{grip}",
            )
        elif key in "zxc":
            state = {"z": 0, "x": 2, "c": 1}[key]
            name = {0: "up", 2: "middle", 1: "down"}[state]
            self.discrete_action(
                f"Gripper {grip}: lift {name}",
                lambda g=grip, s=state: self.robot.lift(g, s),
                channel=f"lift-{grip}",
                replace_pending=True,
            )
        elif key in "yh":
            state = key == "y"
            self.discrete_action(
                f"Gripper {grip}: side grips {'close' if state else 'open'}",
                lambda g=grip, s=state: self.robot.side_grips(g, s),
                channel=f"side-{grip}",
                replace_pending=True,
            )
            self.side_grips[grip] = state
        elif key in "uj":
            if shifted:
                state = key == "u"
                self.discrete_action(
                    f"Cursor {'horizontal' if state else 'vertical'}",
                    lambda s=state: self.robot.cursor(s),
                    channel="cursor",
                    replace_pending=True,
                )
                self.cursor = state
            else:
                delta = (
                    self.args.cursor_jog_step
                    if key == "u"
                    else -self.args.cursor_jog_step
                )
                label = (
                    f"Cursor motor jog {'+' if delta > 0 else '-'}"
                    f"{abs(delta)} deg"
                )

                def jog_cursor_motor() -> None:
                    target, moved = self.robot.cursor_jog(delta)
                    if not moved:
                        self.notice("Cursor motor jog limit reached")
                    else:
                        self.notice(f"Cursor motor -> {target} deg")

                self.discrete_action(
                    label,
                    jog_cursor_motor,
                    channel="cursor",
                    replace_pending=True,
                )
        elif key in "bn":
            state = key == "b"
            self.discrete_action(
                f"Gripper {grip}: blocks {'spread' if state else 'close'}",
                lambda g=grip, s=state: self.robot.blocks_shift(g, s),
                channel=f"blocks-{grip}",
                replace_pending=True,
            )
            self.blocks_spread[grip] = state
        elif key == "i":
            mask = self.suction_mask
            self.discrete_action(
                f"Gripper {grip}: rotate selected blocks {mask:04b}",
                lambda g=grip, m=mask: self.robot.blocks_rotate(g, m),
                channel=f"blocks-{grip}",
                replace_pending=True,
            )
        elif key == "k":
            if shifted:
                mask = self.suction_mask
                self.discrete_action(
                    f"Gripper {grip}: spread/rotate/close mask {mask:04b}",
                    lambda g=grip, m=mask: self.robot.grab_rotate(g, m),
                    channel=f"blocks-{grip}",
                    replace_pending=True,
                )
            else:
                self.discrete_action(
                    f"Gripper {grip}: return rotated blocks",
                    lambda g=grip: self.robot.blocks_rotate(g, 0),
                    channel=f"blocks-{grip}",
                    replace_pending=True,
                )
        elif key in "rf":
            delta = self.args.vertical_step if key == "r" else -self.args.vertical_step
            target = max(0, min(255, self.vertical_mm + delta))
            if target == self.vertical_mm:
                self.notice("Vertical limit reached")
            else:
                self.discrete_action(
                    f"Vertical axis -> {target} mm",
                    lambda value=target: self.robot.vertical(value),
                    channel="vertical",
                    replace_pending=True,
                )
                self.vertical_mm = target
        elif key in "tg":
            delta = self.args.slide_step if key == "t" else -self.args.slide_step
            target = max(-37, min(37, self.slide_mm + delta))
            if target == self.slide_mm:
                self.notice("Slide limit reached")
            else:
                self.discrete_action(
                    f"Horizontal slide -> {target} mm",
                    lambda value=target: self.robot.slide(value),
                    channel="slide",
                    replace_pending=True,
                )
                self.slide_mm = target
        elif key in "78":
            delta = (
                self.args.cursor_jog_step
                if key == "8"
                else -self.args.cursor_jog_step
            )
            label = f"Cursor motor jog {'+' if delta > 0 else '-'}{abs(delta)} deg"

            def jog_cursor_motor() -> None:
                target, moved = self.robot.cursor_jog(delta)
                if not moved:
                    self.notice("Cursor motor jog limit reached")
                else:
                    self.notice(f"Cursor motor -> {target} deg")

            self.discrete_action(
                label,
                jog_cursor_motor,
                channel="cursor",
                replace_pending=True,
            )
        else:
            return False
        return True

    def draw_test_form(
        self,
        test: GuidedTest,
        values: dict[str, object],
        prompt: str,
        error: str = "",
    ) -> tuple[int, int]:
        self.screen.erase()
        height, width = self.screen.getmaxyx()
        safe_addstr(
            self.screen,
            0,
            0,
            f" GUIDED TEST: {test.title} ".ljust(width - 1),
            self.style("title", curses.A_BOLD),
        )
        safe_addstr(self.screen, 2, 1, test.instruction)
        safe_addstr(self.screen, 4, 1, "LIMITS / ALLOWED VALUES", curses.A_BOLD)
        row = 5
        if test.fields:
            for field in test.fields:
                safe_addstr(self.screen, row, 2, "- " + self.field_hint(field))
                row += 1
        else:
            safe_addstr(self.screen, row, 2, "- no input values")
            row += 1
        row += 1
        safe_addstr(self.screen, row, 1, "ENTERED VALUES", curses.A_BOLD)
        row += 1
        if values:
            for name, value in values.items():
                safe_addstr(self.screen, row, 2, f"{name} = {value}")
                row += 1
        else:
            safe_addstr(self.screen, row, 2, "(none yet)")
            row += 1
        if error:
            message_style = (
                self.style("ok", curses.A_BOLD)
                if error.startswith("OK:")
                else self.style("error", curses.A_BOLD)
            )
            safe_addstr(
                self.screen,
                min(height - 4, row + 1),
                1,
                error,
                message_style,
            )
        prompt_row = height - 2
        safe_addstr(self.screen, prompt_row, 1, prompt, self.style("warn", curses.A_BOLD))
        self.screen.clrtoeol()
        self.screen.refresh()
        return prompt_row, min(width - 2, len(prompt) + 2)

    def read_test_line(
        self,
        test: GuidedTest,
        values: dict[str, object],
        prompt: str,
        error: str = "",
    ) -> str | None:
        row, column = self.draw_test_form(test, values, prompt, error)
        self.screen.nodelay(False)
        self.screen.timeout(-1)
        try:
            try:
                curses.curs_set(1)
            except curses.error:
                pass
            curses.echo()
            raw = self.screen.getstr(row, column, 120)
        finally:
            curses.noecho()
            try:
                curses.curs_set(0)
            except curses.error:
                pass
            self.screen.nodelay(True)
            self.screen.timeout(25)
        if raw == b"\x1b":
            return None
        return raw.decode("utf-8", errors="replace").strip()

    def ask_field(
        self,
        test: GuidedTest,
        values: dict[str, object],
        field: FieldSpec,
    ) -> object | None:
        error = ""
        while True:
            suffix = f" (default {field.default})" if field.default else ""
            entered = self.read_test_line(
                test,
                values,
                f"{field.label}{suffix}: ",
                error,
            )
            if entered is None:
                return None
            if not entered and field.default:
                entered = field.default
            try:
                if field.kind == "choice":
                    normalized = entered.upper()
                    allowed = {choice.upper(): choice for choice in field.choices}
                    if normalized not in allowed:
                        raise ValueError("choose " + "/".join(field.choices))
                    return allowed[normalized]
                value: int | float
                value = int(entered) if field.kind == "int" else float(entered)
                if field.minimum is not None and value < field.minimum:
                    raise ValueError(f"minimum is {field.minimum:g}")
                if field.maximum is not None and value > field.maximum:
                    raise ValueError(f"maximum is {field.maximum:g}")
                return value
            except ValueError as exc:
                error = f"Invalid value: {exc}"

    def execute_guided_test(
        self, test: GuidedTest, values: dict[str, object]
    ) -> str:
        label = "Guided test / " + test.title
        if self.dispatcher is not None and not self.dispatcher.wait_idle(
            self.args.timeout + 5.0
        ):
            raise TimeoutError("Previous controller command is still running")
        self.motion = self.last_sent_motion = (0.0, 0.0, 0.0)
        self.motion_deadline = 0.0
        self.robot.set_speed(
            0.0, 0.0, 0.0, action=label + " / preliminary stop"
        )
        previous = self.robot.action_context
        self.robot.action_context = label
        try:
            if test.key == "base":
                direction = str(values["direction"])
                speed = float(values["speed"])
                duration = float(values["duration"])
                vectors = {
                    "W": (speed, 0.0, 0.0),
                    "S": (-speed, 0.0, 0.0),
                    "A": (0.0, speed, 0.0),
                    "D": (0.0, -speed, 0.0),
                    "Q": (0.0, 0.0, speed),
                    "E": (0.0, 0.0, -speed),
                }
                self.robot.set_motors(True, action=label + " / enable motors")
                self.motors_enabled = True
                try:
                    self.robot.set_speed(*vectors[direction], action=label + f" / move {direction}")
                    time.sleep(duration)
                finally:
                    self.robot.set_speed(0.0, 0.0, 0.0, action=label + " / stop")
                    self.robot.set_motors(False, action=label + " / disable motors")
                    self.motors_enabled = False
                return f"Direction {direction}, speed {speed}, duration {duration}s"
            if test.key == "impeller_duty":
                status = self.robot.impeller_duty(int(values["duty"]))
                self.pwm_status = status
                self.pwm_selection = status.target
                return f"Target {status.target}%, PWM output {status.applied}%"
            if test.key == "suction":
                gripper = int(values["gripper"])
                mask = int(values["mask"])
                enabled = values["operation"] == "ON"
                self.robot.suction(gripper, mask, enabled)
                return f"Gripper {gripper}, mask {mask:04b}, {'ON' if enabled else 'OFF'}"
            if test.key == "lift":
                gripper = int(values["gripper"])
                state = int(values["state"])
                self.robot.lift(gripper, state)
                return f"Gripper {gripper}, state {state}"
            if test.key == "vertical":
                position = int(values["position"])
                self.robot.vertical(position)
                self.vertical_mm = position
                return f"Position {position} mm"
            if test.key == "slide":
                position = int(values["position"])
                self.robot.slide(position)
                self.slide_mm = position
                return f"Position {position} mm"
            if test.key == "side_grips":
                gripper = int(values["gripper"])
                state = values["state"] == "CLOSE"
                self.robot.side_grips(gripper, state)
                self.side_grips[gripper] = state
                return f"Gripper {gripper}, {values['state']}"
            if test.key == "cursor":
                state = values["state"] == "HORIZONTAL"
                self.robot.cursor(state)
                self.cursor = state
                return str(values["state"])
            if test.key == "blocks_shift":
                gripper = int(values["gripper"])
                state = values["state"] == "SPREAD"
                self.robot.blocks_shift(gripper, state)
                self.blocks_spread[gripper] = state
                return f"Gripper {gripper}, {values['state']}"
            if test.key == "blocks_mask":
                gripper = int(values["gripper"])
                mask = int(values["mask"])
                self.robot.blocks_rotate(gripper, mask)
                return f"Gripper {gripper}, mask {mask:04b}"
            if test.key == "grab_rotate":
                gripper = int(values["gripper"])
                mask = int(values["mask"])
                self.robot.grab_rotate(gripper, mask)
                return f"Gripper {gripper}, mask {mask:04b}"
            if test.key == "telemetry":
                coord, speed, switches = self.robot.telemetry()
                diagnostics = self.robot.diagnostics()
                return (
                    f"pose={tuple(round(x, 3) for x in coord)}; "
                    f"speed={tuple(round(x, 3) for x in speed)}; switches={switches}; "
                    f"battery={diagnostics['battery']}; "
                    f"ticks={diagnostics['wheel_ticks']}; "
                    f"paths={diagnostics['wheel_paths_m']}; "
                    f"side={diagnostics['field_side']}; "
                    f"start_cord={diagnostics['start_cord_attached']}"
                )
            raise ValueError(f"Unknown guided test: {test.key}")
        finally:
            self.robot.action_context = previous

    def run_guided_test(self, test: GuidedTest) -> None:
        values: dict[str, object] = {}
        for field in test.fields:
            value = self.ask_field(test, values, field)
            if value is None:
                self.notice(f"Cancelled test: {test.title}")
                return
            values[field.name] = value

        confirmation = self.read_test_line(
            test,
            values,
            "Type RUN and press Enter to send commands (Esc cancels): ",
        )
        if confirmation != "RUN":
            self.notice(f"Cancelled test: {test.title}")
            return

        try:
            command_result = "OK: " + self.execute_guided_test(test, values)
        except Exception as exc:
            command_result = "ERROR: " + str(exc)

        verdict = ""
        error = command_result
        while verdict not in {"PASS", "FAIL", "SKIP"}:
            answer = self.read_test_line(
                test,
                values,
                "Operator verdict PASS/FAIL/SKIP: ",
                error,
            )
            if answer is None:
                verdict = "SKIP"
                break
            verdict = answer.upper()
            error = command_result + " | Enter PASS, FAIL or SKIP"
        note = self.read_test_line(
            test,
            values,
            "Observation/note (may be empty): ",
            command_result,
        )
        if note is None:
            note = ""
        self.test_results.add(
            test_name=test.title,
            values=values,
            command_result=command_result,
            verdict=verdict,
            note=note,
        )
        self.notice(f"{test.title}: {verdict}; result saved")

    def handle_test_key(self, code: int) -> None:
        if code in (curses.KEY_UP, ord("k"), ord("K")):
            self.test_index = (self.test_index - 1) % len(GUIDED_TESTS)
        elif code in (curses.KEY_DOWN, ord("j"), ord("J")):
            self.test_index = (self.test_index + 1) % len(GUIDED_TESTS)
        elif code in (10, 13, curses.KEY_ENTER):
            self.run_guided_test(GUIDED_TESTS[self.test_index])

    def handle_key(self, code: int) -> None:
        if code == -1 or code == curses.KEY_RESIZE:
            return
        if code == 27:
            self.running = False
            return
        if code == curses.KEY_F1:
            self.mode = "manual"
            self.full_log_view = False
            return
        if code == curses.KEY_F2:
            self.stop_base(quiet=True, action="Switch to guided tests / stop base")
            self.mode = "tests"
            self.full_log_view = False
            return
        if code == ord("0"):
            self.emergency_stop()
            return
        if code in (curses.KEY_F6, curses.KEY_BACKSPACE, 8, 127):
            self.impeller_key(curses.KEY_F6)
            return
        if self.mode == "tests":
            self.handle_test_key(code)
            return
        impeller_keys = {
            ord("9"): curses.KEY_F5, curses.KEY_UP: curses.KEY_F8,
            curses.KEY_DOWN: curses.KEY_F7, ord("?"): curses.KEY_F9,
        }
        impeller_code = impeller_keys.get(code, code)
        if impeller_code in (curses.KEY_F5, curses.KEY_F7, curses.KEY_F8,
                              curses.KEY_F9, ord("<"), ord(">")):
            self.impeller_key(impeller_code)
            return
        if code == 9:
            self.full_log_view = not self.full_log_view
            return
        try:
            raw_key = chr(code)
            key = raw_key.lower()
        except (ValueError, OverflowError):
            return

        if key in "wasdqe":
            self.drive_key(key)
        elif key == " ":
            self.stop_base()
        elif key in "12":
            self.selected_gripper = int(key)
            self.notice(f"Selected gripper {key}")
        elif key == "m":
            self.toggle_motors()
        elif key == "v":
            self.stop_base(quiet=True, action="Stop before releasing wheels")
            self.motors_enabled = False
            if self.dispatcher is None:
                self.robot.free_wheels()
            else:
                self.dispatcher.submit(
                    "motors",
                    "Release wheel holding",
                    self.robot.free_wheels,
                    replace_pending=True,
                )
                self.notice("Release wheel holding queued")
        elif key == "-":
            self.args.linear_speed = max(0.02, self.args.linear_speed - 0.02)
            self.notice(f"Linear speed {self.args.linear_speed:.2f} m/s")
        elif key == "=":
            self.args.linear_speed = min(0.60, self.args.linear_speed + 0.02)
            self.notice(f"Linear speed {self.args.linear_speed:.2f} m/s")
        elif key == ",":
            self.args.angular_speed = max(0.10, self.args.angular_speed - 0.10)
            self.notice(f"Angular speed {self.args.angular_speed:.2f} rad/s")
        elif key == ".":
            self.args.angular_speed = min(3.00, self.args.angular_speed + 0.10)
            self.notice(f"Angular speed {self.args.angular_speed:.2f} rad/s")
        elif key == "[":
            self.suction_mask = max(0, self.suction_mask - 1)
            self.notice(f"Mask {self.suction_mask:04b}")
        elif key == "]":
            self.suction_mask = min(15, self.suction_mask + 1)
            self.notice(f"Mask {self.suction_mask:04b}")
        elif key == "l":
            def read_and_show_telemetry() -> None:
                coord, speed, switches = self.robot.telemetry()
                diagnostics = self.robot.diagnostics()
                battery = diagnostics["battery"]
                self.notice("Switches: " + "".join(str(value) for value in switches))
                self.notice(
                    f"Speed: vx={speed[0]:.3f} vy={speed[1]:.3f} yaw={speed[2]:.3f}"
                )
                self.notice(
                    f"Pose: x={coord[0]:.3f} y={coord[1]:.3f} phi={coord[2]:.3f}"
                )
                self.notice(
                    f"BMS: {battery['voltage_mv']}mV {battery['current_ma']}mA "
                    f"SOC={battery['soc_percent']}%"
                )
                self.notice(
                    f"Wheels ticks={diagnostics['wheel_ticks']} "
                    f"path={tuple(round(x, 3) for x in diagnostics['wheel_paths_m'])}m"
                )
                self.notice(
                    f"Side={diagnostics['field_side']} start-cord="
                    f"{int(diagnostics['start_cord_attached'])}"
                )

            if self.dispatcher is None:
                read_and_show_telemetry()
            else:
                self.dispatcher.submit(
                    "telemetry",
                    "Read telemetry",
                    read_and_show_telemetry,
                    replace_pending=True,
                )
                self.notice("Telemetry queued")
        else:
            self.mechanism_key(key, shifted=raw_key.isupper())

    def run(self) -> None:
        self.init_screen()
        self.dispatcher = CommandDispatcher(self.robot)
        self.notice("Connected; drive motors deliberately disabled")
        try:
            while self.running:
                for status, message in self.dispatcher.drain_events():
                    prefix = "OK: " if status == "OK" else "ERROR: "
                    self.notice(prefix + message)
                self.handle_key(self.screen.getch())
                self.update_drive()
                self.render()
        finally:
            self.dispatcher.cancel_pending()
            self.dispatcher.close(timeout=self.args.timeout + 5.0)



