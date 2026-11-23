"""Safe incremental control for Stepper Output 2 and S1/PWM5, S3/PWM6."""

from __future__ import annotations

from dataclasses import dataclass
import struct
import time

from .command_codes import (
    CMD_LIFT2_ARM,
    CMD_LIFT2_DISARM,
    CMD_LIFT2_GET_STATE,
    CMD_LIFT2_MOVE_REL,
    CMD_LIFT2_SET_SERVO,
    CMD_LIFT2_SET_ZERO,
    CMD_LIFT2_STOP,
)
from .protocol import ProtocolError


LIFT2_FLAG_INITIALIZED = 1 << 0
LIFT2_FLAG_ARMED = 1 << 1
LIFT2_FLAG_MOVING = 1 << 2
LIFT2_FLAG_SERVO_1 = 1 << 3
LIFT2_FLAG_SERVO_3 = 1 << 4
LIFT2_PWM_OUTPUTS = {1: 5, 3: 6}

RESULT_NAMES = {
    1: "invalid argument",
    2: "not armed; press 9, then A first",
    3: "busy; wait for the previous step or press S in Lift2 mode",
    4: "requested step is outside the safe range",
}


@dataclass(frozen=True)
class Lift2State:
    flags: int
    position_steps: int
    driver_status: int
    servo1_current_us: int
    servo1_target_us: int
    servo3_current_us: int
    servo3_target_us: int

    @property
    def armed(self) -> bool:
        return bool(self.flags & LIFT2_FLAG_ARMED)

    @property
    def moving(self) -> bool:
        return bool(self.flags & LIFT2_FLAG_MOVING)

    def check_stepper_ready(self) -> None:
        if not self.armed:
            raise ProtocolError("Stepper 2 выключен")
        if self.driver_status in (0, 0xFFFF):
            raise ProtocolError("Stepper 2: нет достоверного ответа L6474")
        if self.driver_status & 1:
            reason = ", OCD (перегрузка по току)" if not self.driver_status & 0x1000 else ""
            raise ProtocolError(f"Stepper 2: мост отключён{reason}, STATUS=0x{self.driver_status:04X}")

    def servo_enabled(self, servo_id: int) -> bool:
        flag = LIFT2_FLAG_SERVO_1 if servo_id == 1 else LIFT2_FLAG_SERVO_3
        return bool(self.flags & flag)


class Lift2Commands:
    """Commands added to the lymunt-2026 test firmware at IDs 0x30..0x36."""

    @staticmethod
    def _parse_lift2_state(payload: bytes) -> tuple[int, Lift2State]:
        if len(payload) != 16:
            raise ProtocolError(
                f"Lift2 response has {len(payload)} bytes instead of 16"
            )
        (
            result,
            flags,
            position_steps,
            driver_status,
            servo1_current_us,
            servo1_target_us,
            servo3_current_us,
            servo3_target_us,
        ) = struct.unpack("<BBiHHHHH", payload)
        return result, Lift2State(
            flags=flags,
            position_steps=position_steps,
            driver_status=driver_status,
            servo1_current_us=servo1_current_us,
            servo1_target_us=servo1_target_us,
            servo3_current_us=servo3_current_us,
            servo3_target_us=servo3_target_us,
        )

    def _lift2_command(
        self,
        command: int,
        payload: bytes = b"",
        *,
        action: str,
    ) -> Lift2State:
        result, state = self._parse_lift2_state(
            self.command(command, payload, action=action)
        )
        if result != 0:
            reason = RESULT_NAMES.get(result, f"firmware result {result}")
            raise ProtocolError(f"Lift2 command rejected: {reason}")
        return state

    def lift2_arm(self, *, action: str = "Lift2 ARM") -> Lift2State:
        """Initialize and hold Stepper 2 without moving it."""
        return self._lift2_command(CMD_LIFT2_ARM, action=action)

    def lift2_move_relative(
        self, steps: int, *, action: str | None = None
    ) -> Lift2State:
        if not -4000 <= steps <= 4000 or steps == 0:
            raise ValueError("Lift2 relative step must be -4000..-1 or 1..4000")
        self.lift2_get_state().check_stepper_ready()
        state = self._lift2_command(
            CMD_LIFT2_MOVE_REL,
            struct.pack("<i", steps),
            action=action or f"Lift2 Stepper 2 relative {steps:+d}",
        )
        state.check_stepper_ready()
        return state

    def lift2_stop(self, *, action: str = "Lift2 STOP") -> Lift2State:
        """Stop pulses while keeping Stepper 2 energized."""
        return self._lift2_command(CMD_LIFT2_STOP, action=action)

    def lift2_disarm(self, *, action: str = "Lift2 OFF") -> Lift2State:
        """Stop and disable Stepper 2 plus S1/PWM5 and S3/PWM6."""
        return self._lift2_command(CMD_LIFT2_DISARM, action=action)

    def lift2_set_servo(
        self,
        servo_id: int,
        pulse_us: int,
        *,
        action: str | None = None,
    ) -> Lift2State:
        if servo_id not in (1, 3):
            raise ValueError("Lift2 servo id must be 1 or 3")
        if pulse_us != 0 and not 1000 <= pulse_us <= 2000:
            raise ValueError("Lift2 servo pulse must be 0 or 1000..2000 us")
        return self._lift2_command(
            CMD_LIFT2_SET_SERVO,
            bytes((servo_id,)) + struct.pack("<H", pulse_us),
            action=(
                action
                or f"Lift2 S{servo_id}/PWM{LIFT2_PWM_OUTPUTS[servo_id]} -> {pulse_us} us"
            ),
        )

    def lift2_get_state(self, *, action: str = "Lift2 read state") -> Lift2State:
        return self._lift2_command(CMD_LIFT2_GET_STATE, action=action)

    def lift2_wait_stopped(self, timeout: float = 5.0) -> Lift2State:
        deadline = time.monotonic() + timeout
        while True:
            state = self.lift2_get_state(action="Lift2 wait for step completion")
            state.check_stepper_ready()
            if not state.moving:
                return state
            if time.monotonic() >= deadline:
                raise TimeoutError("Stepper 2 did not finish the short step in time")
            time.sleep(0.05)

    def lift2_set_zero(self, *, action: str = "Lift2 set zero") -> Lift2State:
        return self._lift2_command(CMD_LIFT2_SET_ZERO, action=action)
