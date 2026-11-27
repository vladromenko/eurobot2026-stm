"""2026 PWM8 commands. The controller reports duty, not shaft speed."""
from __future__ import annotations

import time
from dataclasses import dataclass

from .command_codes import (
    CMD_IMPELLER_DUTY, CMD_IMPELLER_START, CMD_IMPELLER_STATUS, CMD_IMPELLER_STOP,
)
from .protocol import ProtocolError


@dataclass(frozen=True)
class ImpellerStatus:
    enabled: bool
    applied: int
    target: int

    def __str__(self) -> str:
        return f"{'ON' if self.enabled else 'OFF'} | applied {self.applied}% target {self.target}%"


class ImpellerCommands:
    def _impeller_command(self, cmd: int, duty: int | None = None) -> ImpellerStatus:
        if duty is not None and not 0 <= duty <= 100:
            raise ValueError("PWM8 duty must be 0..100%")
        payload = b"" if duty is None else bytes((duty,))
        data = self.command(cmd, payload, response_timeout=1.0)
        if len(data) != 4 or data[1] not in (0, 2) or max(data[2:]) > 100:
            raise ProtocolError(f"Unexpected PWM8 reply: {data.hex(' ')}")
        if data[0] != 1:
            raise ProtocolError(f"STM32 rejected PWM8 command 0x{cmd:02X}")
        return ImpellerStatus(data[1] == 2, data[2], data[3])

    def impeller_status(self) -> ImpellerStatus:
        return self._impeller_command(CMD_IMPELLER_STATUS)

    def _update_impeller(self, cmd: int, duty: int | None = None) -> ImpellerStatus:
        self._impeller_command(cmd, duty)
        # Firmware applies requests on its next 10 ms task cycle. A reply to
        # the write can still contain the previous output.
        time.sleep(0.025)
        return self.impeller_status()

    def impeller_start(self, duty: int) -> ImpellerStatus:
        return self._update_impeller(CMD_IMPELLER_START, duty)

    def impeller_duty(self, duty: int) -> ImpellerStatus:
        return self._update_impeller(CMD_IMPELLER_DUTY, duty)

    def impeller_stop(self) -> ImpellerStatus:
        return self._update_impeller(CMD_IMPELLER_STOP)
