"""All packets that command the mobile chassis."""

from __future__ import annotations

import struct

from .command_codes import CMD_FREE_WHEELS, CMD_SET_MAXONS, CMD_SET_SPEED
from .protocol import ProtocolError


class ChassisCommands:
    def set_speed(
        self, vx: float, vy: float, yaw: float, action: str = "Set base speed"
    ) -> None:
        """Send command 0x04 with three little-endian float32 values."""
        self.command(
            CMD_SET_SPEED,
            struct.pack("<fff", vx, vy, yaw),
            response_timeout=0.40,
            action=action,
        )

    def set_motors(self, enabled: bool, action: str | None = None) -> None:
        """Send command 0x15 with one byte: 1=enabled, 0=disabled."""
        self.command(
            CMD_SET_MAXONS,
            bytes((int(enabled),)),
            response_timeout=0.60,
            action=action
            or ("Enable drive motors" if enabled else "Disable drive motors"),
        )

    def free_wheels(self) -> None:
        """Send command 0x2A to stop and remove Maxon holding torque."""
        answer = self.command(CMD_FREE_WHEELS, action="Release wheel holding")
        if answer != b"\x01":
            raise ProtocolError(f"Free-wheels command failed: {answer.hex(' ')}")
