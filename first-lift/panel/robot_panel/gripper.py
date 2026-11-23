"""Packets for the lift, axes, side grips, cursor, and block mechanisms."""

from __future__ import annotations

import struct
import time

from .command_codes import (
    CMD_BLOCKS_ROTATE_MASK,
    CMD_BLOCKS_SHIFT,
    CMD_GET_AX_ANGLE,
    CMD_SET_AX_ANGLE,
    CMD_GRAB_ROTATE,
    CMD_GRIPPER_LIFT,
    CMD_GRIPPER_SLIDE,
    CMD_GRIPPER_VERTICAL,
    CMD_SIDE_GRIPS,
)
from .protocol import ProtocolError


class GripperCommands:
    # The controller2026_4 firmware file says 98, but the live Dynamixel bus
    # answers at 106.  ID 13 is the rack-and-pinion slide motor.
    CURSOR_DYNAMIXEL_ID = 106
    CURSOR_VERTICAL_DEG = 80
    CURSOR_HORIZONTAL_DEG = 198
    CURSOR_NORMAL_SPEED = 600
    CURSOR_JOG_SPEED = 150

    def set_mechanism(
        self,
        command: int,
        payload: bytes,
        settle_seconds: float = 0.0,
        action: str | None = None,
    ) -> None:
        answer = self.command(command, payload, action=action)
        if answer != b"\x01":
            raise ProtocolError(
                f"Command 0x{command:02X} was not confirmed: {answer.hex(' ')}"
            )
        if settle_seconds > 0.0:
            time.sleep(settle_seconds)

    def lift(self, gripper: int, state: int) -> None:
        """0x20: gripper byte + state byte; 0=up, 2=middle, 1=down."""
        self.set_mechanism(CMD_GRIPPER_LIFT, bytes((gripper, state)))

    def side_grips(self, gripper: int, closed: bool) -> None:
        """0x1F: gripper byte + 1=closed/0=open."""
        self.set_mechanism(CMD_SIDE_GRIPS, bytes((gripper, int(closed))))

    def cursor(self, horizontal: bool) -> None:
        """Move the real cursor motor without the stale firmware ID 98."""
        target_deg = (
            self.CURSOR_HORIZONTAL_DEG
            if horizontal
            else self.CURSOR_VERTICAL_DEG
        )
        self.set_ax_angle(
            self.CURSOR_DYNAMIXEL_ID,
            target_deg,
            self.CURSOR_NORMAL_SPEED,
        )

    def vertical(self, position_mm: int) -> None:
        """0x27: one-byte absolute position in the range 0..255."""
        self.set_mechanism(CMD_GRIPPER_VERTICAL, bytes((position_mm,)))

    def slide(self, position_mm: int) -> None:
        """0x2F: signed little-endian int32 position in millimetres."""
        self.set_mechanism(CMD_GRIPPER_SLIDE, struct.pack("<i", position_mm))

    def ax_angle(self, dynamixel_id: int, action: str | None = None) -> int:
        """Read one AX-series Dynamixel angle in degrees."""
        answer = self.command(
            CMD_GET_AX_ANGLE,
            bytes((dynamixel_id,)),
            action=action,
        )
        if len(answer) != 3:
            raise ProtocolError(
                f"GET_AX_ANGLE returned {len(answer)} bytes instead of 3"
            )
        if answer[0] != 0:
            raise ProtocolError(
                f"Dynamixel {dynamixel_id} bus error 0x{answer[0]:02X}"
            )
        return int.from_bytes(answer[1:3], "little")

    def set_ax_angle(
        self,
        dynamixel_id: int,
        angle_deg: int,
        speed: int,
        action: str | None = None,
    ) -> None:
        """Set one AX-series Dynamixel target angle and moving speed."""
        answer = self.command(
            CMD_SET_AX_ANGLE,
            bytes((dynamixel_id,))
            + struct.pack("<HH", angle_deg, speed),
            action=action,
        )
        if answer:
            raise ProtocolError(
                f"SET_AX_ANGLE returned unexpected payload: {answer.hex(' ')}"
            )

    def cursor_jog(self, delta_deg: int) -> tuple[int, bool]:
        """Move the U/J cursor motor a few degrees from its measured position."""
        current_deg = self.ax_angle(
            self.CURSOR_DYNAMIXEL_ID,
            action="Read cursor motor angle before jog",
        )
        target_deg = max(
            self.CURSOR_VERTICAL_DEG,
            min(self.CURSOR_HORIZONTAL_DEG, current_deg + delta_deg),
        )
        if target_deg == current_deg:
            return target_deg, False
        self.set_ax_angle(
            self.CURSOR_DYNAMIXEL_ID,
            target_deg,
            self.CURSOR_JOG_SPEED,
            action=f"Cursor motor jog -> {target_deg} deg",
        )
        return target_deg, True

    def blocks_shift(self, gripper: int, spread: bool) -> None:
        """0x2B: gripper byte + 1=spread/0=close."""
        self.set_mechanism(
            CMD_BLOCKS_SHIFT, bytes((gripper, int(spread))), settle_seconds=0.30
        )

    def blocks_rotate(self, gripper: int, mask: int) -> None:
        """0x2C: gripper byte + four-bit rotation mask."""
        self.set_mechanism(
            CMD_BLOCKS_ROTATE_MASK,
            bytes((gripper, mask)),
            settle_seconds=0.70,
        )

    def grab_rotate(self, gripper: int, mask: int) -> None:
        """0x1C: full spread/rotate/close operation."""
        self.set_mechanism(
            CMD_GRAB_ROTATE, bytes((gripper, mask)), settle_seconds=1.20
        )
