"""In-process fake serial port used by ``--demo``."""

from __future__ import annotations

import struct

from .command_codes import (
    CMD_BLOCKS_ROTATE_MASK,
    CMD_BLOCKS_SHIFT,
    CMD_CURSOR,
    CMD_IMPELLER_START,
    CMD_IMPELLER_STOP,
    CMD_IMPELLER_STATUS,
    CMD_IMPELLER_DUTY,
    CMD_FREE_WHEELS,
    CMD_GET_BATTERY,
    CMD_GET_AX_ANGLE,
    CMD_GET_COORD,
    CMD_GET_DEBUG,
    CMD_GET_KEY,
    CMD_GET_SPEED,
    CMD_GET_SWITCHES,
    CMD_GET_TOGGLE,
    CMD_GET_WHEEL_PATH,
    CMD_GET_WHEEL_TICKS,
    CMD_GRAB_ROTATE,
    CMD_GRIPPER_LIFT,
    CMD_GRIPPER_SLIDE,
    CMD_GRIPPER_VERTICAL,
    CMD_PUMPS,
    CMD_SET_AX_ANGLE,
    CMD_SIDE_GRIPS,
    CMD_VALVES,
)
from .protocol import ProtocolError, make_packet


class DemoPort:
    def __init__(self, timeout: float = 1.0):
        self.timeout = timeout
        self.write_timeout = timeout
        self._rx = bytearray()
        self.impeller_on = False
        self.impeller_target = 75
        self._ax_angles = {106: 80}

    def __enter__(self):
        return self

    def __exit__(self, _kind, _value, _traceback) -> None:
        return None

    def reset_input_buffer(self) -> None:
        self._rx.clear()

    def flush(self) -> None:
        return None

    def write(self, packet: bytes) -> int:
        if len(packet) < 5 or packet[:2] != b"\xff\xff":
            raise ProtocolError("Demo received an invalid request")
        command = packet[3]
        mechanism_commands = {
            CMD_GRAB_ROTATE,
            CMD_CURSOR,
            CMD_SIDE_GRIPS,
            CMD_GRIPPER_LIFT,
            CMD_PUMPS,
            CMD_VALVES,
            CMD_GRIPPER_VERTICAL,
            CMD_BLOCKS_SHIFT,
            CMD_BLOCKS_ROTATE_MASK,
            CMD_GRIPPER_SLIDE,
        }
        if command in (CMD_IMPELLER_START, CMD_IMPELLER_STOP, CMD_IMPELLER_DUTY, CMD_IMPELLER_STATUS):
            payload = packet[4:-1]
            if command == CMD_IMPELLER_START:
                self.impeller_target = payload[0]
                self.impeller_on = True
            elif command == CMD_IMPELLER_DUTY:
                self.impeller_target = payload[0]
            elif command == CMD_IMPELLER_STOP:
                self.impeller_on = False
            applied = self.impeller_target if self.impeller_on else 0
            data = bytes((1, 2 if self.impeller_on else 0, applied, self.impeller_target))
        elif command == CMD_GET_AX_ANGLE:
            dynamixel_id = packet[4]
            angle_deg = self._ax_angles.get(dynamixel_id, 150)
            data = b"\x00" + struct.pack("<H", angle_deg)
        elif command == CMD_SET_AX_ANGLE:
            dynamixel_id = packet[4]
            angle_deg = struct.unpack("<H", packet[5:7])[0]
            self._ax_angles[dynamixel_id] = angle_deg
            data = b""
        elif command in mechanism_commands:
            data = b"\x01"
        elif command in (CMD_GET_COORD, CMD_GET_SPEED):
            data = struct.pack("<fff", 0.0, 0.0, 0.0)
        elif command == CMD_GET_BATTERY:
            data = struct.pack(
                "<I4HHHBB", 12500, 3800, 3800, 3800, 3800, 15200, 820, 76, 4
            )
        elif command == CMD_GET_WHEEL_TICKS:
            data = b"\x01" + struct.pack("<iii", 100, 101, 102)
        elif command == CMD_GET_WHEEL_PATH:
            data = b"\x01" + struct.pack("<fff", 0.1, 0.1, 0.1)
        elif command in (CMD_GET_TOGGLE, CMD_GET_KEY):
            data = b"\x01"
        elif command == CMD_GET_DEBUG:
            data = b"Robot systems: all OK".ljust(32, b"\x00")
        elif command == CMD_GET_SWITCHES:
            data = bytes(9)
        elif command == CMD_FREE_WHEELS:
            data = b"\x01"
        else:
            data = b""
        self._rx.extend(make_packet(command, data))
        return len(packet)

    def read(self, size: int) -> bytes:
        if not self._rx:
            return b""
        data = bytes(self._rx[:size])
        del self._rx[:size]
        return data
