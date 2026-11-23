"""Read-only telemetry and diagnostic packets."""

from __future__ import annotations

import struct

from .command_codes import (
    CMD_GET_BATTERY,
    CMD_GET_COORD,
    CMD_GET_DEBUG,
    CMD_GET_KEY,
    CMD_GET_SPEED,
    CMD_GET_SWITCHES,
    CMD_GET_TOGGLE,
    CMD_GET_WHEEL_PATH,
    CMD_GET_WHEEL_TICKS,
)
from .protocol import ProtocolError


class DiagnosticCommands:
    def telemetry(self) -> tuple[tuple[float, ...], tuple[float, ...], tuple[int, ...]]:
        coord_raw = self.command(CMD_GET_COORD, action="Telemetry / pose")
        speed_raw = self.command(CMD_GET_SPEED, action="Telemetry / speed")
        switches_raw = self.command(CMD_GET_SWITCHES, action="Telemetry / switches")
        if len(coord_raw) != 12 or len(speed_raw) != 12 or len(switches_raw) != 9:
            raise ProtocolError("Unexpected telemetry response length")
        return (
            struct.unpack("<fff", coord_raw),
            struct.unpack("<fff", speed_raw),
            tuple(switches_raw),
        )

    def diagnostics(self) -> dict[str, object]:
        battery_raw = self.command(CMD_GET_BATTERY, action="Diagnostics / battery")
        ticks_raw = self.command(CMD_GET_WHEEL_TICKS, action="Diagnostics / wheel ticks")
        path_raw = self.command(CMD_GET_WHEEL_PATH, action="Diagnostics / wheel paths")
        side_raw = self.command(CMD_GET_TOGGLE, action="Diagnostics / field side")
        key_raw = self.command(CMD_GET_KEY, b"\x00", action="Diagnostics / start cord")
        debug_raw = self.command(CMD_GET_DEBUG, action="Diagnostics / firmware message")
        if len(battery_raw) != 18:
            raise ProtocolError("Unexpected battery response length")
        if len(ticks_raw) != 13 or len(path_raw) != 13:
            raise ProtocolError("Unexpected wheel diagnostics response length")
        if len(side_raw) != 1 or len(key_raw) != 1:
            raise ProtocolError("Unexpected digital-state response length")
        power, c1, c2, c3, c4, voltage, current, soc, cells = struct.unpack(
            "<I4HHHBB", battery_raw
        )
        return {
            "battery": {
                "power_mw": power,
                "cells_mv": (c1, c2, c3, c4),
                "voltage_mv": voltage,
                "current_ma": current,
                "soc_percent": soc,
                "cell_count": cells,
            },
            "wheel_ticks_ok": bool(ticks_raw[0]),
            "wheel_ticks": struct.unpack("<iii", ticks_raw[1:]),
            "wheel_paths_ok": bool(path_raw[0]),
            "wheel_paths_m": struct.unpack("<fff", path_raw[1:]),
            "field_side": "left" if side_raw[0] else "right",
            "start_cord_attached": bool(key_raw[0]),
            "firmware_message": debug_raw.rstrip(b"\x00").decode(
                "utf-8", errors="replace"
            ),
        }
