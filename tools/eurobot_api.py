#!/usr/bin/env python3
"""Reusable upper-level API for the Eurobot 2025 main board."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
import struct
import threading
import time
from typing import Protocol


MAX_PAYLOAD = 32


class Command(IntEnum):
    PING = 0x00
    AX_GET_ANGLE = 0x02
    AX_SET_ANGLE = 0x03
    SET_SPEED = 0x04
    GET_SPEED = 0x05
    SET_COORD = 0x06
    GET_COORD = 0x07
    GET_BATTERY_STATE = 0x08
    SET_BMS_CALIBRATION = 0x09
    TOGGLE_GRIPPER = 0x0A
    POWER_ON_PUMP = 0x0B
    POWER_OFF_PUMP = 0x0C


class EurobotAPIError(RuntimeError):
    """Base class for communication and protocol errors."""


class ResponseTimeout(EurobotAPIError):
    pass


class ProtocolError(EurobotAPIError):
    pass


class ByteStream(Protocol):
    def read(self, size: int = 1) -> bytes: ...
    def write(self, data: bytes) -> int | None: ...
    def flush(self) -> None: ...


@dataclass(frozen=True)
class Velocity:
    dx: float
    dy: float
    dphi: float


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    phi: float


@dataclass(frozen=True)
class BatteryState:
    power_mw: int
    cell_voltage_mv: tuple[int, ...]
    voltage_mv: int
    current_ma: int
    state_of_charge_percent: int


def encode_frame(command: int | Command, payload: bytes = b"") -> bytes:
    """Encode one HL-RTU frame used by USART1 in the 2025 firmware."""
    command_value = int(command)
    if not 0 <= command_value <= 0xFF:
        raise ValueError("command must fit in one byte")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload is longer than {MAX_PAYLOAD} bytes")
    body = bytes((len(payload) + 2, command_value)) + payload
    checksum = (~sum(body)) & 0xFF
    return b"\xFF\xFF" + body + bytes((checksum,))


def decode_frame(frame: bytes) -> tuple[int, bytes]:
    """Validate a complete HL-RTU frame and return command and payload."""
    if len(frame) < 5 or frame[:2] != b"\xFF\xFF":
        raise ProtocolError("invalid frame header")
    pdu_length = frame[2]
    if pdu_length < 2 or pdu_length > MAX_PAYLOAD + 2:
        raise ProtocolError("invalid PDU length")
    if len(frame) != pdu_length + 3:
        raise ProtocolError("frame length does not match PDU length")
    if sum(frame[2:]) & 0xFF != 0xFF:
        raise ProtocolError("checksum mismatch")
    return frame[3], frame[4:-1]


def _three_floats(values: tuple[float, float, float], name: str) -> bytes:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} values must be finite")
    return struct.pack("<fff", *values)


class EurobotAPI:
    """Synchronous, thread-safe API over an already opened byte stream.

    The stream normally is ``serial.Serial`` configured as 921600 8N1. Keeping
    stream ownership outside this class also makes it usable with UART bridges,
    test doubles and a future TCP-to-UART service.
    """

    def __init__(self, stream: ByteStream, timeout: float = 0.3):
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        self._stream = stream
        self._timeout = timeout
        self._lock = threading.Lock()

    def _read_exact(self, size: int, deadline: float) -> bytes:
        result = bytearray()
        while len(result) < size and time.monotonic() < deadline:
            chunk = self._stream.read(size - len(result))
            if chunk:
                result.extend(chunk)
        if len(result) != size:
            raise ResponseTimeout(f"received {len(result)} of {size} bytes")
        return bytes(result)

    def _read_response(self, expected_command: int, deadline: float) -> bytes:
        previous = b""
        while time.monotonic() < deadline:
            current = self._stream.read(1)
            if previous == b"\xFF" and current == b"\xFF":
                length = self._read_exact(1, deadline)[0]
                if not 2 <= length <= MAX_PAYLOAD + 2:
                    previous = b""
                    continue
                frame = b"\xFF\xFF" + bytes((length,)) + self._read_exact(length, deadline)
                command, payload = decode_frame(frame)
                if command == expected_command:
                    return payload
                previous = b""
                continue
            if current:
                previous = current
        raise ResponseTimeout(f"no response to command 0x{expected_command:02X}")

    def request(
        self,
        command: int | Command,
        payload: bytes = b"",
        *,
        response_length: int | None = None,
    ) -> bytes:
        """Send a command and wait for a response with the same command code."""
        command_value = int(command)
        with self._lock:
            self._stream.write(encode_frame(command_value, payload))
            self._stream.flush()
            response = self._read_response(command_value, time.monotonic() + self._timeout)
        if response_length is not None and len(response) != response_length:
            raise ProtocolError(
                f"command 0x{command_value:02X}: expected {response_length} response bytes, "
                f"received {len(response)}"
            )
        return response

    def ping(self) -> float:
        """Return round-trip time in milliseconds."""
        started = time.monotonic()
        self.request(Command.PING, response_length=0)
        return (time.monotonic() - started) * 1000.0

    def set_velocity(self, velocity: Velocity) -> None:
        payload = _three_floats((velocity.dx, velocity.dy, velocity.dphi), "velocity")
        self.request(Command.SET_SPEED, payload, response_length=0)

    def stop(self) -> None:
        self.set_velocity(Velocity(0.0, 0.0, 0.0))

    def get_velocity(self) -> Velocity:
        values = struct.unpack("<fff", self.request(Command.GET_SPEED, response_length=12))
        return Velocity(*values)

    def set_pose(self, pose: Pose) -> None:
        payload = _three_floats((pose.x, pose.y, pose.phi), "pose")
        self.request(Command.SET_COORD, payload, response_length=0)

    def get_pose(self) -> Pose:
        values = struct.unpack("<fff", self.request(Command.GET_COORD, response_length=12))
        return Pose(*values)

    def get_ax_angle(self, device_id: int) -> tuple[int, int]:
        if not 0 <= device_id <= 0xFF:
            raise ValueError("device_id must fit in one byte")
        status, angle = struct.unpack(
            "<BH", self.request(Command.AX_GET_ANGLE, bytes((device_id,)), response_length=3)
        )
        return status, angle

    def set_ax_angle(self, device_id: int, angle: int) -> None:
        if not 0 <= device_id <= 0xFF:
            raise ValueError("device_id must fit in one byte")
        if not 0 <= angle <= 0xFFFF:
            raise ValueError("angle must fit in uint16")
        self.request(Command.AX_SET_ANGLE, struct.pack("<BH", device_id, angle), response_length=0)

    def get_battery_state(self) -> BatteryState:
        values = struct.unpack("<I4HHHBB", self.request(Command.GET_BATTERY_STATE, response_length=18))
        power, *tail = values
        cells = tuple(tail[:4])
        voltage, current, state_of_charge, cell_count = tail[4:]
        if cell_count > len(cells):
            raise ProtocolError(f"firmware reported unsupported cell count {cell_count}")
        return BatteryState(power, cells[:cell_count], voltage, current, state_of_charge)

    def set_bms_calibration(self, register: int, value: int) -> None:
        if not 0 <= register <= 0xFF or not 0 <= value <= 0xFFFF:
            raise ValueError("register/value do not fit the firmware fields")
        self.request(
            Command.SET_BMS_CALIBRATION,
            struct.pack("<BH", register, value),
            response_length=0,
        )

    def set_gripper(self, state: int) -> None:
        if not 0 <= state <= 0xFF:
            raise ValueError("gripper state must fit in one byte")
        self.request(Command.TOGGLE_GRIPPER, bytes((state,)), response_length=0)

    def set_pump(self, enabled: bool) -> None:
        command = Command.POWER_ON_PUMP if enabled else Command.POWER_OFF_PUMP
        self.request(command, response_length=0)


def open_serial_api(port: str, timeout: float = 0.3) -> tuple[EurobotAPI, object]:
    """Open the standard UART transport and return API plus owned serial port."""
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("PySerial is required: python -m pip install pyserial") from error
    stream = serial.Serial(
        port,
        921600,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0.01,
        write_timeout=1,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )
    stream.reset_input_buffer()
    return EurobotAPI(stream, timeout), stream
