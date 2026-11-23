"""HL-RTU framing and serial-port discovery.

Packet format: ``FF FF | length | command | data | checksum``.
"""

from __future__ import annotations

from dataclasses import dataclass
import time

from .command_codes import MAX_PAYLOAD


class ProtocolError(RuntimeError):
    """The STM32 returned a malformed or unexpected HL-RTU response."""


class ResponseTimeout(TimeoutError):
    """The response deadline expired; ``raw`` contains every received byte."""

    def __init__(self, message: str, raw: bytes):
        super().__init__(message)
        self.raw = raw


@dataclass(frozen=True)
class Response:
    command: int
    data: bytes
    raw: bytes


def checksum(length: int, command: int, data: bytes) -> int:
    return (~(length + command + sum(data))) & 0xFF


def make_packet(command: int, data: bytes = b"") -> bytes:
    if len(data) > MAX_PAYLOAD:
        raise ValueError("HL-RTU payload is too long")
    length = len(data) + 2
    return bytes((0xFF, 0xFF, length, command)) + data + bytes(
        (checksum(length, command, data),)
    )


def _read_byte(port, deadline: float, raw: bytearray) -> int:
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        old_timeout = port.timeout
        try:
            # Short reads preserve one overall deadline and make partial RX visible.
            port.timeout = min(0.10, max(0.001, remaining))
            chunk = port.read(1)
        finally:
            port.timeout = old_timeout
        if chunk:
            raw.extend(chunk)
            return chunk[0]
    shown = raw.hex(" ") or "<empty>"
    raise ResponseTimeout(
        f"STM32 did not answer before the deadline; RX: {shown}", bytes(raw)
    )


def read_response(port, timeout: float | None = None) -> Response:
    response_timeout = timeout if timeout is not None else port.timeout
    if response_timeout is None or response_timeout <= 0:
        response_timeout = 1.0
    deadline = time.monotonic() + response_timeout
    raw = bytearray()
    previous = None
    while True:
        current = _read_byte(port, deadline, raw)
        if previous == 0xFF and current == 0xFF:
            break
        previous = current

    length = _read_byte(port, deadline, raw)
    if length < 2 or length > MAX_PAYLOAD + 2:
        raise ProtocolError(f"Invalid response length: {length}")

    body = bytes(_read_byte(port, deadline, raw) for _ in range(length))
    command = body[0]
    data = body[1:-1]
    received_crc = body[-1]
    expected_crc = checksum(length, command, data)
    packet = bytes(raw)
    if received_crc != expected_crc:
        raise ProtocolError(
            f"CRC mismatch: got 0x{received_crc:02X}, "
            f"expected 0x{expected_crc:02X}"
        )
    return Response(command, data, packet)


def require_pyserial():
    try:
        import serial
        from serial.tools import list_ports
    except ImportError as exc:
        raise SystemExit(
            "PySerial is not installed. Run: python3 -m pip install pyserial"
        ) from exc
    return serial, list_ports


def list_serial_ports() -> int:
    _, list_ports = require_pyserial()
    ports = list(list_ports.comports())
    if not ports:
        print("Serial ports were not found.")
        return 1
    for item in ports:
        print(f"{item.device}\t{item.description}\t{item.hwid}")
    return 0


def resolve_port(requested: str | None) -> str:
    if requested:
        return requested
    _, list_ports = require_pyserial()
    ports = list(list_ports.comports())
    if len(ports) == 1:
        return ports[0].device
    if not ports:
        raise SystemExit("No serial port found. Use --list-ports after connecting STM32.")
    names = ", ".join(item.device for item in ports)
    raise SystemExit(f"Several serial ports found: {names}. Select one with --port.")
