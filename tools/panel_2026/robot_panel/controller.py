"""Shared serial transaction layer composed with subsystem command groups."""

from __future__ import annotations

import time

from .chassis import ChassisCommands
from .command_codes import CMD_GET_SPEED, CMD_PING, COMMAND_NAMES
from .impeller import ImpellerCommands
from .diagnostics import DiagnosticCommands
from .gripper import GripperCommands
from .logs import TraceLog
from .protocol import ProtocolError, make_packet, read_response
from .vacuum import VacuumCommands


class FirmwareLinkError(RuntimeError):
    """USART1 did not prove that the expected 2026 firmware is running."""


class Robot2026(
    ImpellerCommands,
    ChassisCommands,
    GripperCommands,
    VacuumCommands,
    DiagnosticCommands,
):
    """One serial connection plus commands grouped in subsystem mixins."""

    def __init__(self, port, trace: TraceLog):
        self.port = port
        self.trace = trace
        self.action_context: str | None = None

    def command(
        self,
        command: int,
        data: bytes = b"",
        response_timeout: float | None = None,
        action: str | None = None,
    ) -> bytes:
        packet = make_packet(command, data)
        old_timeout = self.port.timeout
        old_write_timeout = self.port.write_timeout
        started = time.monotonic()
        raw_response = b""
        try:
            if response_timeout is not None:
                self.port.timeout = response_timeout
                self.port.write_timeout = response_timeout
            self.port.reset_input_buffer()
            self.port.write(packet)
            self.port.flush()
            response = read_response(self.port, response_timeout)
            raw_response = response.raw
            if response.command != command:
                raise ProtocolError(
                    f"Unexpected response 0x{response.command:02X}; "
                    f"expected 0x{command:02X}"
                )
            self.trace.add(
                action=action
                or self.action_context
                or COMMAND_NAMES.get(command, f"0x{command:02X}"),
                command=command,
                tx=packet,
                rx=raw_response,
                result="OK",
                latency_ms=(time.monotonic() - started) * 1000,
                detail=(
                    f"payload: {response.data.hex(' ')}"
                    if response.data
                    else "acknowledged"
                ),
            )
            return response.data
        except Exception as exc:
            raw_response = getattr(exc, "raw", raw_response)
            self.trace.add(
                action=action
                or self.action_context
                or COMMAND_NAMES.get(command, f"0x{command:02X}"),
                command=command,
                tx=packet,
                rx=raw_response,
                result="ERROR",
                latency_ms=(time.monotonic() - started) * 1000,
                detail=str(exc),
            )
            raise
        finally:
            self.port.timeout = old_timeout
            self.port.write_timeout = old_write_timeout

    def ping(
        self,
        action: str = "Check connection",
        response_timeout: float | None = None,
    ) -> None:
        self.command(CMD_PING, action=action, response_timeout=response_timeout)

    def verify_firmware(self, attempts: int = 3, timeout: float = 1.0) -> None:
        """Prove the exact 2026 HL-RTU link before any actuator command."""
        last_error: Exception | None = None
        for attempt in range(1, attempts + 1):
            try:
                self.ping(
                    action=f"Firmware check / PING {attempt}/{attempts}",
                    response_timeout=timeout,
                )
                speed = self.command(
                    CMD_GET_SPEED,
                    response_timeout=timeout,
                    action="Firmware check / GET_SPEED",
                )
                if len(speed) != 12:
                    raise ProtocolError(
                        f"GET_SPEED returned {len(speed)} bytes instead of 12"
                    )
                return
            except (OSError, TimeoutError, ProtocolError) as exc:
                last_error = exc
                time.sleep(0.15)

        raise FirmwareLinkError(
            "Нет ответа от прошивки Eurobot 2026 после трёх попыток. "
            "Параметры программы уже совпадают с прошивкой: USART1, 921600, "
            "8N1, PING FF FF 02 00 FD. Проверьте общий GND, "
            "CP2102 TX→PA10, CP2102 RX←PA9 и что программа STM32 запущена. "
            f"Последняя ошибка: {last_error}"
        ) from last_error
