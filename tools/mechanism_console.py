#!/usr/bin/env python3
"""Пошаговый UART-пульт для Stepper 2 и сервоприводов PWM5/PWM6."""

from __future__ import annotations

import argparse
import struct
import sys

try:
    import serial
except ImportError:
    serial = None


CMD_SET_POSITION = 0x30
CMD_SET_SERVOS = 0x31
CMD_SET_ZERO = 0x32
CMD_STOP = 0x33
CMD_GET_STATE = 0x34


def make_packet(command: int, data: bytes = b"") -> bytes:
    length = len(data) + 2
    checksum = (~(length + command + sum(data))) & 0xFF
    return bytes((0xFF, 0xFF, length, command)) + data + bytes((checksum,))


def read_exact(port: serial.Serial, size: int) -> bytes:
    result = bytearray()
    while len(result) < size:
        block = port.read(size - len(result))
        if not block:
            raise TimeoutError("таймаут ответа платы")
        result.extend(block)
    return bytes(result)


def read_packet(port: serial.Serial) -> tuple[int, bytes]:
    previous = -1
    header_found = False
    while not header_found:
        value = read_exact(port, 1)[0]
        header_found = previous == 0xFF and value == 0xFF
        previous = value

    length = read_exact(port, 1)[0]
    if length < 2:
        raise ValueError(f"некорректная длина ответа: {length}")

    body = read_exact(port, length)
    command = body[0]
    data = body[1:-1]
    received_checksum = body[-1]
    expected_checksum = (~(length + command + sum(data))) & 0xFF
    if received_checksum != expected_checksum:
        raise ValueError("ошибка CRC ответа")
    return command, data


def request(port: serial.Serial, command: int, data: bytes = b"") -> bytes:
    port.reset_input_buffer()
    port.write(make_packet(command, data))
    port.flush()
    response_command, response_data = read_packet(port)
    if response_command != command:
        raise ValueError(
            f"ожидалась команда 0x{command:02X}, получена 0x{response_command:02X}"
        )
    return response_data


def check_result(data: bytes) -> None:
    descriptions = {0: "OK", 1: "некорректные данные или очередь занята", 2: "COMM отключен"}
    if len(data) != 1:
        raise ValueError("некорректный ответ команды")
    print(descriptions.get(data[0], f"неизвестный код {data[0]}"))


def get_state(port: serial.Serial, show: bool = True) -> dict[str, int]:
    data = request(port, CMD_GET_STATE)
    if len(data) != 11:
        raise ValueError("некорректная длина состояния")
    current, goal = struct.unpack_from("<ii", data, 1)
    state = {
        "flags": data[0],
        "current": current,
        "goal": goal,
        "left": data[9],
        "right": data[10],
    }
    if show:
        print(
            f"position={current}, goal={goal}, PWM5={state['left']}°, "
            f"PWM6={state['right']}°, initialized={bool(data[0] & 1)}, "
            f"moving={bool(data[0] & 2)}, servos={bool(data[0] & 4)}"
        )
    return state


def print_help() -> None:
    print("Команды:")
    print("  status                 показать состояние")
    print("  zero                   принять текущее положение за 0")
    print("  move <steps>           абсолютная позиция Stepper 2")
    print("  step <steps>           относительный шаг от текущей позиции")
    print("  servos <pwm5> <pwm6>  углы 0..180 градусов")
    print("  stop                   остановить мотор и снять PWM5/PWM6")
    print("  help                   эта справка")
    print("  quit                   выход")


def execute(port: serial.Serial, line: str) -> bool:
    parts = line.split()
    if not parts:
        return True

    name = parts[0].lower()
    if name == "status" and len(parts) == 1:
        get_state(port)
    elif name == "zero" and len(parts) == 1:
        check_result(request(port, CMD_SET_ZERO))
    elif name == "move" and len(parts) == 2:
        position = int(parts[1])
        check_result(request(port, CMD_SET_POSITION, struct.pack("<i", position)))
    elif name == "step" and len(parts) == 2:
        state = get_state(port, show=False)
        position = state["current"] + int(parts[1])
        check_result(request(port, CMD_SET_POSITION, struct.pack("<i", position)))
    elif name == "servos" and len(parts) == 3:
        left, right = int(parts[1]), int(parts[2])
        if not (0 <= left <= 180 and 0 <= right <= 180):
            raise ValueError("углы должны быть в диапазоне 0..180")
        check_result(request(port, CMD_SET_SERVOS, bytes((left, right))))
    elif name == "stop" and len(parts) == 1:
        check_result(request(port, CMD_STOP))
    elif name == "help" and len(parts) == 1:
        print_help()
    elif name in ("quit", "exit") and len(parts) == 1:
        return False
    else:
        raise ValueError("неизвестная команда или неверное число аргументов")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="UART-порт, например /dev/tty.usbserial-0001")
    parser.add_argument("--baud", type=int, default=921600, help="скорость UART (по умолчанию 921600)")
    args = parser.parse_args()

    if serial is None:
        raise SystemExit("Установите pyserial: python3 -m pip install pyserial")

    print("При запуске движение не выполняется. Введите help для списка команд.")
    with serial.Serial(args.port, args.baud, timeout=1.0, write_timeout=1.0) as port:
        running = True
        while running:
            try:
                running = execute(port, input("mechanism> ").strip())
            except (ValueError, TimeoutError, serial.SerialException) as error:
                print(f"Ошибка: {error}")
            except EOFError:
                running = False
    return 0


if __name__ == "__main__":
    sys.exit(main())
