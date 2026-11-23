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
CMD_SET_SPEED = 0x35
CMD_GET_DIAGNOSTICS = 0x36
SERVO_PWM5 = 0x01
SERVO_PWM6 = 0x02
SERVO_BOTH = SERVO_PWM5 | SERVO_PWM6


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
    if len(data) != 15:
        raise ValueError("некорректная длина состояния")
    current, goal = struct.unpack_from("<ii", data, 1)
    state = {
        "flags": data[0],
        "current": current,
        "goal": goal,
        "left": data[9],
        "right": data[10],
        "left_target": data[11],
        "right_target": data[12],
        "stepper_speed": data[13],
        "servo_speed": data[14],
    }
    if show:
        print(
            f"Stepper2={current} -> {goal} шагов (скорость {state['stepper_speed']}/8), "
            f"PWM5={state['left']}° -> {state['left_target']}°, "
            f"PWM6={state['right']}° -> {state['right_target']}° "
            f"(скорость {state['servo_speed']}/8), initialized={bool(data[0] & 1)}, "
            f"moving={bool(data[0] & 2)}, PWM5_on={bool(data[0] & 4)}, "
            f"PWM6_on={bool(data[0] & 8)}"
        )
        if state["flags"] & 0x10:
            print("Stepper 2: драйвер не подтвердил готовность, импульсы STEP остановлены. Введите diag.")
    return state


def signed_step(parts: list[str], default_step: int) -> int:
    if len(parts) not in (2, 3) or parts[1] not in ("+", "-"):
        raise ValueError("укажите направление + или - и, при необходимости, размер шага")
    amount = default_step if len(parts) == 2 else int(parts[2])
    if amount <= 0:
        raise ValueError("размер шага должен быть положительным")
    return amount if parts[1] == "+" else -amount


def changed_speed(current: int, value: str) -> int:
    if value == "+":
        result = current + 1
    elif value == "-":
        result = current - 1
    else:
        result = int(value)
    if not 1 <= result <= 8:
        raise ValueError("уровень скорости должен быть в диапазоне 1..8")
    return result


def set_speed(port: serial.Serial, stepper_level: int, servo_level: int) -> None:
    check_result(request(port, CMD_SET_SPEED, bytes((stepper_level, servo_level))))


def show_diagnostics(port: serial.Serial) -> None:
    data = request(port, CMD_GET_DIAGNOSTICS)
    if len(data) != 32 or data[:2] != bytes((0xD5, 0xA6)):
        raise ValueError("некорректный ответ аппаратной диагностики")

    driver_status, = struct.unpack_from("<H", data, 2)
    driver_config, = struct.unpack_from("<H", data, 6)
    tim1_ccer, tim5_ccer, pwm5_ccr, pwm6_ccr, spi_gpo = struct.unpack_from(
        "<IIIII", data, 8
    )
    step_mode = data[4] & 0x07
    current_ma = (data[5] + 1) * 31.25
    active_faults = []
    if not driver_status & 0x1000:
        active_faults.append("OCD")
    if not driver_status & 0x0800:
        active_faults.append("TH_SD")
    if not driver_status & 0x0400:
        active_faults.append("TH_WRN")
    if not driver_status & 0x0200:
        active_faults.append("UVLO")
    if driver_status & 0x0100:
        active_faults.append("WRONG_CMD")
    if driver_status & 0x0080:
        active_faults.append("NOTPERF_CMD")
    print(
        f"FW_DIAG=D5A6, L6474_STATUS=0x{driver_status:04X} "
        f"(HiZ={bool(driver_status & 0x0001)}), STEP_MODE={step_mode}/7, "
        f"TVAL=0x{data[5]:02X} (~{current_ma:.0f} mA), CONFIG=0x{driver_config:04X}"
    )
    print(f"L6474_FAULTS={','.join(active_faults) if active_faults else 'none'}")
    print(
        f"TIM1_CCER=0x{tim1_ccer:08X} (Stepper2_CH2={bool(tim1_ccer & 0x10)}), "
        f"TIM5_CCER=0x{tim5_ccer:08X} "
        f"(PWM5_CH1={bool(tim5_ccer & 0x01)}, PWM6_CH2={bool(tim5_ccer & 0x10)})"
    )
    print(
        f"TIM5_CCR1={pwm5_ccr} us, TIM5_CCR2={pwm6_ccr} us, "
        f"SPI_GPO=0x{spi_gpo:06X} (L6474_NRST2={bool(spi_gpo & (1 << 9))})"
    )
    print(
        f"PA0/PA1_MODER=0x{data[28] & 0x0F:X}, PA0/PA1_AF=0x{data[29]:02X}, "
        f"PE11_MODE={data[30]}, PE11_AF={data[31]}"
    )


def print_help() -> None:
    print("Команды:")
    print("  status                 показать состояние")
    print("  diag                   аппаратные регистры Stepper2 и PWM5/PWM6")
    print("  zero                   принять текущее положение за 0")
    print("  stepper +/- [steps]    Stepper 2 вперёд/назад; по умолчанию 10 шагов")
    print("  pwm5 +/- [degrees]     PWM5 в обе стороны; по умолчанию 1 градус")
    print("  pwm6 +/- [degrees]     PWM6 в обе стороны; по умолчанию 1 градус")
    print("  speed stepper +/-      увеличить/уменьшить скорость Stepper 2")
    print("  speed servos +/-       увеличить/уменьшить скорость PWM5/PWM6")
    print("  speed stepper 1..8     установить уровень скорости")
    print("  speed servos 1..8      установить уровень скорости")
    print("  move <steps>           абсолютная позиция Stepper 2")
    print("  servos <pwm5> <pwm6>  абсолютные углы 0..180 градусов")
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
    elif name == "diag" and len(parts) == 1:
        show_diagnostics(port)
    elif name == "zero" and len(parts) == 1:
        check_result(request(port, CMD_SET_ZERO))
    elif name == "move" and len(parts) == 2:
        position = int(parts[1])
        check_result(request(port, CMD_SET_POSITION, struct.pack("<i", position)))
    elif name == "stepper":
        state = get_state(port, show=False)
        base = state["goal"] if state["flags"] & 2 else state["current"]
        position = base + signed_step(parts, 10)
        if not -200000 <= position <= 200000:
            raise ValueError("позиция Stepper 2 выходит за диапазон ±200000")
        check_result(request(port, CMD_SET_POSITION, struct.pack("<i", position)))
    elif name in ("pwm5", "pwm6"):
        state = get_state(port, show=False)
        left = state["left_target"]
        right = state["right_target"]
        delta = signed_step(parts, 1)
        if name == "pwm5":
            left += delta
        else:
            right += delta
        if not (0 <= left <= 180 and 0 <= right <= 180):
            raise ValueError("угол выходит за диапазон 0..180")
        servo_mask = SERVO_PWM5 if name == "pwm5" else SERVO_PWM6
        check_result(request(port, CMD_SET_SERVOS, bytes((left, right, servo_mask))))
    elif name == "servos" and len(parts) == 3:
        left, right = int(parts[1]), int(parts[2])
        if not (0 <= left <= 180 and 0 <= right <= 180):
            raise ValueError("углы должны быть в диапазоне 0..180")
        check_result(request(port, CMD_SET_SERVOS, bytes((left, right, SERVO_BOTH))))
    elif name == "speed" and len(parts) == 3 and parts[1] in ("stepper", "servos"):
        state = get_state(port, show=False)
        stepper_level = state["stepper_speed"] or 1
        servo_level = state["servo_speed"] or 1
        if parts[1] == "stepper":
            stepper_level = changed_speed(stepper_level, parts[2])
        else:
            servo_level = changed_speed(servo_level, parts[2])
        set_speed(port, stepper_level, servo_level)
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
