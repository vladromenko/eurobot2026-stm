#!/usr/bin/env python3
"""Найти ID динамикселей на шине через существующую команду BOARD_AX_GetAngle (0x02).

Перебирает ID 0..253: для каждого шлёт AX_GetAngle и смотрит байт ошибки в ответе.
error == 0  -> на этом ID кто-то ответил. Прошивку менять/перешивать не нужно.

Пример:
    python tools/dynamixel_scan.py --list
    python tools/dynamixel_scan.py --port /dev/ttyUSB0
"""
import argparse
import sys
import time

AX_GET_ANGLE = 0x02
BOARD_PING = 0x00


def frame(command, payload=b''):
    body = bytes([len(payload) + 2, command]) + payload
    return b'\xff\xff' + body + bytes([(~sum(body)) & 0xff])


def read_exact(port, length, deadline):
    result = b''
    while len(result) < length and time.monotonic() < deadline:
        result += port.read(length - len(result))
    if len(result) != length:
        raise RuntimeError(f'Таймаут: получено {len(result)} из {length} байт')
    return result


def receive(port, command, deadline):
    previous = b''
    while time.monotonic() < deadline:
        current = port.read(1)
        if previous == b'\xff' and current == b'\xff':
            length_byte = read_exact(port, 1, deadline)
            length = length_byte[0]
            if length < 2 or length > 250:
                raise RuntimeError('Неверная длина ответа')
            rest = read_exact(port, length, deadline)
            packet = b'\xff\xff' + length_byte + rest
            if sum(packet[2:]) & 0xff != 0xff:
                raise RuntimeError('Неверная контрольная сумма: ' + packet.hex(' '))
            if rest[0] == command:
                return rest[1:-1], packet
            previous = b''
            continue
        previous = current
    raise RuntimeError('Нет ответа FF FF от платы; проверьте порт/PA9-PA10/питание')


def board_alive(port):
    """Проверка связи с платой: Ping (0x00) должен вернуть кадр-ответ."""
    for _ in range(5):
        port.reset_input_buffer()
        port.write(frame(BOARD_PING))
        port.flush()
        try:
            receive(port, BOARD_PING, time.monotonic() + 0.5)
            return True
        except RuntimeError:
            continue
    return False


def get_angle(port, servo_id):
    """Вернуть (error, angle) для одного ID, или None если плата не ответила кадром."""
    port.reset_input_buffer()
    port.write(frame(AX_GET_ANGLE, bytes([servo_id])))
    port.flush()
    try:
        payload, _ = receive(port, AX_GET_ANGLE, time.monotonic() + 0.5)
    except RuntimeError:
        return None
    if len(payload) != 3:
        return None
    error = payload[0]
    angle = payload[1] | (payload[2] << 8)
    return error, angle


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--list', action='store_true', help='показать последовательные порты')
    parser.add_argument('--port', help='порт USART1, например /dev/ttyUSB0')
    parser.add_argument('--min', type=int, default=0, help='начальный ID (по умолчанию 0)')
    parser.add_argument('--max', type=int, default=253, help='конечный ID (по умолчанию 253)')
    args = parser.parse_args()

    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        raise RuntimeError('Требуется PySerial: python -m pip install pyserial')

    if args.list:
        ports = list(list_ports.comports())
        for port in ports:
            print(port.device, port.description)
        if not ports:
            print('Портов не найдено. ST-LINK SWD это не USART1 — нужен USB-UART на PA9/PA10.')
        return

    if not args.port:
        parser.error('укажите --port из списка --list')

    found = []
    with serial.Serial(args.port, 921600, bytesize=8, parity='N', stopbits=1,
                       timeout=0.1, write_timeout=2) as port:
        print(f'Порт {args.port} открыт на 921600. Проверяю связь с платой (Ping)...', flush=True)
        if not board_alive(port):
            print('\nПлата НЕ отвечает на Ping. Динамиксель тут ни при чём — нет связи с платой.')
            print('Проверь:')
            print('  - тот ли это порт (это USB-UART на PA9/PA10, НЕ ST-Link SWD);')
            print('  - залита ли на плату рабочая прошивка и запущена ли (не в сбросе);')
            print('  - TX платы -> RX адаптера, RX платы -> TX адаптера, общая земля;')
            print('  - скорость USART1 в прошивке = 921600.')
            return
        print('Связь есть. Сканирую ID 0..{}...'.format(args.max), flush=True)

        for servo_id in range(args.min, args.max + 1):
            if servo_id % 16 == 0:
                print(f'  ...проверяю ID {servo_id}', flush=True)
            result = get_angle(port, servo_id)
            if result is None:
                continue
            error, angle = result
            if error == 0:
                found.append(servo_id)
                print(f'НАЙДЕН динамиксель: ID = {servo_id} (позиция {angle})', flush=True)

    if found:
        print(f'\nИтого найдено: {len(found)} -> ID {found}')
    else:
        print('\nНичего не найдено. Проверьте: питание серво, подключение к шине (USART3),')
        print('совпадение baudrate серво с мастером, и что прошивка на плате свежая.')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError) as error:
        print('FAIL: ' + str(error), file=sys.stderr)
        sys.exit(1)
