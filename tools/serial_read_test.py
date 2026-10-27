#!/usr/bin/env python3
"""Query existing read-only terminal commands over USART1; no actuator commands."""
import argparse
import math
import struct
import sys
import time


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
    # Search for frame header, bounded by deadline.
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
    raise RuntimeError('Нет заголовка ответа FF FF; проверьте порт/PA9/PA10/питание')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--port')
    parser.add_argument('--command', choices=['speed', 'coord'], default='speed')
    parser.add_argument('--count', type=int, default=5)
    args = parser.parse_args()
    if args.count < 1:
        parser.error('count >= 1')
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        raise RuntimeError('Требуется PySerial. Установите в отдельном venv: python -m pip install pyserial')
    if args.list:
        ports = list(list_ports.comports())
        for port in ports:
            print(port.device, port.description)
        if not ports:
            print('Последовательные порты не найдены. ST-LINK SWD сам по себе не является соединением с USART1.')
        return
    if not args.port:
        parser.error('укажите --port из списка --list')
    commands = {'speed': (0x05, 12), 'coord': (0x07, 12)}
    command, expected = commands[args.command]
    with serial.Serial(args.port, 921600, bytesize=8, parity='N', stopbits=1,
                       timeout=0.1, write_timeout=2, xonxoff=False, rtscts=False, dsrdtr=False) as port:
        port.reset_input_buffer()
        for index in range(args.count):
            request = frame(command)
            port.write(request)
            port.flush()
            payload, packet = receive(port, command, time.monotonic() + 3)
            if len(payload) != expected:
                raise RuntimeError(f'Длина данных {len(payload)} вместо {expected}')
            result = struct.unpack('<fff', payload)
            if not all(math.isfinite(x) for x in result):
                raise RuntimeError('NaN/Inf в ответе: ' + repr(result))
            print(f'PASS {index+1}: {args.command} = {result}; RX: {packet.hex(" ")}', flush=True)
            time.sleep(0.25)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError) as error:
        print('FAIL: ' + str(error), file=sys.stderr)
        sys.exit(1)
