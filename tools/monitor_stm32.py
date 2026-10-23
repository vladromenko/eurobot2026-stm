#!/usr/bin/env python3
"""Read-only SWD checks for the exact local firmware image. No reset or MCU write."""
import argparse
import hashlib
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile
import time


def cache_values(path):
    result = {}
    for line in path.read_text().splitlines():
        if not line.startswith(('#', '//')) and ':' in line and '=' in line:
            key, value = line.split('=', 1)
            result[key.split(':', 1)[0]] = value
    return result


def get_symbols(nm, elf):
    out = subprocess.check_output([nm, '-n', str(elf)], text=True)
    symbols = {}
    for line in out.splitlines():
        match = re.fullmatch(r'([0-9a-fA-F]+)\s+\S\s+(\S+)', line.strip())
        if match:
            symbols[match[2]] = int(match[1], 16)
    return symbols


def upload(serial, address, size, path):
    # Plain init attaches without reset or halt; memory is read while the MCU runs.
    command = ['openocd', '-f', 'interface/stlink.cfg']
    if serial:
        command += ['-c', 'adapter serial ' + serial]
    command += ['-f', 'target/stm32f4x.cfg', '-c', 'init',
                '-c', f'dump_image {path} {hex(address)} {size}', '-c', 'exit']
    result = subprocess.run(command, capture_output=True, text=True, timeout=30)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    content = path.read_bytes()
    if len(content) != size:
        raise RuntimeError(f'Неполное чтение: {len(content)} вместо {size} байт')
    return content


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, default=Path.cwd())
    parser.add_argument('--build', default='build/debug')
    parser.add_argument('--serial', default='', help='ST-LINK serial number')
    parser.add_argument('--samples', type=int, default=24)
    parser.add_argument('--interval', type=float, default=0.07)
    args = parser.parse_args()
    if args.samples < 4 or args.interval < 0:
        parser.error('samples >= 4, interval >= 0')
    root = args.project.resolve()
    build = root / args.build
    cache = cache_values(build / 'CMakeCache.txt')
    symbols = get_symbols(cache['CMAKE_NM'], build / 'firmware.elf')
    names = ['SystemCoreClock', 'PCA9685_InitStatus', 'uxCurrentNumberOfTasks',
             'xSchedulerRunning', 'xTickCount', 'SPI_GPO_REG']
    for name in names + ['DemoTask']:
        if name not in symbols:
            raise RuntimeError('Символ отсутствует в ELF: ' + name)
    for name in names:
        if not 0x20000000 <= symbols[name] <= 0x2001fffc:
            raise RuntimeError('Неожиданный SRAM-адрес символа ' + name)
    image = (build / 'firmware.bin').read_bytes()
    if not image or len(image) > 0x100000:
        raise RuntimeError('Неверный размер firmware.bin')
    with tempfile.TemporaryDirectory(prefix='eurobot-read-test-') as temporary:
        temp = Path(temporary)
        stored = upload(args.serial, 0x08000000, len(image), temp / 'image.bin')
        if stored != image:
            raise RuntimeError('На плате другая прошивка. SRAM-тест не запускался: сначала flash и verify для этой сборки.')
        print('PASS: Flash совпадает с локальным firmware.bin, SHA256 ' + hashlib.sha256(image).hexdigest(), flush=True)
        clock_address = symbols['SystemCoreClock']
        clock = struct.unpack('<I', upload(args.serial, clock_address, 4, temp / 'clock.bin'))[0]
        print(f'SystemCoreClock = {clock} (это переменная прошивки, не независимое измерение кварца)', flush=True)
        sample_names = [n for n in names if n != 'SystemCoreClock']
        low = min(symbols[n] for n in sample_names)
        high = max(symbols[n] for n in sample_names) + 4
        mask = 0x003c0000  # SPI_POWER_OUTPUT_1 | 2 | 3 | 4
        samples = []
        print('sample   tick       tasks scheduler PCA9685 SPI_GPO_REG masked_1_4', flush=True)
        for index in range(args.samples):
            begin = time.monotonic()
            contents = upload(args.serial, low, high - low, temp / 'sram.bin')
            end = time.monotonic()
            values = {name: struct.unpack_from('<I', contents, symbols[name] - low)[0] for name in sample_names}
            values['time'] = (begin + end) / 2
            samples.append(values)
            pca_status = struct.unpack('<i', struct.pack('<I', values['PCA9685_InitStatus']))[0]
            print(f"{index+1:2}  {values['xTickCount']:10}  {values['uxCurrentNumberOfTasks']:3}    {values['xSchedulerRunning']:3}      {pca_status:3}  "
                  f"0x{values['SPI_GPO_REG']:08x} 0x{values['SPI_GPO_REG'] & mask:08x}", flush=True)
            if index + 1 < args.samples:
                time.sleep(args.interval)
        dt = samples[-1]['time'] - samples[0]['time']
        ticks = (samples[-1]['xTickCount'] - samples[0]['xTickCount']) & 0xffffffff
        rate = ticks / dt if dt else 0
        states = {sample['SPI_GPO_REG'] & mask for sample in samples}
        checks = {
            'SystemCoreClock = 168000000': clock == 168000000,
            'PCA9685 initialization succeeded': all(x['PCA9685_InitStatus'] == 0 for x in samples),
            'FreeRTOS scheduler running': all(x['xSchedulerRunning'] == 1 for x in samples),
            'At least 9 tasks (8 application + idle)': all(x['uxCurrentNumberOfTasks'] >= 9 for x in samples),
            'Tick progressing near 1000/s': 700 < rate < 1300,
            'Demo outputs 1-4 observed OFF and ON': 0 in states and mask in states,
        }
        print(f'Tick rate estimate: {rate:.1f}/s; SWD snapshots are not simultaneous across words.')
        for title, ok in checks.items():
            print(('PASS: ' if ok else 'FAIL: ') + title)
        print('PCA9685 status: 0=OK, -2=timeout, -3=no ACK, -4=bus error, -5=arbitration lost, -6=overrun.')
        print('SPI_GPO_REG is a software shadow. Confirm physical outputs with an oscilloscope/logic analyzer.')
        if not all(checks.values()):
            raise RuntimeError('Не все проверки прошли. Сохраните вывод; не запускайте дополнительные движения.')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
        print('FAIL: ' + str(error), file=sys.stderr)
        sys.exit(1)
