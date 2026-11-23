#!/usr/bin/env bash
set -euo pipefail
exec python3 - "$@" <<'PY'
import argparse
import hashlib
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--check-only', action='store_true')
args = parser.parse_args()
base = 0x08000000
image = (Path.home() / 'stm32_backups/stm32-before-bench-free-20261120-235556.bin').read_bytes()
if len(image) != 1048576 or hashlib.sha256(image).hexdigest() != 'd4c6bc7bde6a1afce657ea0b6fd59c86d0fa10e851ec54fb8896509a93705714':
    raise SystemExit('Ошибка проверки бэкапа.')

def swd(driver, commands):
    cmd = ['openocd', '-f', f'interface/{driver}.cfg', '-f', 'target/stm32f4x.cfg']
    for command in ['gdb_port disabled', 'tcl_port disabled', 'telnet_port disabled', 'init', 'halt', *commands, 'shutdown']:
        cmd.extend(['-c', command])
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if p.returncode or 'Error:' in p.stdout:
        raise SystemExit(p.stdout)

with tempfile.TemporaryDirectory() as temp:
    folder = Path(temp)
    def read_flash():
        out = folder / 'readback.bin'
        out.unlink(missing_ok=True)
        swd('stlink', [f'dump_image {out} {base:#x} {len(image)}'])
        data = out.read_bytes()
        if len(data) != len(image):
            raise SystemExit('Неполное чтение флеш-памяти.')
        return data

    data = read_flash()
    offset = 0
    changes = []
    for sector, size in enumerate([16384]*4 + [65536] + [131072]*7):
        old = data[offset:offset+size]
        expected = image[offset:offset+size]
        if old != expected:
            changes.append((sector, offset, expected, old == b'\xff'*size))
        offset += size
    print('Отличающиеся секторы:', [c[0] for c in changes], flush=True)
    if args.check_only:
        raise SystemExit(0)
    # Restore the vector table last. Keep the core halted throughout.
    commands = []
    for sector, offset, expected, blank in sorted(changes, key=lambda c: c[0] == 0):
        part = folder / f'sector{sector}.bin'
        part.write_bytes(expected)
        address = base + offset
        if not blank:
            commands.append(f'flash erase_sector 0 {sector} {sector}')
        if expected != b'\xff'*len(expected):
            commands.append(f'flash write_bank 0 {part} {offset:#x}')
        commands.append(f'verify_image {part} {address:#x} bin')
    if commands:
        swd('stlink-dap', commands)
        data = read_flash()
    if data != image:
        raise SystemExit('Содержимое памяти отличается от бэкапа. Не перезапускайте плату.')
    print('Бэкап восстановлен. Весь 1 МиБ совпадает. Контроллер остановлен.')
PY
