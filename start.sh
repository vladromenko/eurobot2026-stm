#!/usr/bin/env bash
# Start keyboard teleoperation (tools/teleop.py) over USART1.
# Usage: ./start.sh [/dev/cu.PORT] [teleop.py options...]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$ROOT/.venv"
PYTHON="$VENV/bin/python3"

if [[ ! -x "$PYTHON" ]]; then
    echo "Создаю venv в $VENV"
    python3 -m venv "$VENV"
fi

if ! "$PYTHON" -c 'import serial' 2>/dev/null; then
    echo "Устанавливаю pyserial"
    "$PYTHON" -m pip install --quiet --disable-pip-version-check pyserial
fi

if [[ $# -gt 0 && "$1" == /dev/* ]]; then
    PORT="$1"
    shift
else
    # USB-UART adapters show up as usbserial/usbmodem/wchusbserial (macOS) or ttyUSB/ttyACM (Linux)
    PORTS=()
    for candidate in /dev/cu.usbserial* /dev/cu.usbmodem* /dev/cu.wchusbserial* /dev/cu.SLAB_USBtoUART* /dev/ttyUSB* /dev/ttyACM*; do
        [[ -e "$candidate" ]] && PORTS+=("$candidate")
    done
    if [[ ${#PORTS[@]} -eq 0 ]]; then
        echo "USB-UART адаптер не найден. Доступные порты:" >&2
        "$PYTHON" "$ROOT/tools/teleop.py" --list >&2
        echo "Укажите порт явно: ./start.sh /dev/cu.PORT" >&2
        exit 1
    fi
    if [[ ${#PORTS[@]} -gt 1 ]]; then
        echo "Найдено несколько портов: ${PORTS[*]}" >&2
        echo "Укажите порт явно: ./start.sh /dev/cu.PORT" >&2
        exit 1
    fi
    PORT="${PORTS[0]}"
fi

echo "Порт: $PORT"
exec "$PYTHON" "$ROOT/tools/teleop.py" --port "$PORT" "$@"
