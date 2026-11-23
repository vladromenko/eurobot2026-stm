#!/usr/bin/env bash
# Start keyboard teleoperation (tools/teleop.py) over USART1.
# Usage: ./start.sh [/dev/PORT] [teleop.py options...]
#   ./start.sh                                   keyboard control (T/Y impeller on/off, </> its PWM duty -/+5 %,
#                                                1/2 grail grippers, 3/4 grail lift up/down)
#   ./start.sh --impeller on                    one impeller command and exit (duty set in the firmware, 75 % after reset)
#   ./start.sh --impeller on --impeller-duty 60  duty 60 % = 2 V on the filter (3.3 V x duty)
#   ./start.sh --impeller off
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
    PORTS=()
    # On Linux /dev/serial/by-id names the same adapter as ttyUSB*, so prefer it to avoid duplicates.
    for candidate in /dev/serial/by-id/*; do
        [[ -e "$candidate" ]] && PORTS+=("$candidate")
    done
    if [[ ${#PORTS[@]} -eq 0 ]]; then
        # USB-UART adapters show up as usbserial/usbmodem/wchusbserial (macOS) or ttyUSB/ttyACM (Linux)
        for candidate in /dev/cu.usbserial* /dev/cu.usbmodem* /dev/cu.wchusbserial* /dev/cu.SLAB_USBtoUART* /dev/ttyUSB* /dev/ttyACM*; do
            [[ -e "$candidate" ]] && PORTS+=("$candidate")
        done
    fi
    if [[ ${#PORTS[@]} -eq 0 ]]; then
        echo "USB-UART адаптер не найден. Доступные порты:" >&2
        "$PYTHON" "$ROOT/tools/teleop.py" --list >&2
        echo "Укажите порт явно: ./start.sh /dev/PORT" >&2
        exit 1
    fi
    if [[ ${#PORTS[@]} -gt 1 ]]; then
        echo "Найдено несколько портов: ${PORTS[*]}" >&2
        echo "Укажите порт явно: ./start.sh /dev/PORT" >&2
        exit 1
    fi
    PORT="${PORTS[0]}"
fi

echo "Порт: $PORT"
exec "$PYTHON" "$ROOT/tools/teleop.py" --port "$PORT" "$@"
