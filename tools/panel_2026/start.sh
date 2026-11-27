#!/usr/bin/env bash
set -Eeuo pipefail
cd "$(dirname "$0")"

for arg in "$@"; do
  case "$arg" in
    --demo|--help|-h|--self-test) exec python3 -u robot_console_2026.py "$@" ;;
  esac
done

serial_device="${SERIAL_PORT:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}"
if [[ ! -e "$serial_device" ]]; then
  shopt -s nullglob
  candidates=(/dev/ttyUSB* /dev/ttyACM*)
  if [[ -z "${SERIAL_PORT:-}" && ${#candidates[@]} -eq 1 ]]; then
    serial_device="${candidates[0]}"
  else
    echo "Укажите адаптер: SERIAL_PORT=/dev/ttyUSB0 bash start.sh" >&2
    exit 2
  fi
fi

python_bin=.venv/bin/python
if [[ ! -x "$python_bin" && -x ../test_low_2026_final/.venv/bin/python ]]; then
  python_bin=../test_low_2026_final/.venv/bin/python
fi
if [[ ! -x "$python_bin" ]]; then python3 -m venv .venv; fi
if ! "$python_bin" -c 'import serial' 2>/dev/null; then
  "$python_bin" -m pip install -r requirements.txt
fi

stopped=()
restore_bridge() {
  for name in "${stopped[@]}"; do docker start "$name" >/dev/null || true; done
}
trap restore_bridge EXIT
if command -v docker >/dev/null; then
  for name in communication eurobot27-hw_bridge-1; do
    if [[ "$(docker inspect -f '{{.State.Running}}' "$name" 2>/dev/null || true)" == true ]]; then
      docker stop "$name" >/dev/null
      stopped+=("$name")
    fi
  done
fi

"$python_bin" -u robot_console_2026.py --port "$serial_device" --confirm-2026 "$@"
