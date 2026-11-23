#!/usr/bin/env bash
set -Eeuo pipefail

cd "$(dirname "$0")"

DEFAULT_SERIAL_PORT="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
SERIAL_DEVICE="${SERIAL_PORT:-$DEFAULT_SERIAL_PORT}"
COMMUNICATION_WAS_RUNNING=0

if command -v docker >/dev/null 2>&1; then
  if [ "$(docker inspect -f '{{.State.Running}}' communication 2>/dev/null || true)" = "true" ]; then
    echo "Останавливаю контейнер communication, чтобы освободить STM32..."
    docker stop communication >/dev/null
    COMMUNICATION_WAS_RUNNING=1
  fi
fi

restore_communication() {
  if [ "$COMMUNICATION_WAS_RUNNING" = "1" ]; then
    echo "Возвращаю штатный контейнер communication..."
    docker start communication >/dev/null || true
  fi
}
trap restore_communication EXIT

if [ ! -e "$SERIAL_DEVICE" ]; then
  shopt -s nullglob
  SERIAL_CANDIDATES=(/dev/ttyUSB* /dev/ttyACM*)
  shopt -u nullglob
  if [ "${#SERIAL_CANDIDATES[@]}" -eq 1 ]; then
    SERIAL_DEVICE="${SERIAL_CANDIDATES[0]}"
    echo "Штатный путь не найден, использую $SERIAL_DEVICE"
  else
    echo "Не удалось однозначно найти STM32. Доступные USB-порты:"
    if [ "${#SERIAL_CANDIDATES[@]}" -eq 0 ]; then
      echo "  не найдены"
    else
      printf '  %s\n' "${SERIAL_CANDIDATES[@]}"
    fi
    echo
    echo "Укажите порт вручную:"
    echo "  SERIAL_PORT=/dev/ttyUSB0 bash start.sh"
    exit 2
  fi
fi

if [ ! -x .venv/bin/python ]; then
  echo "Создаю локальное окружение .venv..."
  python3 -m venv .venv
fi

# Всегда используем Python именно из .venv. Активировать окружение командой
# source не требуется: явный путь гарантирует запуск с тем же PySerial.
PYTHON_BIN=".venv/bin/python"
if ! "$PYTHON_BIN" -c "import serial" >/dev/null 2>&1; then
  echo "Устанавливаю PySerial в .venv..."
  "$PYTHON_BIN" -m pip install --disable-pip-version-check -r requirements.txt
fi

echo
echo "Запускаю терминальный пульт Eurobot 2026"
echo "STM32: $SERIAL_DEVICE"
echo "Рекомендуемый размер терминала: не меньше 104x34"
echo "SPACE — стоп базы, 0 — полная аварийная остановка, ESC — выход"
echo

"$PYTHON_BIN" -u robot_console_2026.py \
  --port "$SERIAL_DEVICE" \
  --confirm-2026 \
  "$@"
