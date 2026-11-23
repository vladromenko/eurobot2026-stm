#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
printf '%s  %s\n' a515867c430918fb06a1faa830cf8744953583140f8649b586ec82971602ec58 stepper2-spi.hex | sha256sum -c -
st-flash --hot-plug --format ihex write stepper2-spi.hex
openocd -f interface/stlink-dap.cfg -f target/stm32f4x.cfg -c init -c 'reset_config none' -c 'cortex_m reset_config vectreset' -c 'reset run' -c shutdown
sleep 1
exec panel/.venv/bin/python panel/stepper_check.py
