#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
printf '%s  %s\n' 605a75866c901f07e4dc9f8565cb4b75aaa2b80584b0d8aa8fce960f1ef97bad stepper3-spi.hex | sha256sum -c -
st-flash --hot-plug --format ihex write stepper3-spi.hex
openocd -f interface/stlink-dap.cfg -f target/stm32f4x.cfg -c init -c 'reset_config none' -c 'cortex_m reset_config vectreset' -c 'reset run' -c shutdown
sleep 1
exec panel/.venv/bin/python panel/stepper3_check.py
