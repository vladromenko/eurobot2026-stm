#!/usr/bin/env bash
# Optional onboard operation with ST-LINK. Stop the panel and remove actuator power first.
set -Eeuo pipefail
cd "$(dirname "$0")"
case "${1:-}" in
  bench|match) image="pwm8_2026_${1}.hex" ;;
  *) echo "Usage: sudo bash flash_pwm8.sh bench|match" >&2; exit 2 ;;
esac
command -v openocd >/dev/null
sha256sum -c SHA256SUMS
backup=$(mktemp --suffix=.bin stm32-before-pwm8-XXXXXXXX)
config=(-f interface/stlink.cfg -f target/stm32f4x.cfg -c 'reset_config none' -c 'adapter speed 1000')
openocd "${config[@]}" -c init -c halt -c 'wait_halt 5000' \
  -c "dump_image $backup 0x08000000 0x100000" -c shutdown
[[ $(wc -c < "$backup") -eq 1048576 ]] || { echo "Incomplete backup: $backup" >&2; exit 1; }
openocd "${config[@]}" -c init -c halt -c 'wait_halt 5000' \
  -c "flash write_image erase $image" -c "verify_image $image" -c shutdown
echo "Verified $image. Backup: $backup. Power-cycle STM32 logic before running the panel."
