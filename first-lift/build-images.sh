#!/usr/bin/env bash
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
repo=$(git -C "$here" rev-parse --show-toplevel)
tmp=$(mktemp -d)
trap 'rm -rf -- "$tmp"' EXIT
src="$tmp/src"
mkdir "$src"
git -C "$repo" archive 0d073ce | tar -x -C "$src"
cd "$src"
for variant in first-lift stepper2-spi stepper3-spi; do
    patch="$variant.patch"
    if [[ "$variant" == stepper3-spi ]]; then patch=stepper3-channel.patch; fi
    git apply --whitespace=nowarn "$here/$patch"
    cmake --preset bench
    cmake --build --preset bench
    python3 cmake/check_firmware.py --build build/bench
    cp build/bench/firmware.hex "$tmp/$variant.hex"
done
(cd "$tmp" && sha256sum -c "$here/images.sha256")
for variant in first-lift stepper2-spi stepper3-spi; do
    cp "$tmp/$variant.hex" "$here/$variant.hex"
done
