# Thin wrapper over CMake presets: make, make release, make flash, ...

REMOTE ?= reset10

.PHONY: all debug release check check-release flash flash-release flash-remote reset monitor clean help

all: debug

debug release:
	cmake --preset $@
	cmake --build --preset $@

check: debug
	cmake --build --preset debug --target check

check-release: release
	cmake --build --preset release --target check

flash: debug
	cmake --build --preset debug --target flash

flash-release: release
	cmake --build --preset release --target flash

flash-remote: release
	scp build/release/firmware.hex $(REMOTE):~/firmware.hex
	ssh $(REMOTE) 'cd ~ && ./flash_stm32.sh'

reset:
	cmake --preset debug
	cmake --build --preset debug --target reset

monitor:
	python3 tools/monitor_stm32.py

clean:
	rm -rf build

help:
	@echo "make [debug]        build Debug (build/debug)"
	@echo "make release        build Release (build/release)"
	@echo "make check          verify Debug image; check-release for Release"
	@echo "make flash          build and flash Debug via OpenOCD; flash-release for Release"
	@echo "make flash-remote   build Release, copy to $(REMOTE) and run flash_stm32.sh there"
	@echo "make reset          reset the MCU"
	@echo "make monitor        read-only SWD runtime check"
	@echo "make clean          remove build/"
