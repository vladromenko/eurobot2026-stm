# Eurobot 2025 — STM32F407 firmware

Прошивка основной платы Eurobot 2025 для STM32F407VGTx. Единственная поддерживаемая система сборки — CMake + Ninja + `arm-none-eabi-gcc`; прошивка и отладка выполняются через OpenOCD + ST-LINK.

## Требования

- CMake 3.24 или новее;
- Ninja;
- ARM GNU Toolchain (`arm-none-eabi-gcc`, `arm-none-eabi-gdb`);
- OpenOCD для целей `flash` и `reset`;
- Python 3 для проверки образа и вспомогательных инструментов.

На macOS CMake также ищет ARM GNU Toolchain внутри установленного STM32CubeIDE/STM32CubeCLT. Явный путь можно передать через `ARM_TOOLCHAIN_BIN`.

## Сборка

```bash
make                 # Debug
make release         # Release (-Os)
make check           # Debug + аудит образа
make check-release   # Release + аудит образа
```

Результаты находятся в `build/debug/` или `build/release/`: `firmware.elf`, `firmware.hex`, `firmware.bin`, `firmware.map`.

То же напрямую через CMake:

```bash
cmake --preset debug
cmake --build --preset debug
cmake --build --preset debug --target check
```

Список всех 76 исходников исходного Keil-target перенесён явно в `cmake/sources.cmake`. При добавлении нового `.c` файла его нужно добавить в `EUROBOT_SOURCES`.

## Прошивка и отладка

```bash
make flash           # Debug через OpenOCD
make flash-release   # Release через OpenOCD
make reset
make monitor         # read-only проверка Flash/FreeRTOS по SWD
```

Если подключено несколько ST-LINK:

```bash
cmake --preset debug -DOPENOCD_ADAPTER_SERIAL=<serial>
cmake --build --preset debug --target flash
```

`make flash-remote` собирает Release, копирует `firmware.hex` на `reset10` и запускает там `~/flash_stm32.sh`. Другой хост задаётся как `REMOTE=user@host`.

В `.vscode/` находятся задачи сборки/прошивки и конфигурации Cortex-Debug. IntelliSense использует `build/debug/compile_commands.json`, поэтому сначала нужно собрать Debug.

История изменений, а также подробности переноса, результаты проверок и известные ограничения описаны в [docs/CHANGELOG.md](docs/CHANGELOG.md), краткая инструкция — в [docs/USAGE_RU.md](docs/USAGE_RU.md).
