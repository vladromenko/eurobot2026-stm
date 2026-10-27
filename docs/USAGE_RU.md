# Прошивка Eurobot 2025: сборка и загрузка

## Быстрый старт

```bash
make check
```

Команда настраивает Debug, собирает `firmware.elf/.hex/.bin/.map` и проверяет структуру образа. Release:

```bash
make check-release
```

## Путь сборки

```text
cmake/sources.cmake + CMakeLists.txt
        ↓
CMake preset → Ninja → arm-none-eabi-gcc
        ↓
GNU startup + GCC-порт FreeRTOS + linker script
        ↓
firmware.elf → firmware.hex / firmware.bin / firmware.map
        ↓
cmake/check_firmware.py
        ↓
OpenOCD → ST-LINK → STM32F407VGTx
```

## Основные команды

| Команда | Назначение |
|---|---|
| `make` | собрать Debug |
| `make release` | собрать Release |
| `make check` | собрать и проверить Debug |
| `make check-release` | собрать и проверить Release |
| `make flash` | записать Debug, verify, reset |
| `make flash-release` | записать Release, verify, reset |
| `make flash-remote` | отправить Release HEX на удалённый программатор |
| `make reset` | software reset через OpenOCD |
| `make monitor` | read-only проверка работающей прошивки по SWD |
| `make clean` | удалить каталог `build/` |

## Первая прошивка платы

Перед записью рекомендуется сохранить Flash:

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "init; dump_image backup-flash.bin 0x08000000 0x100000; exit"
```

Затем:

```bash
make flash
```

Ожидаемые сообщения OpenOCD: `Verified OK` и `Resetting Target`.

## Проверка после загрузки

```bash
make monitor
```

Проверка требует, чтобы Flash платы совпадал с локальным Debug-образом. Ожидается:

- `SystemCoreClock = 168000000`;
- FreeRTOS scheduler работает;
- создано не менее 8 задач с idle-задачей;
- tick увеличивается примерно на 1000 в секунду.

UART1 проверяется без команд движения:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install pyserial
python3 tools/serial_read_test.py --list
python3 tools/serial_read_test.py --port /dev/cu.НУЖНЫЙ_ПОРТ --command speed
```

## VS Code

- `Cmd+Shift+B` — Debug build;
- `Terminal → Run Task` — Release, Flash и Clean;
- `F5` — прошивка и запуск Cortex-Debug;
- `Attach (OpenOCD)` — подключение без перепрошивки.

Для IntelliSense сначала выполните `make`, чтобы появился `build/debug/compile_commands.json`.

## Ограничения

Сборка и статический аудит образов проверены. Фактический запуск прошивки 2025 года на плате после переноса, периферия и движение робота пока не квалифицированы. Полный чек-лист и известные предупреждения находятся в [MIGRATION_REPORT.md](../MIGRATION_REPORT.md).
