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

Пресет `bench` — Debug без остановки по таймеру матча (`EUROBOT_MATCH_TIMEOUT=OFF`).
Пресет `mechanism` — только терминал и `GrailGripperTask` (`make flash-mechanism`).

### Захваты грааля (PCA9685 CH13/CH14)

Задача `GrailGripperTask` ([grail_gripper_task.c](Core/OS_Tasks/grail_gripper_task.c)) управляет двумя серво на плате PCA9685 (I2C2, PB10/PB11, адрес `0x40`):

| Команда | Код | Серво |
|---|---|---|
| `BOARD_TOGGLE_GRAIL_PICK_GRIPPER` | `0x11` | PCA9685 CH13 |
| `BOARD_TOGGLE_GRAIL_SHELF_GRIPPER` | `0x12` | PCA9685 CH14 |

Данных у команд нет, ответ — `0x01`. Каждая команда переключает серво между
закрытым и открытым углом (`GRAIL_*_ANGLE` в задаче); первая после старта открывает.
Если запись по I2C не прошла, угол не меняется, и следующая команда повторяет тот же поворот.

Лифт грааля на Stepper 2 — `BOARD_GRAIL_LIFT_UP` (`0x13`) и `BOARD_GRAIL_LIFT_DOWN` (`0x14`):
один байт — ход в мм от текущего положения. Ответ приходит после хода (не дольше ~5 с):
`0x01` — проехал, `0x00` — таймаут 4 с, нижний концевик или лифт ещё занят.
Нижний концевик (GPIO28/PC14, замыкает на 3.3 В) останавливает ход вниз.
Шаговик настроен и двигается как вертикаль гриппера на Stepper 1: полный шаг, 1100 мА (вниз 320 мА),
25 шагов/мм, делители 64/10, разгон 200 мс; после хода драйвер выключается.
Пересчёт `GRAIL_MM_TO_STEPS` и скорость не откалиброваны под механику лифта.
Пока едет вертикаль гриппера (Stepper 1), лифт грааля не запускается и отвечает `0x00`, и наоборот.

### Гриппер (перенесено с lymunt-2026)

| Команда | Код | Данные | Механизм |
|---|---|---|---|
| `BOARD_GRIPPER_VERTICAL` | `0x15` | 1 байт: высота, мм от положения при включении | Stepper 1: полный шаг, 25 шагов/мм, вниз на 320 мА, таймаут 4 с, после хода драйвер выключается |
| `BOARD_TOGGLE_CURSOR` | `0x16` | 1 байт: 1 — горизонтально (140°), 0 — вертикально (80°) | динамиксель ID 13 |
| `BOARD_GRIPPER_SLIDE` | `0x17` | 4 байта int32: мм по рейке от стартового положения | динамиксель-шестерня ID 106, 148° + 3,979°/мм, 0..300° |

Ответ — один байт после хода: `0x01` — дошёл, `0x00` — таймаут, ошибка шины, гриппер занят
или (для вертикали) едет лифт грааля. При старте курсор встаёт вертикально, шестерня — в 148°.
Пульт: `U`/`J` — гриппер вверх/вниз на `--gripper-step` мм, `H` — курсор, `K`/`L` — рейка на `--rack-step` мм.

Клавиатурный пульт `./start.sh`: `1`/`2` — захваты грааля, `3`/`4` — лифт вверх/вниз на `--grail-step` мм (по умолчанию 10).

Импеллер (турбинка): подключение, команды и проверка — в [docs/USAGE_RU.md](docs/USAGE_RU.md#импеллер-турбинка), проверка логики — `python3 tools/test_impeller.py`.

История изменений, а также подробности переноса, результаты проверок и известные ограничения описаны в [docs/CHANGELOG.md](docs/CHANGELOG.md), краткая инструкция — в [docs/USAGE_RU.md](docs/USAGE_RU.md).
