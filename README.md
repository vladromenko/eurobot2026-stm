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
| `BOARD_TOGGLE_GRAIL_PICK_GRIPPER` | `0x37` | PCA9685 CH13 |
| `BOARD_TOGGLE_GRAIL_SHELF_GRIPPER` | `0x38` | PCA9685 CH14 |

Данных у команд нет, ответ — `0x01`. Каждая команда переключает серво между
закрытым и открытым углом (`GRAIL_*_ANGLE` в задаче); первая после старта открывает.
Если запись по I2C не прошла, угол не меняется, и следующая команда повторяет тот же поворот.

Лифт грааля на Stepper 2 — `BOARD_GRIPPER_GRAIL_VERTICAL` (`0x39`): один байт — высота
в мм от положения при включении. Ответ приходит после хода (не дольше ~11 с): `0x01` — дошёл,
`0x00` — авария L6474, таймаут 10 с или лифт ещё занят. После хода драйвер держит вал.
Пересчёт `GRAIL_MM_TO_STEPS` и скорость не откалиброваны под механику лифта.
`BOARD_STEPPER_TEST` (`0x0D`) тоже управляет Stepper 2 — не используйте их одновременно.

Клавиатурный пульт `./start.sh`: `1`/`2` — захваты грааля, `3`/`4`/`5` — лифт вверх/вниз/в 0.

Импеллер (турбинка): подключение, команды и проверка — в [docs/USAGE_RU.md](docs/USAGE_RU.md#импеллер-турбинка), проверка логики — `python3 tools/test_impeller.py`.

История изменений, а также подробности переноса, результаты проверок и известные ограничения описаны в [docs/CHANGELOG.md](docs/CHANGELOG.md), краткая инструкция — в [docs/USAGE_RU.md](docs/USAGE_RU.md).
