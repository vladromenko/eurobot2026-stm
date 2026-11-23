# Eurobot 2026 — STM32F407 firmware

Прошивка основной платы Eurobot 2026 для STM32F407VGTx. Собирается через CMake, Ninja и `arm-none-eabi-gcc`, прошивается и отлаживается через OpenOCD + ST-LINK.

- STM32F407VGTx, Cortex-M4F, hard float; Flash 1 МБ, SRAM 128 КБ, CCM 64 КБ;
- FreeRTOS 10.4.3 LTS Patch 3 (только статическое выделение памяти);
- STM32 LL-драйверы, CMSIS-DSP (матрицы, sin/cos);
- PCA9685 `0x40` на I2C2: PB10/SCL, PB11/SDA, 100 кГц.

## Структура

| Путь | Назначение |
|---|---|
| `Core/App` | `main`, кинематика, одометрия, поведение робота, терминал, прерывания, `SystemInit` |
| `Core/BSP` | Периферия платы: GPIO, PWM, I2C2, SPI, USART, L6474, WS2812B |
| `Core/Lib` | Драйверы устройств: Dynamixel, Maxon, H-мост, RC-серво, PCA9685, BMS, кнопки, FIFO |
| `Core/OS_Tasks` | Задачи FreeRTOS (`template_task.c` — заготовка, не собирается) |
| `Drivers` | CMSIS, LL-драйверы STM32F4, CMSIS-DSP — только используемые файлы |
| `Middlewares/FreeRTOS-Kernel` | Ядро FreeRTOS, порт `GCC/ARM_CM4F`, `FreeRTOSConfig.h` в `include/` |
| `cmake/sources.cmake` | Явный список исходников и include-путей |
| `cmake/arm-none-eabi.cmake` | Toolchain-файл |
| `cmake/STM32F407VG_FLASH.ld` | Linker script |
| `cmake/openocd.cmake` | Цели `flash` и `reset` |
| `cmake/check_firmware.py` | Проверка образа: векторы, ISR, память, HEX/BIN |
| `tools/` | Мониторинг по SWD и проверка UART-терминала |
| `.vscode/` | Задачи сборки/прошивки, конфигурация отладки Cortex-Debug |

Новый `.c` файл нужно добавить в `EUROBOT_SOURCES` в [cmake/sources.cmake](cmake/sources.cmake).

## Требования

CMake ≥ 3.24, Ninja, ARM GNU Toolchain (`arm-none-eabi-gcc`, `arm-none-eabi-gdb`), OpenOCD. На macOS:

```bash
brew install cmake ninja open-ocd
```

Тулчейн ищется в `PATH`, в `ARM_TOOLCHAIN_BIN` и в установленных STM32CubeIDE/CubeCLT.

## Командная строка

Короткий вариант — [Makefile](Makefile) поверх CMake-пресетов:

```bash
make                # Debug
make release
make check          # check-release для Release
make flash          # flash-release для Release
make flash-remote   # Release → scp на reset10 → ~/flash_stm32.sh (REMOTE=host для другого)
make reset
make monitor
make clean
make help
```

То же напрямую через CMake:

```bash
cmake --preset debug                          # или release
cmake --build --preset debug
cmake --build --preset debug --target check   # проверка образа
cmake --build --preset debug --target flash   # сборка + прошивка + verify + reset
cmake --build --preset debug --target reset
```

Результаты — в `build/debug/` (`build/release/`): `firmware.elf`, `.bin`, `.hex`, `.map`.

### Длительное управление на стенде

Для пульта `robot_console_2026.py` без остановки по истечении 99 секунд:

```bash
cmake --preset bench
cmake --build --preset bench --target check
```

Образ: `build/bench/firmware.hex`. Пресет `bench` задаёт
`EUROBOT_MATCH_TIMEOUT=OFF`: таймер конца матча не отключает приводы и не
блокирует команды, даже если вход чеки постоянно показывает «вынута».
Остальные обработчики команд и чеки продолжают работать как прежде.
Пульты и UART-протокол менять не требуется.

В обычных сборках таймер включён по умолчанию. Для явного включения:
`cmake --preset debug -DEUROBOT_MATCH_TIMEOUT=ON` (или `release`).
Выбор Debug/Release сам по себе не задаёт игровой или стендовый режим.
Состояние физической чеки следует проверять отдельно: её ложное определение
как вынутой этим изменением не исправляется.

Проверка таймера на компьютере без подключения к плате:
`python3 tools/test_game_timeout.py` (требуется C-компилятор `cc`).
Она запускает исходный GameTask с моделируемыми часами на 180 секунд
для обоих режимов и трёх состояний входа чеки.

### Захваты грааля (PCA9685 CH13/CH14)

Задача `GrailGripperTask` ([grail_gripper_task.c](Core/OS_Tasks/grail_gripper_task.c)) управляет двумя серво на плате PCA9685 (I2C2, адрес `0x40`):

| Команда | Код | Серво |
|---|---|---|
| `BOARD_TOGGLE_GRAIL_PICK_GRIPPER` | `0x37` | PCA9685 CH13 |
| `BOARD_TOGGLE_GRAIL_SHELF_GRIPPER` | `0x38` | PCA9685 CH14 |

Данных у команд нет. Ответ — один байт: `0x01` — принято, `0x00` — отклонено, потому что связь отключена таймером матча. Каждая команда переключает
серво между закрытым и открытым углом: захват 0° ↔ 110°, полка 0° ↔ 180°; первая после старта открывает. Если запись по I2C
не прошла, угол не меняется, и следующая команда повторяет тот же поворот.
Каналы CH13/CH14 `gripper_task` не использует (у него CH1–CH12).

Лифт грааля на Stepper 2 — `BOARD_GRIPPER_GRAIL_VERTICAL` (`0x39`), по аналогии с
`BOARD_GRIPPER_VERTICAL`: один байт — высота в мм от положения при включении.
Ответ приходит после хода (не дольше ~11 с): `0x01` — дошёл, `0x00` — авария
L6474, таймаут 10 с, связь отключена или лифт ещё занят. После хода драйвер держит
вал. Пересчёт `GRAIL_MM_TO_STEPS` (25 полных шагов на мм × 1/16) и скорость 6 мм/с
не откалиброваны под механику лифта.

Клавиатурный пульт: `./start.sh` (клавиши `1` и `2` — захваты грааля,
`U`/`J`/`H` — лифт грааля, `W/A/S/D` — езда после `M`).

Если подключено несколько ST-LINK: `cmake --preset debug -DOPENOCD_ADAPTER_SERIAL=<serial>`.

## VS Code

Рекомендуемые расширения (`.vscode/extensions.json`): Cortex-Debug и Task Buttons.

- **Кнопки в статус-баре** (Task Buttons): Debug, Release, Flash, Flash Release.
- **Cmd+Shift+B** — сборка Debug; остальное — `Terminal → Run Task`.
- **F5** — «Debug (OpenOCD)»: собирает Debug, прошивает, останавливается на `main`, показывает задачи FreeRTOS.
- «Attach (OpenOCD)» — подключение к работающей плате без перепрошивки.
- IntelliSense берёт флаги из `build/debug/compile_commands.json`, поэтому сначала соберите Debug.

Подробности, история исправлений и чек-лист стендовых проверок: [docs/USAGE_RU.md](docs/USAGE_RU.md).
