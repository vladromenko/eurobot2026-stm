# Changelog

## 2026-10-27 — HwInstance: единая точка доступа к периферии

- Добавлены `Core/App/Include/hw_instance.h` и `Core/App/hw_instance.c`: `struct hw_instance_s` хранит указатели на все драйверы периферии (Stepper, L6474H, PWM, SPI GPO, WS2812B, USARTx, DynamixelMaster, Serial, HL_RTU, Dynamixel AX/MX, Maxon, BMS, RC_Servo, H_Bridge, Button) и таблицу `bus_devices`. Экземпляр объявлен `static const` и лежит во Flash; доступ — только через `const struct hw_instance_s* GetHwInstance(void)`.
- Код приложения (`Core/App`, `Core/OS_Tasks`) обращается к железу через `GetHwInstance()`: `hw->maxon->Set_TPM(hw->bus_devices[MAXON_MOTOR_1].usart, ...)`. Драйверы внутри BSP/Lib по-прежнему используют друг друга напрямую, чтобы нижние слои не зависели от App.
- Для SPI GPO добавлен интерфейс `struct spi_gpo_s` / `SPI_GPO` (`SetOutputPin`, `ResetOutputPin`) в стиле остальных драйверов; функции `SPI_GPO_SetOutputPin/ResetOutputPin` и `SPI_GPO_REG` сохранены.
- Сборка: 78 единиц компиляции, число предупреждений GCC не изменилось (141).

## 2026-10-27 — управление насосами только с телеоперации

- `DemoTask` больше не создаётся в `main.c`: режим `SPI_GPO_DEMO` больше не включает выходы `SPI_POWER_OUTPUT_1`–`SPI_POWER_OUTPUT_4` сам по себе. Исходник `demo_task.c` оставлен для ручных проверок. `make monitor` теперь ожидает не менее 7 задач (6 прикладных + idle).
- Команды `BOARD_POWER_ON_PUMP` (0x0B) и `BOARD_POWER_OFF_PUMP` (0x0C) теперь управляют всеми насосами на выходах `SPI_POWER_OUTPUT_1`–`SPI_POWER_OUTPUT_4`; раньше они затрагивали только выходы 1–2, а `BOARD_POWER_OFF_PUMP` по ошибке их включала.
- После старта все силовые выходы выключены (`SPI_GPO_ResetOutputPin(SPI_GPO_ALL)` в `main.c`), насосы включаются только командой `BOARD_POWER_ON_PUMP` (0x0B), например клавишей P в `tools/teleop.py`.
- Добавлены `tools/teleop.py` (управление с клавиатуры по USART1) и `start.sh` (запуск с автоматическим venv и поиском порта).

## 2026-10-27 — тест насосов Power Output 1–4

- `DemoTask` уже создавалась в `main.c`; в ней включён режим `SPI_GPO_DEMO`.
- Режим `STEPPER_DEMO` отключён, чтобы во время проверки насосов не запускались шаговые двигатели.
- Выходы `SPI_POWER_OUTPUT_1`–`SPI_POWER_OUTPUT_4` одновременно включаются на 250 мс и выключаются на 250 мс в непрерывном цикле.

## 2026-10-27 — перенос прошивки Eurobot 2025 с Keil на GCC/CMake

### Отчёт о переносе

Дата проверки: 27 октября 2026 года. Целевая ветка: `gcc-cmake-2025` репозитория `vladromenko/eurobot2026-stm`.

#### Итог и границы проверки

Перенос системы сборки выполнен и проверен чистой компиляцией Debug/Release и статическим аудитом полученных образов. Запись на STM32 и аппаратная проверка робота в рамках этой работы не выполнялись.

Итоговая ветка содержит только CMake/GCC-путь сборки. Каталоги Keil/ARMCC и старые результаты MDK-ARM не включены. Исходная прошивка 2025 года не изменялась на месте: работа выполнена в отдельной Git-ветке.

#### Проверка, не существовал ли перенос раньше

На компьютере найдены и проверены следующие копии:

- `/Users/vladromenko/Desktop/Eurobot/2025 г./eurobot_2025_main_board-master`;
- `/Users/vladromenko/Desktop/Eurobot/ORIGINAL_ARCHIVE/2025 г./eurobot_2025_main_board-master`;
- `/Users/vladromenko/Desktop/Eurobot/01_FIRMWARE_2025/PROJECT/eurobot_2025_main_board`;
- `/Users/vladromenko/Downloads/eurobot_2025_main_board-master`.

Все четыре копии содержали Keil-проект `MDK-ARM/Main_Board.uvprojx` и не содержали CMake/Makefile/отчёта о миграции. Git-метаданных в этих каталогах не было. Готовый перенос прошивки 2025 года не найден.

#### Исходный target

Единственный target `Main_Board`:

| Параметр | Исходный Keil | GCC/CMake |
|---|---|---|
| MCU | STM32F407VGTx | STM32F407xx |
| CPU | Cortex-M4F, Thumb | `-mcpu=cortex-m4 -mthumb` |
| FPU | FPv4-SP-D16 | `-mfpu=fpv4-sp-d16` |
| Float ABI | hard float | `-mfloat-abi=hard` |
| Flash | `0x08000000`, 1 МБ | без изменений |
| SRAM | `0x20000000`, 128 КБ | 127 КБ data/heap + 1 КБ stack |
| CCM | `0x10000000`, 64 КБ, не использовалась | описана, использование запрещено ASSERT до добавления инициализации |
| Defines | `USE_FULL_LL_DRIVER`, `ARM_MATH_CM4`, `__FPU_PRESENT=1U` | те же + `STM32F407xx` |
| Startup | ARMASM | GNU CMSIS startup |
| FreeRTOS port | RVDS/ARM_CM4F | GCC/ARM_CM4F |
| Исходники target | 76 | те же 76 с двумя toolchain-заменами |

В итоговой сборке 77 единиц компиляции: 76 единиц target и отдельный `cmake/gcc_runtime.c` для `_init`/`_fini` при `-nostartfiles`.

#### Что было добавлено

- `CMakeLists.txt`, `CMakePresets.json`;
- `cmake/arm-none-eabi.cmake` с поиском тулчейна в `PATH`, `ARM_TOOLCHAIN_BIN`, STM32CubeIDE и STM32CubeCLT;
- `cmake/sources.cmake` с явным списком исходников;
- `cmake/STM32F407VG_FLASH.ld`;
- `cmake/gcc_runtime.c`;
- `cmake/check_firmware.py`;
- `cmake/openocd.cmake`;
- `Makefile` и конфигурации `.vscode/`;
- `tools/monitor_stm32.py` и `tools/serial_read_test.py`;
- README, инструкция и CHANGELOG с этим отчётом.

#### Изменения исходного кода из-за компилятора

Прикладной код перенесён из архива 2025 года. Потребовались только две совместимые правки:

1. В `Core/App/terminal_cmd_handlers.c` добавлен отсутствовавший include `spi_gpo.h`. ARMCC принимал вызов без видимого прототипа, а GCC 14 в режиме C99 останавливал сборку на неявном объявлении функции.
2. В `Core/BSP/l6474_drvr.c` удалена Keil-специфичная `#pragma O0`. Эквивалентная оптимизация `-O0` назначена этому файлу через CMake во всех конфигурациях.

Дополнительно линковщик использует `-fno-strict-aliasing`, как в переносе 2026 года, чтобы не менять поведение существующего кода с доступом к данным через указатели другого типа.

#### Удалённые зависимости Keil

- `MDK-ARM/` и ARMASM startup;
- `ARMCC_Include/`;
- FreeRTOS-порт RVDS и остальные неиспользуемые порты;
- неиспользуемые startup для ARM/IAR;
- неиспользуемые файлы CMSIS-DSP, HAL и LL;
- старые Keil build-артефакты.

Удаление относится только к итоговой ветке. Исходные локальные архивы не изменялись.

#### Проверка сборки

Выполнены команды:

```bash
cmake --preset debug
cmake --build --preset debug --target clean
cmake --build --preset debug
cmake --build --preset debug --target check
cmake --preset release
cmake --build --preset release
cmake --build --preset release --target check
```

| Сборка | text | data | bss с резервами | Flash image | Результат |
|---|---:|---:|---:|---:|---|
| Debug `-O0` | 46 572 | 2 520 | 17 188 | 49 100 | build + check PASS |
| Release `-Os` | 26 292 | 2 472 | 17 180 | 28 772 | build + check PASS |

Аудит подтвердил:

- отсутствие дублированных единиц компиляции;
- обязательные CPU/FPU/ABI-флаги и defines;
- ARM32 little-endian hard-float ELF;
- таблицу из 98 векторов с начальным MSP `0x20020000`;
- 17 сильных обработчиков прерываний;
- отсутствие пересечений `.data`, `.bss`, heap и stack;
- отсутствие неожиданного использования CCM;
- запись HEX только в диапазон Flash;
- совпадение данных HEX и BIN, включая заполнение промежутков `0xFF`.

#### Предупреждения GCC

Чистая Debug-сборка выдаёт 145 предупреждений: 65 `unused-parameter`, 64 `comment`, 6 `sign-compare`, 3 `implicit-fallthrough`, 2 `misleading-indentation`, 1 `sequence-point` и 4 linker-предупреждения о различии размера `wchar_t` между приложением и библиотеками GNU.

Предупреждения не скрывались через `-w` или массовые `-Wno-*`. `-fshort-wchar` сохранён для соответствия ABI исходного ARMCC-проекта; wide-character API в текущем приложении не используется.

`sequence-point` в `robot_behavior_routine.c` указывает на неопределённый порядок изменения `strategy`. Это существующая проблема логики 2025 года; перенос не выбирает новое поведение без аппаратной спецификации.

#### Инструменты прошивки

`make flash` и `make flash-release` используют OpenOCD + ST-LINK, записывают `firmware.elf`, выполняют verify и software reset. При нескольких адаптерах используется `OPENOCD_ADAPTER_SERIAL`.

`make flash-remote` сохраняет рабочий процесс ветки 2026 года: Release HEX копируется на `REMOTE` (по умолчанию `reset10`) и там запускается `~/flash_stm32.sh`.

`make monitor` ничего не записывает в MCU. Он сначала сравнивает Flash с локальным `firmware.bin`, затем читает `SystemCoreClock`, состояние scheduler, число задач, tick и программную тень `SPI_GPO_REG`.

#### Что ещё требуется проверить на стенде

1. Сделать backup Flash целевой платы перед первой записью.
2. Выполнить `make flash`, убедиться в `Verified OK` и корректном reset.
3. Запустить `make monitor`: ожидаются `SystemCoreClock = 168000000`, scheduler = 1, не менее 7 задач и tick около 1000/с.
4. Проверить UART1 921600 8N1 read-only командами из `tools/serial_read_test.py`.
5. С поднятыми колёсами и доступным аварийным отключением проверить GPIO, SPI GPO, PWM, энкодеры, Maxon, Dynamixel, BMS, шаговые приводы и WS2812B.
6. Сравнить направления, масштабы одометрии, команды захвата и аварийную остановку со старой Keil-прошивкой.
7. Отдельно квалифицировать Release после прохождения Debug.

Успешный `check` подтверждает структуру и непротиворечивость образа, но не доказывает корректность поведения робота на аппаратуре.

### Система сборки

- Проект переведён с Keil MDK-ARM/ARMCC 5 на CMake, Ninja и `arm-none-eabi-gcc`.
- Состав единственного Keil-target `Main_Board` извлечён из `Main_Board.uvprojx`: все 76 исходных единиц перенесены в явный список `cmake/sources.cmake` без `GLOB`.
- ARMASM startup заменён на GNU startup STM32F407 из той же поставки CMSIS.
- FreeRTOS-порт `portable/RVDS/ARM_CM4F` заменён на `portable/GCC/ARM_CM4F`; ядро и `FreeRTOSConfig.h` сохранены.
- Добавлены GNU linker script, минимальный runtime с `_init`/`_fini`, toolchain-файл и CMake-пресеты Debug/Release.
- Сохранены параметры Cortex-M4F: Thumb, FPv4-SP-D16, hard-float ABI, short enums, 16-битный `wchar_t`, unsigned `char`, `STM32F407xx` и прежние LL/CMSIS defines.
- Сохранена карта памяти: Flash 1 МБ с `0x08000000`, SRAM 128 КБ, отдельный стек 1 КБ в верхней части SRAM и неиспользуемая CCM 64 КБ.
- Для `l6474_drvr.c` оптимизация `-O0` теперь задаётся CMake; Keil-специфичная `#pragma O0` удалена.
- В `terminal_cmd_handlers.c` явно подключён `spi_gpo.h`, потому что современный GCC/C99 не допускает неявное объявление `SPI_GPO_SetOutputPin`.

### Только CMake/GCC

- Keil-проект, ARMCC-заголовки, RVDS-порт и старые бинарные артефакты не включены в итоговую ветку.
- Неиспользуемые порты FreeRTOS, варианты startup и неиспользуемые файлы CMSIS/HAL/LL удалены из итогового дерева.
- Прикладной код 2025 года сохранён. Код 2026 года не переносился в прошивку 2025 года.

### Инструменты

- Добавлен `Makefile`: `make`, `release`, `check`, `check-release`, `flash`, `flash-release`, `flash-remote`, `reset`, `monitor`, `clean`, `help`.
- Добавлена прошивка и reset через OpenOCD + ST-LINK с выбором адаптера по серийному номеру.
- Добавлен `cmake/check_firmware.py`: проверка состава компиляции, CPU/FPU/ABI-флагов, ELF, 98 векторов, 17 обработчиков прерываний, границ Flash/RAM/stack и согласованности HEX/BIN.
- Добавлены read-only инструменты SWD-мониторинга и проверки UART1, а также конфигурации VS Code/Cortex-Debug.

### Проверено

- Чистая Debug-сборка и `check`: успешно; Flash image 49 100 байт.
- Release-сборка и `check`: успешно; Flash image 28 772 байта.
- В обеих конфигурациях аудит подтвердил 77 единиц компиляции (76 перенесённых единиц target + `cmake/gcc_runtime.c`), 98 векторов и 17 сильных ISR.
- Прошивка на реальную плату и поведение периферии не проверялись в рамках переноса.

### Проверка существующего переноса

Проверены четыре найденные локальные копии прошивки 2025 года. Во всех была только Keil-сборка (`MDK-ARM`); `CMakeLists.txt`, `Makefile`, `CHANGELOG` и `MIGRATION_REPORT` отсутствовали. Ранее выполненный перенос 2025 года на GCC/CMake не найден.
