# Отчёт о переносе Eurobot 2025 на CMake/GCC

Дата проверки: 27 октября 2026 года. Целевая ветка: `gcc-cmake-2025` репозитория `vladromenko/eurobot2026-stm`.

## Итог и границы проверки

Перенос системы сборки выполнен и проверен чистой компиляцией Debug/Release и статическим аудитом полученных образов. Запись на STM32 и аппаратная проверка робота в рамках этой работы не выполнялись.

Итоговая ветка содержит только CMake/GCC-путь сборки. Каталоги Keil/ARMCC и старые результаты MDK-ARM не включены. Исходная прошивка 2025 года не изменялась на месте: работа выполнена в отдельной Git-ветке.

## Проверка, не существовал ли перенос раньше

На компьютере найдены и проверены следующие копии:

- `/Users/vladromenko/Desktop/Eurobot/2025 г./eurobot_2025_main_board-master`;
- `/Users/vladromenko/Desktop/Eurobot/ORIGINAL_ARCHIVE/2025 г./eurobot_2025_main_board-master`;
- `/Users/vladromenko/Desktop/Eurobot/01_FIRMWARE_2025/PROJECT/eurobot_2025_main_board`;
- `/Users/vladromenko/Downloads/eurobot_2025_main_board-master`.

Все четыре копии содержали Keil-проект `MDK-ARM/Main_Board.uvprojx` и не содержали CMake/Makefile/отчёта о миграции. Git-метаданных в этих каталогах не было. Готовый перенос прошивки 2025 года не найден.

## Исходный target

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

## Что было добавлено

- `CMakeLists.txt`, `CMakePresets.json`;
- `cmake/arm-none-eabi.cmake` с поиском тулчейна в `PATH`, `ARM_TOOLCHAIN_BIN`, STM32CubeIDE и STM32CubeCLT;
- `cmake/sources.cmake` с явным списком исходников;
- `cmake/STM32F407VG_FLASH.ld`;
- `cmake/gcc_runtime.c`;
- `cmake/check_firmware.py`;
- `cmake/openocd.cmake`;
- `Makefile` и конфигурации `.vscode/`;
- `tools/monitor_stm32.py` и `tools/serial_read_test.py`;
- README, инструкция, CHANGELOG и этот отчёт.

## Изменения исходного кода из-за компилятора

Прикладной код перенесён из архива 2025 года. Потребовались только две совместимые правки:

1. В `Core/App/terminal_cmd_handlers.c` добавлен отсутствовавший include `spi_gpo.h`. ARMCC принимал вызов без видимого прототипа, а GCC 14 в режиме C99 останавливал сборку на неявном объявлении функции.
2. В `Core/BSP/l6474_drvr.c` удалена Keil-специфичная `#pragma O0`. Эквивалентная оптимизация `-O0` назначена этому файлу через CMake во всех конфигурациях.

Дополнительно линковщик использует `-fno-strict-aliasing`, как в переносе 2026 года, чтобы не менять поведение существующего кода с доступом к данным через указатели другого типа.

## Удалённые зависимости Keil

- `MDK-ARM/` и ARMASM startup;
- `ARMCC_Include/`;
- FreeRTOS-порт RVDS и остальные неиспользуемые порты;
- неиспользуемые startup для ARM/IAR;
- неиспользуемые файлы CMSIS-DSP, HAL и LL;
- старые Keil build-артефакты.

Удаление относится только к итоговой ветке. Исходные локальные архивы не изменялись.

## Проверка сборки

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

## Предупреждения GCC

Чистая Debug-сборка выдаёт 145 предупреждений: 65 `unused-parameter`, 64 `comment`, 6 `sign-compare`, 3 `implicit-fallthrough`, 2 `misleading-indentation`, 1 `sequence-point` и 4 linker-предупреждения о различии размера `wchar_t` между приложением и библиотеками GNU.

Предупреждения не скрывались через `-w` или массовые `-Wno-*`. `-fshort-wchar` сохранён для соответствия ABI исходного ARMCC-проекта; wide-character API в текущем приложении не используется.

`sequence-point` в `robot_behavior_routine.c` указывает на неопределённый порядок изменения `strategy`. Это существующая проблема логики 2025 года; перенос не выбирает новое поведение без аппаратной спецификации.

## Инструменты прошивки

`make flash` и `make flash-release` используют OpenOCD + ST-LINK, записывают `firmware.elf`, выполняют verify и software reset. При нескольких адаптерах используется `OPENOCD_ADAPTER_SERIAL`.

`make flash-remote` сохраняет рабочий процесс ветки 2026 года: Release HEX копируется на `REMOTE` (по умолчанию `reset10`) и там запускается `~/flash_stm32.sh`.

`make monitor` ничего не записывает в MCU. Он сначала сравнивает Flash с локальным `firmware.bin`, затем читает `SystemCoreClock`, состояние scheduler, число задач, tick и программную тень `SPI_GPO_REG`.

## Что ещё требуется проверить на стенде

1. Сделать backup Flash целевой платы перед первой записью.
2. Выполнить `make flash`, убедиться в `Verified OK` и корректном reset.
3. Запустить `make monitor`: ожидаются `SystemCoreClock = 168000000`, scheduler = 1, не менее 8 задач и tick около 1000/с.
4. Проверить UART1 921600 8N1 read-only командами из `tools/serial_read_test.py`.
5. С поднятыми колёсами и доступным аварийным отключением проверить GPIO, SPI GPO, PWM, энкодеры, Maxon, Dynamixel, BMS, шаговые приводы и WS2812B.
6. Сравнить направления, масштабы одометрии, команды захвата и аварийную остановку со старой Keil-прошивкой.
7. Отдельно квалифицировать Release после прохождения Debug.

Успешный `check` подтверждает структуру и непротиворечивость образа, но не доказывает корректность поведения робота на аппаратуре.
