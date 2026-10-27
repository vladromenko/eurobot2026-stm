# Changelog

## 2026-10-27 — перенос прошивки Eurobot 2025 с Keil на GCC/CMake

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
