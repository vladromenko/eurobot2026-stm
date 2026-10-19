# Отчёт о переносе Eurobot 2026 на CMake/GCC

Дата проверки: 19 октября 2026 года, macOS arm64. Рабочий каталог: `/Users/vladromenko/Desktop/Eurobot/02_FIRMWARE_2026/PROJECT/eurobot2026-stm`.

## Итог и границы проверки

**Перенос системы сборки выполнен и проверен компиляцией. Аппаратная миграция не завершена: запуск на STM32, периферия и поведение робота не проверены.**

- Найден полный Git worktree `02_FIRMWARE_2026/PROJECT/eurobot2026-stm`; обзорный SOURCE и остальные копии не изменялись.
- Единственный target `Main_Board`, MCU `STM32F407VGTx`, ARMCC 5.06 update 7 build 960. Отдельного target, названного по синей плате, нет. Соответствие физической синей плате без маркировки и подключения установить нельзя.
- На этом Mac прошли чистая Debug-сборка (`clean` + сборка), Release и сборка прямыми командами в `build/`. Для всех трёх прошёл аудит.
- Созданы `firmware.elf`, `.hex`, `.bin`, `.map`. Flash/RAM не переполнены.
- GCC и Programmer найдены в уже установленной STM32CubeIDE. Установка новых инструментов, CubeMX и CubeCLT не потребовалась.
- Programmer 2.23.0 запущен, `-l stlink` вернул `No ST-Link detected!`. Target `probe` проверен до ошибки отсутствующего probe. Фактический Device ID/Flash size прочитать невозможно.
- `flash`, `backup`, `verify`, `reset` на MCU не запускались. Ни запись, ни erase, ни изменение RDP/Option Bytes не выполнялись.
- Все **2514 файлов**, существовавших до начала работы (включая Keil, старые AXF/MAP, vendor и ARMCC_Include), сохранены побайтово. Проверка по SHA256, а не по Git HEAD: в исходной извлечённой рабочей копии уже было 1900 tracked-изменений. Эти изменения не сбрасывались.

## Фактическая структура и инвентаризация

```text
eurobot2026-stm/
  ARMCC_Include/                сохранён, в GCC include не добавлен
  Core/
    App/                       main, команды, одометрия, SystemInit, IRQ
      Include/
    BSP/                       периферия и низкоуровневые функции платы
      Include/
    Lib/                       Maxon, Dynamixel, BMS, HL_RTU и прочее
      Include/
    OS_Tasks/                  задачи робота
      Include/
  Drivers/
    CMSIS/                     Core headers, DSP и Device/ST
    STM32F4xx_HAL_Driver/       в target используются LL .c
  Middlewares/FreeRTOS-Kernel/ существующее ядро и все альтернативные порты
  MDK-ARM/                     оригинальная конфигурация/старые результаты
  cmake/                       добавленные файлы переноса и аудита
  CMakeLists.txt
  CMakePresets.json
  README_CMAKE.md
  MIGRATION_REPORT.md
  build/                       новая прямая сборка + debug/ + release/
```

В исходном дереве найдены следующие количества файлов (с учётом stock-шаблонов .cmsis): `.c`: 659, `.h`: 617, `.ld`: 19, `.s`: 117, `.sct`: 40, `.uvoptx`: 1, `.uvprojx`: 1. Полный список — `cmake/migration/original-file-inventory.json`. Полный baseline SHA256 — `cmake/migration/original-sha256.json`. `.ioc` отсутствует; `.cpp` в target нет. Альтернативные startup/scatter в `.cmsis` относятся к другим устройствам и не выбирались.

## Параметры Keil → GCC/CMake

| Параметр | Keil | GCC/CMake | Источник |
|---|---|---|---|
| Target | Main_Board, один | firmware | `.uvprojx/Targets`, статический список |
| Device | STM32F407VGTx; DFP 2.17.1 | STM32F407xx | `.uvprojx/Device`, сохранённая команда в `build/Keil/main.o` |
| CPU | Cortex-M4.fp.sp, little-endian, Thumb-2 | `-mcpu=cortex-m4 -mthumb` | Cpu, RvdsVP=2, `.lnp`, ARM attributes main.o |
| Flash | 0x08000000, 0x100000 (1 МБ) | те же начало/ёмкость | `.sct`, memory в `.uvprojx`, Keil MAP |
| SRAM | 0x20000000, 0x20000 (128 КБ) | данные/heap до 0x2001FC00; верхние 0x400 — отдельный STACK | `.sct`, Keil MAP |
| CCM | 0x10000000, 0x10000 (64 КБ); RW_IRAM2 пуст | описана, остаётся пустой; ASSERT запрещает необработанную CCM-секцию | `.sct`, Keil MAP |
| FPU | FPv4-SP-D16 | `-mfpu=fpv4-sp-d16` | `.lnp --cpu`, ARM attributes, macros main.o |
| Float ABI | VFP register arguments | `-mfloat-abi=hard` | `readelf -A MDK-ARM/build/Keil/main.o`, Tag_ABI_VFP_args |
| enum / wchar | small enum; wchar=2 | `-fshort-enums -fshort-wchar` | ARM attributes main.o; EnumInt=0 |
| char | PlainCh=0 (unsigned char default ARMCC) | `-funsigned-char` | Cads/PlainCh; без `--signed_chars` в сохранённой команде |
| Выравнивание / упаковка | AAPCS, стек 8 байт; специальных pack в Core нет | обычный ARM EABI, без `-fpack-struct` | attributes, аудит Core |
| C | `--c99` | `-std=c99` | Cads/uC99 и строка команд main.o |
| Оптимизация | `-O0`, space, split_sections | Debug `-O0`, function/data-sections, GC; Release дополнительно `-Os` | Optim=1; `__OPTIMISE_LEVEL 0` и строка команд main.o |
| LTO | отключён | отключён | uLtcg=0 |
| Отладка | debug=1 | `-g3` Debug; `-g` Release | `.uvprojx` |
| Startup | ARMASM `MDK-ARM/startup_stm32f407xx.s` | существующий `Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s` | список Keil и инвентаризация |
| Stack | 0x2001FC00..0x2001FFFF, initial SP 0x20020000, 0x400 байт | те же адреса и размер | `.sct`, startup, MAP |
| Heap | startup объявляет 0x200, но Keil MAP удаляет HEAP как unused | зарезервированы 0x200 байт после bss; никакого FreeRTOS heap allocator | startup, MAP Removing HEAP, статическое выделение RTOS |
| Linker | ArmLink `--scatter STM32F407.sct`, MicroLIB, strict, MAP/xref/callgraph | GNU ld, новый проверенный `.ld`, `--gc-sections`, MAP, print-memory-usage | `.lnp` подтверждает scatter даже при неоднозначных LDads/useFile=0 |
| Boot offset | отсутствует в target и MAP | 0, векторы 0x08000000 | `.sct`, Keil MAP; реальная плата не проверена |
| Библиотеки | MicroLIB, compiler helpers; дополнительных binary libs в target нет | newlib-nano, libm, libgcc; nosys; без semihosting | `.uvprojx IncludeLibs` пуст, `.lnp`, GCC MAP |
| Debug/Flash | ST-LINKIII-KEIL_SWO.dll, SW-DP, STM32F4xx_1024.FLM | CubeProgrammer + ST-LINK SWD; manual targets | `.uvoptx`, FlashDriverDll |

Keil не переносит названия своих ключей в GCC буквально: `--cpu`, `--apcs`, MicroLIB, ARMASM и scatter имеют другой формат/библиотеки. `-fno-strict-aliasing` добавлен для реально найденных обращений к хранилищу указателей через uint32_t в Dynamixel/WS2812B; это сохраняет эти обращения при оптимизации GCC, без переписывания драйверов. `#pragma O0` в l6474 оставлена, а её требование выполнено настройкой конкретного файла.

В исходном AXF нет полезного итогового блока attributes; ABI определён по сохранённому **main.o**, а не угадан по имени MCU. Старый ARMCC помечает FP model как Finite/denormal Sign only, GCC — IEEE 754. Ключи fast-math не включались ради имитации: идентичность численных краевых случаев не установлена; одометрия/FPU требуют стендового сравнения.

### Defines

Из Cads: `USE_FULL_LL_DRIVER`, `ARM_MATH_CM4`, `__FPU_PRESENT=1U` — перенесены все. Сохранённая реальная команда ARMCC из main.o также содержит `STM32F407xx` и `_RTE_`, автоматически добавленные Keil; они явно перенесены. Дубли `_RTE_` устранены. `__MICROLIB` и `__UVISION_VERSION=536` описывают Keil runtime/IDE и намеренно не подделываются под GCC. `__CC_ARM`/`__ARMCC_VERSION` — встроенные макросы компилятора, их заменяет настоящая ветвь `__GNUC__`. ROBOT_BIG и ROBOT_GUIDO в данном target **не определены**, поэтому не добавлялись. Иные настройки других проектов Eurobot не смешивались.

### Include directories (с сохранением порядка)

1. `Drivers/STM32F4xx_HAL_Driver/Inc`
2. `Drivers/STM32F4xx_HAL_Driver/Inc/Legacy`
3. `Drivers/CMSIS/Device/ST/STM32F4xx/Include`
4. `Drivers/CMSIS/Include`
5. `Drivers/CMSIS/DSP/Include`
6. `Middlewares/FreeRTOS-Kernel`
7. `Middlewares/FreeRTOS-Kernel/include`
8. `Middlewares/FreeRTOS-Kernel/portable/GCC/ARM_CM4F` (в Keil: `Middlewares/FreeRTOS-Kernel/portable/RVDS/ARM_CM4F`)
9. `Core/App/Include`
10. `Core/BSP/Include`
11. `Core/Lib/Include`
12. `Core/OS_Tasks/Include`
13. `MDK-ARM/.cmsis/include`
14. `MDK-ARM/RTE/_Main_Board`

Дополнительные абсолютные include путей к установленным на Windows packs из сохранённой команды не нужны: соответствующие заголовки уже входят в локальные пути выше. `ARMCC_Include` не используется, чтобы GCC не подхватил стандартные заголовки от другой runtime.

### Assembler и linker

Aads: interw=1, thumb=0 (целевая архитектура всё равно Cortex-M4/Thumb), Ropi=0, Rwpi=0, SplitLS=0, SwStkChk=0, NoWarn=0, MiscControls/Define/IncludePath пусты. Необходимые архитектурные параметры задаются GNU assembler через gcc-driver. Готовые инструкции ARMASM в GAS не передаются.

LDads: umfTarg=0, useFile=0, noStLib=0, ScatterFile=STM32F407.sct, RepFail=1, Ropi/Rwpi=0; IncludeLibs/IncludeLibsPath/Misc/LinkerInputFile пусты. Сохранённый `.lnp` подтверждает `--cpu=Cortex-M4.fp.sp --library_type=microlib --strict --scatter STM32F407.sct`, summary sizes, MAP, symbols, xref, callgraph, unused/veneer info. GCC выдаёт MAP и memory sizes; Keil HTML callgraph и browse-файлы не создаёт.

## Совместимость с GCC

| Файл / компонент | Зависимость от Keil | Проблема для GCC | Решение |
|---|---|---|---|
| MDK-ARM/startup_stm32f407xx.s | AREA/DCD/EXPORT, `__main`, MicroLIB heap | ARMASM несовместим с GAS | Выбран уже имеющийся GNU startup; оригинал сохранён |
| portable/RVDS/ARM_CM4F/port.c и portmacro.h | ARMCC inline asm / intrinsics | Не компилируются как GNU C asm | Выбран existing portable/GCC/ARM_CM4F той же версии |
| CMSIS headers | Ветки `__CC_ARM`, ArmClang, `__GNUC__`, `__weak`, `__packed`, `__asm` | Эти конструкции зависят от выбранной ветви | GCC-ветвь cmsis_gcc.h уже есть; заголовки не менялись |
| CMSIS DSP arm_math.h | Compiler-specific SIMD/packed ветви | Нужна правильная GCC-ветвь и ARM_MATH_CM4 | Существующие GCC intrinsics и определения |
| Core/BSP/l6474_drvr.c:37 | `#pragma O0` | GCC игнорирует pragma | Пофайловый `-O0`; warning не скрыт |
| dynamixel_master.c:347, ws2812b.c:62 | Type punning хранилища указателей | GCC strict-alias optimization может нарушить обращения | `-fno-strict-aliasing`; исходники сохранены |
| GNU startup → __libc_init_array | Keil вместо этого вызывал `__main` | При -nostartfiles отсутствует `_init` | Новый независимый `cmake/gcc_runtime.c` с пустыми `_init/_fini` |
| ARMCC_Include | Стандартные заголовки Keil runtime | Несовместимая runtime GCC | Каталог сохранён и не включён в GCC search paths |
| wchar ABI | Keil wchar_t 2, GNU library wchar_t 4 | Linker выдаёт 22 предупреждения ABI | Локальный ABI сохранён; wide-char API в приложении нет; предупреждения оставлены открытыми |
| Core/App/robot_behavior_routine.c:115 | Ошибка исходной C-логики, не специфична Keil | GCC диагностирует assignment + postincrement в одном выражении | Не менялась логика; зарегистрирована отдельная нерешённая ошибка |
| Core/OS_Tasks/gripper_task.c:1036..1089 | Отсутствует значение move_speed при dir != 0/1 | Release диагностирует возможное неинициализированное значение | Не выбиралось новое поведение; отдельная нерешённая ошибка |

Core не содержит активных `__irq`, `__declspec`, собственных функций Keil runtime, Keil-only inline asm или нестандартных binary libs. Внешние C-библиотечные зависимости сведены к памяти/строкам и математике. Пользовательский код не содержит pragma pack/нестандартно packed структур. Протокольные значения остаются uint8/16/32 и float32, кодирование через существующие memcpy-функции. Нет оснований менять упаковку всего проекта. Побайтовая идентичность исполняемого кода не заявляется.

### FreeRTOS и прерывания

Фактическое ядро — **V10.4.3 LTS Patch 3** (`include/task.h`, port.c), хотя комментарий FreeRTOSConfig.h ещё упоминает 10.0.1. Ядро/конфигурация не обновлялись. GCC порт этой же версии уже находился в дереве. Используются семь C-файлов ядра из Keil и единственный GCC/ARM_CM4F/port.c. Heap_1/2/3/4/5 не добавлялись: configSUPPORT_STATIC_ALLOCATION=1, DYNAMIC=0. Пользовательские static TCB/stack буферы и idle memory hook сохранены.

Такт RTOS 1000 Гц, configCPU_CLOCK_HZ=SystemCoreClock, configMAX_PRIORITIES=7, min idle stack=16 слов. Приоритет ядра 0xF0, max syscall 0xB0 (4 NVIC priority bits). NVIC_SetPriorityGrouping(0) в main не менялся. Критические секции используют BASEPRI из существующего GCC порта с barrier-инструкциями. Порт сохраняет контекст FPU и включает lazy stacking. Выбраны hard-float и FPv4-SP-D16; SystemInit открывает CP10/CP11 при `__FPU_USED=1`.

SVC/PendSV/SysTick направлены на обработчики порта через исходные макросы FreeRTOSConfig.h. 14 собственных обработчиков из stm32f4xx_it.c + 3 RTOS обработчика проверены как strong, а не default aliases. Второй startup и второй RTOS port в сборку не попадают.

98 записей векторов совпадают по позиции и смыслу. В оригинальном ARMASM неиспользуемый IRQ48 назван `FMC_IRQHandler`, в существующем ST GNU шаблоне — `FSMC_IRQHandler`; оба ведут в default handler. Реализации FMC/FSMC в приложении нет. Аудит учитывает это единственное различие имён и не разрешит молча потерять добавленный strong FMC handler. Название initial SP `_estack` соответствует `__initial_sp=0x20020000`.

## Память и startup

GNU `.ld` составлен по действующему scatter и двум сохранённым Keil MAP. В обоих RW_IRAM2 пуст; данные помещены в SRAM, включая DMA buffers. Автоматического перелива данных в CCM нет: DMA не должен неожиданно получать адрес CCM. CCM описана, но ненулевая пользовательская CCM-секция вызывает ошибку линковки, пока не будет явно добавлена её инициализация. В текущих исходниках специальных секций/внешней RAM/bootloader offset нет.

- Векторы: `0x08000000`, 392 байта (98 записей), сохраняются KEEP, выравнивание 512.
- `.text`, rodata, unwind и init arrays — Flash; `.data` SRAM с load image во Flash.
- Startup использует согласованные `_sidata/_sdata/_edata/_sbss/_ebss`, все выровнены на 4 байта.
- `.bss`, `.heap`, `.stack` — NOLOAD; в HEX отсутствуют адреса SRAM.
- Отдельный STACK исключён из доступного для данных SRAM, что предотвращает переполнение в область 0x2001FC00..0x2001FFFF.
- GNU startup сначала копирует `.data`, затем обнуляет `.bss`, вызывает SystemInit (FPU/VTOR), __libc_init_array, main. Reset_Handler указывает на Thumb код; initial MSP=0x20020000. В main бесконечный цикл после scheduler; возврат main не ожидается.
- Статически проверен дизассемблированный Reset_Handler, но выполнение этого пути на MCU ещё не проверено.
- Heap reserve 512 байт — осознанное небольшое отличие от Keil live map, удалявшего unused HEAP. Оно сохраняет размер, объявленный startup; это не выделение FreeRTOS heap. `_sbrk`/malloc не требуются текущему приложению; nosys не даёт semihosting I/O.

## Конфигурация периферии (сохранена)

`.ioc` отсутствует. Фактические функции и настройки прочитаны из `Core/App/main.c`, `Core/BSP/board_hw_config.c` и GPIO-файлов, без генерации CubeMX:

- `SYSCFG_PWR_Init`, `SystemClockConfig`: HSE_VALUE=16 МГц, PLL M=8/N=168/P=2/Q=4, SYSCLK=168 МГц, AHB /1, APB1 /4, APB2 /2, Flash latency 5; LSI включается. Поле CLOCK(12000000) в Keil — metadata отладки, не действительная настройка PLL; 12 МГц не перенесены в runtime.
- `HW_ClockConfig`, `Board_GPIO_Config`, `Robot_GPIO_Config`: прежние GPIO и DMA clocks.
- `I2C2_Init`, PCA9685.Init(0x40,50,500,2500,180); исходное допущение частоты I2C2 не корректировалось. В коде есть неоднозначные комментарии о частоте APB1; это требует проверки формы SCL на плате.
- `SPI1_Board_HW_Init`, SPI_GPO_ResetOutputPin(SPI_GPO_ALL).
- USART1 terminal 921600, USART3 Dynamixel 1000000, UART4 RS485 2000000, UART5 Serial 115200, USART6 RS485 2000000; DMA/IRQ — прежние.
- TIM1 stepper PWM, TIM4/TIM5 PWM groups, TIM8 WS2812B, TIM14 USART timeout.
- IWDG_Init в main закомментирован исходно; не включался.

Не найденные `MX_*` функции не придумывались. CubeMX/CubeCLT не обязательны. HAL .c в Keil target нет: присутствующий STM32F4 HAL package 1.7.13 сохранён, компилируются только те 18 LL .c, которые указаны в target. CMSIS Core 5.1 из существующего первого include path и DSP остаются прежними.

## Что добавлено / изменено

**Изменённых старых файлов нет.** Новые файлы:

| Файл | Действие | Причина |
|---|---|---|
| `CMakeLists.txt` | Создан | Цель firmware, CPU/FPU/ABI, артефакты и опциональный check |
| `CMakePresets.json` | Создан | Проверенные Debug/Release presets |
| `cmake/arm-none-eabi.cmake` | Создан | GCC/G++, инструменты ARM и автоматический поиск нативного пакета |
| `cmake/keil_sources.cmake` | Создан | Явные 82 исходника и 14 include-путей из Keil с двумя заменами |
| `cmake/STM32F407VG_FLASH.ld` | Создан | Проверенные Flash/SRAM/CCM/stack и символы startup |
| `cmake/gcc_runtime.c` | Создан | Только `_init/_fini` для newlib; без изменения приложения |
| `cmake/stm32_flash.cmake` | Создан | Ручные targets probe/backup/flash/verify/reset |
| `cmake/programmer.cmake` | Создан | Реализация команд Programmer; обычная сборка не вызывает их |
| `cmake/check_firmware.py` | Создан | Аудит состава, ABI, векторов, памяти, HEX/BIN и original hashes |
| `cmake/migration/original-sha256.json` | Создан | SHA256 всех 2514 исходных файлов до переноса |
| `cmake/migration/original-file-inventory.json` | Создан | Полная инвентаризация заданных расширений |
| `README_CMAKE.md` | Создан | Русская инструкция сборки, прошивки и аппаратного тестирования |
| `MIGRATION_REPORT.md` | Создан | Конфигурация Keil, изменения, проверки и ограничения |

Сгенерированные build/debug, build/release, build/firmware.*, compile_commands.json и журналы не являются изменениями исходников. Keil файлы и все старые MDK-ARM/build артефакты сохранены.

## Реальные ошибки и исправления при переносе

1. Все 82 исходных единицы target сразу прошли компиляцию GCC. Ошибок из-за пропавших исходников, include или defines не было.
2. Линковка сначала остановилась на `undefined reference to _init` из __libc_init_array при `-nostartfiles`. Добавлен отдельный gcc_runtime.c с `_init/_fini`; исходное main не менялось.
3. Первое обнаружение Programmer через CMake не нашло пакет из-за проверки target-платформы APPLE при cross compile Generic. Исправлено на CMAKE_HOST_APPLE. После этого найден нативный CLI 2.23.0; `probe` дошёл до фактической ошибки отсутствующего ST-LINK.
4. При сравнении векторов обнаружено различие FMC/FSMC между двумя уже имеющимися ST шаблонами; подтверждены одинаковый слот и default handler, учтено в аудите без правки startup.
5. Release диагностика указала на pointer type punning в Dynamixel/WS2812B. Отключена strict alias optimization; это изменение параметров генерации кода, не подавление warning-флагом.
6. Для согласованной последующей readback-проверки objcopy заполняет промежутки Flash-образа 0xFF в HEX и BIN. Иначе HEX оставляет промежутки стёртыми, а обычный BIN содержит нули, давая ложное расхождение verify.
7. Небольшие ошибки первоначального вспомогательного аудита (разные имена FMC/FSMC и Thumb-bit на default alias) исправлены. Аудит теперь проверяет фактические адреса и роли символов, а не только одинаковое написание.

### Открытые предупреждения

Не применялись `-w`, `--no-wchar-size-warning`, массовые `-Wno-*` или исключение компонентов. Debug: **229 предупреждений**. Release: **230 предупреждений** после -fno-strict-aliasing. Категории Debug: Wcomment 70, Wunused-parameter 128, Wmisleading-indentation 3, Wunknown-pragmas 1, Wsign-compare 2, Wsequence-point 1, Wunused-variable 1, Wunused-function 1; ещё 22 linker wchar warnings. Release добавляет 1 Wmaybe-uninitialized.

- Критично для сопоставления поведения: `robot_behavior_routine.c:115` повторно изменяет strategy в одном выражении. Желаемый цикл стратегии должен определить владелец логики; перенос не выбирает его самостоятельно.
- `gripper_task.c:1036..1089`: move_speed не задаётся при dir != 0/1. Нужно определить обработку недопустимого dir; до этого не допускать такие команды, не объявлять движение аппаратно проверенным.
- `l6474_drvr.c` pragma warning остаётся видимым, но его смысл `-O0` выполнен в CMake для этого файла, включая Release.
- Wcomment вызваны исходными строками `// ... \\` в FreeRTOSConfig.h; при C99 перенос строки внутри // диагностируется GCC, активные определения ниже сохранены.
- Wmisleading-indentation — в VerifyPacket Dynamixel/HL_RTU и ожидании LSI; код не «переформатировался» вместе с логикой.
- 22 wchar warnings означают реальное различие ABI с библиотечными объектами GNU. В текущем приложении нет wchar/wide-string API; вызовы используемых math/memory/string функций не передают wchar. Перенос сохраняет ABI приложения и не скрывает предупреждение. При будущем wide-character API нужно привести библиотеку и приложение к одному ABI, а не подавлять warning.

## Проверенная сборка и размеры

| Сборка | text | data | bss (включая heap/stack reserves) | Flash image BIN | Результат |
|---|---:|---:|---:|---:|---|
| Debug -O0 | 65 604 | 2812 | 17 860 | 68 424 | чистая сборка + check PASS |
| Release -Os | 39 596 | 2764 | 17 852 | 42 368 | сборка + check PASS |
| Прямая CMake -B build, Debug | 65 604 | 2812 | 17 860 | 68 424 | сборка + check PASS |

Debug GNU memory usage: Flash 6.53%; RAM без отдельного STACK 19 648 / 130 048 байт; STACK 1024 / 1024; CCM 0. Отдельная `.bss` 16 324 байта. `size` считает NOLOAD heap и stack в bss, поэтому в таблице bss=17 860. Это резервы, не измерение реального расхода стека. RAM всего с резервами 20 672 / 131 072 байт. Разница Flash image и text+data связана с выравниванием секций.

Keil build/Keil MAP: load region 0x405CC, ER_IROM1 0x3FA38, RW_IRAM1 0x55B4, CCM=0, stack=0x400. В другом старом MDK-ARM/Main_Board.map размеры отличаются: это сохранённые результаты разных сборок, не повторно полученный baseline. GCC и Keil удаляют unused sections и размещают библиотеки по-разному; меньший GNU размер не доказывает потерю функциональности. Все источники target сверены независимо от размеров.

### Выполненные команды

```bash
cd '/Users/vladromenko/Desktop/Eurobot/02_FIRMWARE_2026/PROJECT/eurobot2026-stm'
cmake --preset debug
cmake --build --preset debug --target clean
cmake --build --preset debug
cmake --build --preset debug --target check
cmake --preset release
cmake --build --preset release
cmake --build --preset release --target check
cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
cmake --build build
cmake --build build --target check
python3 cmake/check_firmware.py --build build/debug --check-originals
```

Все эти команды завершились успешно. `--check-originals` предназначен для проверки самого переноса: после ваших дальнейших правок исходников его ожидаемое сообщение changed files не является ошибкой обычной сборки. Обычный `check` сравнивает состав сборки и образ, не запрещает редактирование кода.

Аудит проверяет: 82 source units из XML, ровно две замены по toolchain и один runtime; отсутствие duplicate compilation units; все include/defines/CPU/FPU/ABI flags; непустые четыре результата; ELF ARM32 little endian hard float; 98 векторов, Thumb Reset/MSP; 17 strong ISR; отсутствие пересечения данных/стека; начальные значения .data в BIN; контрольные суммы HEX, только адреса Flash, совпадение HEX/BIN. Отдельно просмотрены linker sections/segments и дизассемблированный Reset_Handler. Ни один из этих пунктов не является запуском на MCU.

Полные логи и результаты проверки находятся в `build/migration-logs/`; экспортированная копия отчёта сопровождается теми же логами. Путь к использованному GCC: `/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.macosaarch64_1.0.0.202602081740/tools/bin`. Programmer: `/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macosaarch64_1.0.100.202603051304/tools/bin/STM32_Programmer_CLI`.

## Сохранение функциональных компонентов

| Компонент | Keil | GCC/CMake | Статус |
|---|---|---|---|
| MCU settings | STM32F407VGTx | та же архитектура, память, определения | подтверждено конфигурацией/ELF |
| Startup | ARMASM | существующий ST GNU вариант | статический аудит PASS; запуск не проверен |
| Linker | STM32F407.sct | эквивалентные регионы в GNU ld, отдельный stack | секции/границы PASS, heap reserve описан выше |
| FreeRTOS | 10.4.3 LTS P3, RVDS CM4F | то же ядро/настройки, existing GCC CM4F | скомпилировано; scheduler на MCU не проверен |
| CMSIS | существующие Core/Device/DSP | те же заголовки, те же 11 DSP .c | скомпилировано |
| HAL | пакет в дереве; HAL .c не входят в target | сохранён, дополнительные HAL .c не добавлены | состав сохранён |
| LL | 18 .c из target | все те же 18 .c | скомпилировано |
| UART | board_hw_config/usartx | те же файлы/IRQ | скомпилировано; аппаратно нет |
| RS485 | UART4/USART6, прежние GPIO | те же драйверы и настройки | скомпилировано; аппаратно нет |
| DMA | исходные потоки/каналы/buffers | те же, buffers в main SRAM | статически сохранено; аппаратно нет |
| PWM | pwm/rc_servo/таймеры | те же файлы | скомпилировано; аппаратно нет |
| Dynamixel | master/ax/mx | те же файлы | скомпилировано; type aliasing защищено флагом; аппаратно нет |
| Колёсные приводы | maxon/kinematics/odometry | те же файлы | скомпилировано; движение не проверено |
| Насосы | spi_gpo + gripper_task | те же файлы/маски | скомпилировано; выходы не включались |
| Захваты | gripper/stepper/PCA9685 | те же файлы/настройки | скомпилировано; механика не проверена |
| Команды STM32 | terminal_cmd_handlers/table | исходники/ID/параметры не менялись | побайтово сохранены; live обмен не проверен |
| Протокол обмена | HL_RTU, Serial, byte2word | те же файлы и кодирование | побайтово сохранён; live обмен не проверен |

Команда PCA9685 0x1D определена в заголовке, но в исходном terminal_cmd_handlers[] не имеет назначенного обработчика; новая сборка сохраняет это состояние. Для проверки PCA9685 подходят активные 0x2B/0x2C.

Неактивные функции и закомментированные CreateTask/условные ROBOT_* не включались искусственно. Скомпилированы также debug_task/demo_task из исходного target, даже если их CreateTask закомментирован. Недостижимый код может удаляться linker, как и в Keil.

## Состав target: полный список из Keil

Файлов с IncludeInBuild=0 в единственном target не найдено; групповых/file overrides нет. Файлы, отсутствующие в XML (например Core/OS_Tasks/template_task.c, дополнительные HAL/DSP .c и альтернативные RTOS ports), не подключены. Список CMake явный, без GLOB исходников.

| Группа Keil | Исходный файл | Выбор GCC |
|---|---|---|
| Core/OS_Tasks | `Core/OS_Tasks/buttons_callback_task.c` | тот же |
| Core/OS_Tasks | `Core/OS_Tasks/buttons_poll_task.c` | тот же |
| Core/OS_Tasks | `Core/OS_Tasks/debug_task.c` | тот же |
| Core/OS_Tasks | `Core/OS_Tasks/demo_task.c` | тот же |
| Core/OS_Tasks | `Core/OS_Tasks/freertos_routine_plug.c` | тот же |
| Core/OS_Tasks | `Core/OS_Tasks/game_task.c` | тот же |
| Core/OS_Tasks | `Core/OS_Tasks/gripper_task.c` | тот же |
| Core/OS_Tasks | `Core/OS_Tasks/odometry_task.c` | тот же |
| Core/OS_Tasks | `Core/OS_Tasks/terminal_task.c` | тот же |
| Core/OS_Tasks | `Core/OS_Tasks/stepper_task.c` | тот же |
| Core/Lib | `Core/Lib/bms_board.c` | тот же |
| Core/Lib | `Core/Lib/button.c` | тот же |
| Core/Lib | `Core/Lib/dynamixel_ax.c` | тот же |
| Core/Lib | `Core/Lib/dynamixel_master.c` | тот же |
| Core/Lib | `Core/Lib/dynamixel_mx.c` | тот же |
| Core/Lib | `Core/Lib/fifo_routine.c` | тот же |
| Core/Lib | `Core/Lib/h_bridge.c` | тот же |
| Core/Lib | `Core/Lib/maxon.c` | тот же |
| Core/Lib | `Core/Lib/rc_servo.c` | тот же |
| Core/Lib | `Core/Lib/serial.c` | тот же |
| Core/Lib | `Core/Lib/hl_rtu.c` | тот же |
| Core/Lib | `Core/Lib/pca9685.c` | тот же |
| Core/BSP | `Core/BSP/board_gpio_config.c` | тот же |
| Core/BSP | `Core/BSP/board_hw_config.c` | тот же |
| Core/BSP | `Core/BSP/l6474_drvr.c` | тот же |
| Core/BSP | `Core/BSP/pwm.c` | тот же |
| Core/BSP | `Core/BSP/spi_gpo.c` | тот же |
| Core/BSP | `Core/BSP/usartx_communication.c` | тот же |
| Core/BSP | `Core/BSP/usartx_timeout.c` | тот же |
| Core/BSP | `Core/BSP/stepper.c` | тот же |
| Core/BSP | `Core/BSP/ws2812b.c` | тот же |
| Core/BSP | `Core/BSP/i2c2.c` | тот же |
| Core/App | `Core/App/bus_device.c` | тот же |
| Core/App | `Core/App/debug_terminal_cmd_handlers.c` | тот же |
| Core/App | `Core/App/debug_terminal.c` | тот же |
| Core/App | `Core/App/kinematics.c` | тот же |
| Core/App | `Core/App/main.c` | тот же |
| Core/App | `Core/App/odometry.c` | тот же |
| Core/App | `Core/App/robot_behavior_routine.c` | тот же |
| Core/App | `Core/App/robot_gpio_config.c` | тот же |
| Core/App | `Core/App/stm32f4xx_it.c` | тот же |
| Core/App | `Core/App/system_stm32f4xx.c` | тот же |
| Core/App | `Core/App/terminal.c` | тот же |
| Core/App | `Core/App/terminal_cmd_handlers.c` | тот же |
| Middlewares/FreeRTOS-Kernel | `Middlewares/FreeRTOS-Kernel/croutine.c` | тот же |
| Middlewares/FreeRTOS-Kernel | `Middlewares/FreeRTOS-Kernel/event_groups.c` | тот же |
| Middlewares/FreeRTOS-Kernel | `Middlewares/FreeRTOS-Kernel/list.c` | тот же |
| Middlewares/FreeRTOS-Kernel | `Middlewares/FreeRTOS-Kernel/queue.c` | тот же |
| Middlewares/FreeRTOS-Kernel | `Middlewares/FreeRTOS-Kernel/stream_buffer.c` | тот же |
| Middlewares/FreeRTOS-Kernel | `Middlewares/FreeRTOS-Kernel/tasks.c` | тот же |
| Middlewares/FreeRTOS-Kernel | `Middlewares/FreeRTOS-Kernel/timers.c` | тот же |
| Middlewares/FreeRTOS-Kernel | `Middlewares/FreeRTOS-Kernel/portable/RVDS/ARM_CM4F/port.c` | `Middlewares/FreeRTOS-Kernel/portable/GCC/ARM_CM4F/port.c` |
| Drivers/CMSIS/DSP | `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.c` | тот же |
| Drivers/CMSIS/DSP | `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.c` | тот же |
| Drivers/CMSIS/DSP | `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.c` | тот же |
| Drivers/CMSIS/DSP | `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c` | тот же |
| Drivers/CMSIS/DSP | `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c` | тот же |
| Drivers/CMSIS/DSP | `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f32.c` | тот же |
| Drivers/CMSIS/DSP | `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c` | тот же |
| Drivers/CMSIS/DSP | `Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c` | тот же |
| Drivers/CMSIS/DSP | `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_cos_f32.c` | тот же |
| Drivers/CMSIS/DSP | `Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c` | тот же |
| Drivers/CMSIS/DSP | `Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_crc.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dac.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dma.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dma2d.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_exti.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fmpi2c.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_gpio.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_i2c.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_lptim.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_pwr.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_rcc.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_rng.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_rtc.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_spi.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_tim.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usart.c` | тот же |
| Drivers/STM32F4xx_HAL_Driver*LL | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_utils.c` | тот же |
| MDK-ARM | `MDK-ARM/startup_stm32f407xx.s` | `Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s` |

## Что осталось перед первой загрузкой и аппаратным завершением

1. Подключить ST-LINK и питание, выполнить `probe`, подтвердить маркировку STM32F407VG, Device ID 0x413 и Flash 1024 КБ. При несоответствии не писать.
2. Выполнить `backup` и сохранить 1 МБ вне build. При RDP не снимать защиту и не стирать память.
3. Подтвердить отсутствие bootloader и нужных данных в секторах 0–4, включая хвост сектора 4 до 0x0801FFFF. Это нельзя установить по отсутствию offset в исходном проекте.
4. Получить явное разрешение владельца на запись/сброс, затем запустить `flash`; успешная запись должна сопровождаться verify и reset. Сейчас такого разрешения нет и эти действия не выполнялись.
5. Повторить `verify`, проверить tick/clock/UART/RTOS/FPU, измерить запас MSP и стеков задач. Точные команды и ожидаемые результаты приведены в README_CMAKE.md.
6. Разобрать исходные ошибки strategy/move_speed до допуска движения; согласовать ожидаемое поведение, не исправлять их догадкой в рамках смены сборщика.
7. На стенде по очереди проверить RS485/Maxon/BMS, Dynamixel, GPIO/PWM/DMA, PCA9685/I2C, шаговый механизм, насосы/клапаны и захваты по процедуре README. Сохранить результаты сравнения с Keil/рабочей платой. Release требует отдельного прогона после Debug.

## Первичные справочные источники

Конкретные значения проекта взяты из локальных `.uvprojx`, `.uvoptx`, `.sct`, `.o`, `.map` и исходников. Для толкования compiler/ABI и карты STM32 использованы:

- [GCC: ARM options — hard/softfp и выбор FPU](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html).
- [GCC: code generation — short enum/wchar ABI](https://gcc.gnu.org/onlinedocs/gcc/Code-Gen-Options.html).
- [Keil: ARM C/C++ compiler options](https://www.keil.com/support/man/docs/uv4/uv4_dg_adscc.asp).
- [ST RM0090: reference manual семейства STM32F407](https://www.st.com/resource/en/reference_manual/rm0090-stm32f4xx-reference-manual-stmicroelectronics.pdf).
- Синтаксис команд Programmer дополнительно сверялся с `STM32_Programmer_CLI --help` установленной версии 2.23.0; запись для проверки синтаксиса не выполнялась.


## Последующее исправление подключения программатора, 19 октября 2026

После ручного запуска владельцем flash подключение UR завершилось DEV_TARGET_RESET_ERR до программирования. Весь 1 МБ Flash повторно прочитан через HOTPLUG и совпадает с резервной копией (SHA256 ff5de93b152beaf3a8719db26fc893373702379cddbbe50feb01436de1af01d2). В cmake/stm32_flash.cmake добавлен STM32_FLASH_CONNECT_MODE, по умолчанию NORMAL; cmake/programmer.cmake использует NORMAL/reset=SWrst либо явно выбранный UR/reset=HWrst. Режим UR требует NRST. Автоматического повторения записи при ошибке нет; probe/backup/verify сохраняют HOTPLUG. Синтаксис/выбор режима проверен с имитацией CLI, check реального Debug ELF прошёл. Сам агент реальную запись, сброс или подключение NORMAL не запускал.

## Исправление запуска при ошибке PCA9685, 19 октября 2026

После успешной ручной записи через `st-flash` чтение состояния показало работающий CPU, отсутствие HardFault и зависание до запуска FreeRTOS в ожидании флага `I2C2 ADDR`. В регистрах были `SR1.AF=1`, `SR1.ADDR=0`, `SR2.MSL=1`, `SR2.BUSY=1`: устройство по адресу 0x40 не подтвердило адрес, а прежний код ожидал ACK без ограничения времени.

В `Core/BSP/i2c2.c` добавлены тайм-аут 20 мс, распознавание NACK/BERR/ARLO/OVR, STOP и восстановление ACK/POS после ошибки, а также корректная последовательность чтения 1, 2 и нескольких байт. В `Core/BSP/board_hw_config.c` входная частота I2C исправлена на реальную APB1=42 МГц; адрес 0x40, PB10/PB11 и скорость 100 кГц сохранены. `main` сохраняет результат в `PCA9685_InitStatus` и продолжает запуск задач при недоступном модуле. Значения: 0 — успех, -2 — тайм-аут, -3 — NACK, -4 — ошибка шины, -5 — потеря арбитража, -6 — переполнение.

Debug и Release собраны GCC 14.3.1 и прошли target `check`. Текущий Debug: 69 516 байт, reset `0x0800e155`, SHA256 BIN `360eadd1a9368601bc39f8fcbaf20d0532582d73822365f54fac48cfc0898d7c`. Release: 42 992 байта, reset `0x08007d21`. Аппаратный результат этого нового образа нужно подтвердить после ручной загрузки.
