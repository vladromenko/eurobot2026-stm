# Eurobot 2026: сборка на macOS без Keil

> Актуализация 19 октября 2026: плата STM32F407 обнаружена, резервная копия Flash сохранена, отдельный загрузчик в пользовательской Flash не найден, а Debug-образ был успешно записан владельцем через `st-flash`. После первой аппаратной проверки исправлен бесконечный цикл инициализации PCA9685: I2C2 теперь обрабатывает NACK/ошибки и ограничивает каждое ожидание 20 мс; частота APB1 исправлена с ошибочных 84 МГц на реальные 42 МГц для режима I2C 100 кГц. Текущий Debug BIN — 69 516 байт, SHA256 `360eadd1a9368601bc39f8fcbaf20d0532582d73822365f54fac48cfc0898d7c`. Следующие команды загрузки следует выполнять через `st-flash`, поскольку CubeProgrammer на этой плате возвращает `DEV_TARGET_RESET_ERR`.

Проверено 19 октября 2026 года на этом Mac (arm64): чистая Debug-сборка, Release-сборка и сборка прямыми командами CMake прошли. Получены ELF, HEX, BIN и MAP. **Запуск на STM32 и поведение робота ещё не проверены. ST-LINK не обнаружен. Запись Flash, reset, erase и изменение Option Bytes не выполнялись.**

Рабочий проект: `/Users/vladromenko/Desktop/Eurobot/02_FIRMWARE_2026/PROJECT/eurobot2026-stm`. Это полный проект из `02_FIRMWARE_2026/PROJECT`, а не обзорная копия SOURCE, прошивка 2025 или EUROBOT_NEXT. Все 2514 исходных файлов и прежних артефактов сохранены без изменения байтов. Используется единственный Keil target `Main_Board`: STM32F407VGTx. Цвет платы и маркировку реального MCU нужно подтвердить на оборудовании: отдельного target «синяя плата» в проекте нет.

## Как проект собирался раньше

µVision читал `MDK-ARM/Main_Board.uvprojx`: список файлов, include-пути, defines, Cortex-M4F, оптимизацию и выходные файлы. ARMCC 5.06 update 7 компилировал C99, armasm собирал `MDK-ARM/startup_stm32f407xx.s`. ArmLink объединял объектные файлы и MicroLIB по `MDK-ARM/STM32F407.sct`: размещал код во Flash, данные и стек — в SRAM. Начальные значения данных хранились во Flash. FromElf создавал HEX из AXF. По `.uvoptx` использовался ST-LINK и алгоритм STM32F4xx_1024.FLM. Сохранённые старые AXF/MAP не пересобирались и не заменялись.

## Как проект собирается теперь

CMake формирует правила, Ninja запускает компилятор только для изменившихся файлов. ARM GNU Toolchain собирает Cortex-M4F код и связывает его по `cmake/STM32F407VG_FLASH.ld`. Используется **уже находившийся в проекте** GNU startup: он копирует `.data` из Flash, обнуляет `.bss`, вызывает `SystemInit`, библиотечную инициализацию и `main`. Уже имеющийся GCC порт FreeRTOS заменяет RVDS порт при сборке. Ядро, задачи и настройки FreeRTOS не меняются.

STM32CubeProgrammer передаёт готовый образ через ST-LINK по SWD. CubeMX и CubeCLT для этой сборки не требуются. `.ioc` в исходном проекте не найден. Запускать графический CubeMX и генерировать код после изменения приложения не нужно.

```text
          Eurobot 2026 — существующие исходники
                         |
                         v
                  CMakeLists.txt
                         |
                         v
                        Ninja
                         |
                         v
                  arm-none-eabi-gcc
                         |
                         v
              компиляция + GNU linker
                         |
                         v
                   firmware.elf  + firmware.map
                         |
                  +------+------+
                  |             |
                  v             v
             firmware.bin   firmware.hex
                  |             |
                  +------+------+
                         |
                         v
                STM32CubeProgrammer
                         |
                         v
                       ST-LINK
                         |
                         v
                   STM32F407VGTx
```

## Инструменты на этом Mac

| Инструмент | Установленная и реально запущенная версия | Архитектура |
|---|---|---|
| CMake | 4.4.3, Homebrew | arm64 |
| Ninja | 1.13.2, Homebrew | arm64 |
| ARM GNU Toolchain | GCC 14.3.1, GNU Tools for STM32 14.3.rel1 | arm64 |
| STM32CubeProgrammer | 2.23.0 из STM32CubeIDE | universal, содержит arm64 |

Ничего дополнительно устанавливать на этом Mac не понадобилось. Полная среда STM32CubeIDE не запускается: используются её самостоятельные консольные инструменты. CMake ищет GCC в PATH и в установленных пакетах CubeIDE/CubeCLT. Programmer также находится автоматически. Отдельно установленный CubeProgrammer 2.22.0 оказался x86_64; выбран доступный нативный 2.23.0.

Проверить пути и версии вручную:

```bash
cmake --version
ninja --version
export ARM_TOOLCHAIN_BIN="/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.macosaarch64_1.0.0.202602081740/tools/bin"
export PATH="$ARM_TOOLCHAIN_BIN:$PATH"
arm-none-eabi-gcc --version
export STM32_PROGRAMMER_CLI="/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macosaarch64_1.0.100.202603051304/tools/bin/STM32_Programmer_CLI"
"$STM32_PROGRAMMER_CLI" --version
```

После обновления CubeIDE номера каталогов могут измениться. При необходимости явно задать новые пути:

```bash
cmake --preset debug -DARM_TOOLCHAIN_BIN="$ARM_TOOLCHAIN_BIN" \
  -DSTM32_PROGRAMMER_CLI="$STM32_PROGRAMMER_CLI"
```

В уже настроенном каталоге CMake сохраняет путь к компилятору. Для смены GCC используйте новый build-каталог, например `cmake -S . -B build/new-gcc -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake -DARM_TOOLCHAIN_BIN="$ARM_TOOLCHAIN_BIN"`.

## Сборка и проверка

```bash
cd '/Users/vladromenko/Desktop/Eurobot/02_FIRMWARE_2026/PROJECT/eurobot2026-stm'
cmake --preset debug
cmake --build --preset debug
cmake --build --preset debug --target check
```

Результаты: `build/debug/firmware.elf`, `firmware.bin`, `firmware.hex`, `firmware.map`. `check` доступен при найденном Python 3; на этом Mac он найден. Для обычной сборки Python не требуется.

Debug использует `-O0`, как Keil. Его следует первым проверять на плате. Размер Flash-образа — **68 424 байта**; `.data` 2812 байт; `.bss` 16 324 байта; дополнительно зарезервированы heap 512 байт и MSP stack 1024 байта. Всего SRAM с этими резервами — 20 672 байта из 131 072. Строка STACK 100% у linker означает полностью выделенный резерв 1 КБ, а не измеренную глубину стека. Запас по стеку нужно измерить на плате.

```bash
cmake --build --preset debug --target clean
cmake --build --preset debug

# Отдельная оптимизированная сборка, ещё без аппаратного подтверждения:
cmake --preset release
cmake --build --preset release
cmake --build --preset release --target check
```

Release: `-Os`, кроме `l6474_drvr.c`, которому оставлен `-O0` из его исходной pragma. В Release выявлены потенциальные отличия поведения: см. предупреждения в отчёте. Для существующих преобразований указателей отключена оптимизация на основе strict aliasing (`-fno-strict-aliasing`), сами драйверы сохранены.

Прямые команды тоже проверены:

```bash
cmake -S . -B build -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
cmake --build build
cmake --build build --target check
arm-none-eabi-size build/firmware.elf
```

При этом результаты лежат непосредственно в `build/`, а не `build/debug/`. В дальнейших командах используется preset debug. При прямой сборке замените `cmake --build --preset debug` на `cmake --build build`.

## Где изменять код

| Что | Существующий файл |
|---|---|
| Точка входа и создание задач | `Core/App/main.c`, `main()` |
| Реализация задач и их статическая память | `Core/OS_Tasks/*_task.c`, `freertos_routine_plug.c` |
| Конфигурация RTOS | `Middlewares/FreeRTOS-Kernel/include/FreeRTOSConfig.h` |
| Обработчики команд и их таблица | `Core/App/terminal_cmd_handlers.c`, `Core/App/Include/terminal_cmd_table.h` |
| Транспорт команд | `Core/App/terminal.c`, `Core/Lib/hl_rtu.c`, `Core/Lib/serial.c` |
| GPIO платы / робота | `Core/BSP/board_gpio_config.c`, `Core/App/robot_gpio_config.c`, соответствующие Include-файлы |
| Тактирование, UART, DMA, таймеры, I2C | `Core/BSP/board_hw_config.c` |
| UART IRQ и транспорт | `Core/App/stm32f4xx_it.c`, `Core/BSP/usartx_communication.c`, `usartx_timeout.c` |
| Насосы, клапаны, захваты | `Core/OS_Tasks/gripper_task.c`, `Core/BSP/spi_gpo.c`, `Core/BSP/Include/spi_gpo_defines.h` |
| PCA9685 | `Core/Lib/pca9685.c`, `Core/BSP/i2c2.c` |
| Колёса | `Core/Lib/maxon.c`, `Core/App/odometry.c`, `kinematics.c`, `Core/OS_Tasks/odometry_task.c` |
| Dynamixel | `Core/Lib/dynamixel_master.c`, `dynamixel_ax.c`, `dynamixel_mx.c` |
| Шаговые двигатели / PWM | `Core/BSP/stepper.c`, `l6474_drvr.c`, `pwm.c`, `Core/Lib/rc_servo.c`, `h_bridge.c` |

После редактирования существующего `.c/.h` достаточно `cmake --build --preset debug`. При добавлении нового `.c` явно внесите его в `cmake/keil_sources.cmake`, а для сохранения двух сборок — также в Keil. `check` специально сообщает о расхождении списков; не отключайте проверку ради пропавшего компонента. Включать все файлы автоматически нельзя: в дереве есть альтернативные порты и драйверы.

## Обнаружение платы и резервная копия — до любой записи

Ниже команды обнаружения и чтения. Не используйте `flash`, `reset` или команды записи, пока владелец не разрешил загрузку. HOTPLUG сохраняет работающую программу без команды сброса/останова.

```bash
cd '/Users/vladromenko/Desktop/Eurobot/02_FIRMWARE_2026/PROJECT/eurobot2026-stm'
export STM32_PROGRAMMER_CLI="/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macosaarch64_1.0.100.202603051304/tools/bin/STM32_Programmer_CLI"
"$STM32_PROGRAMMER_CLI" -l stlink
cmake --build --preset debug --target probe
```

`probe` перечисляет ST-LINK и читает `DBGMCU_IDCODE` по `0xE0042000` и Flash size по `0x1FFF7A22`. На этом Mac он корректно нашёл Programmer, но завершился ошибкой `No debug probe detected` из-за отсутствия ST-LINK.

Ожидается семейство Device ID **0x413** (младшие 12 бит IDCODE) и Flash size **0x0400**, т.е. 1024 КБ. Один ID 0x413 не отличает STM32F407 от всех родственных STM32F405/415/417: прочитайте маркировку корпуса, ожидается STM32F407VG. Суффикс Tx в Keil не доказывает температурный класс и точную маркировку реальной платы. При другом MCU/размере — остановиться и пересмотреть linker, не пробовать запись.

При нескольких ST-LINK выберите конкретный серийный номер из списка:

```bash
cmake --preset debug -DSTM32_STLINK_SERIAL=СЕРИЙНЫЙ_НОМЕР_ИЗ_СПИСКА
```

После подтверждения MCU можно прочитать всю пользовательскую Flash:

```bash
cmake --build --preset debug --target backup
```

Результат — `build/debug/backup-flash-<UTC-дата-время>.bin`, **1 048 576 байт**. Скопируйте резервную копию за пределы build до очистки каталога. Target не пишет память MCU. При RDP/ошибке чтения остановиться: не выполнять `-rdu`, не менять RDP/Option Bytes и не стирать плату.

Эквивалентные прямые команды:

```bash
"$STM32_PROGRAMMER_CLI" -c port=SWD mode=HOTPLUG \
  -r32 0xE0042000 4 -r16 0x1FFF7A22 2
backup_file="flash-before-migration-$(date +%Y%m%d-%H%M%S).bin"
"$STM32_PROGRAMMER_CLI" -c port=SWD mode=HOTPLUG \
  -u 0x08000000 0x100000 "$backup_file"
wc -c "$backup_file"
shasum -a 256 "$backup_file"
```

Векторы и код текущей Flash могут принадлежать 2025 году. Прочитайте начало Flash (`-r32 0x08000000 8`) и сопоставьте дамп с известным образом/проектом. В новом проекте offset отсутствует, но это **не доказывает отсутствие загрузчика на реальной плате**. Если владелец или содержимое дампа указывает на bootloader, сначала выясните его границы и адрес приложения; текущий образ с `0x08000000` записывать нельзя.

Образ Debug занимает `0x08000000..0x08010B47`. Запись затрагивает Flash-секторы 0–4; их суммарная область `0x08000000..0x0801FFFF`. Сектор 4 продолжается за концом образа: подтвердите отсутствие нужной конфигурации/данных во всём этом диапазоне. Процедура не обещает сохранения неизвестных данных внутри стираемых секторов. Mass erase не используется.

## Загрузка — только после явного разрешения владельца

Для подключённой платы с ST-LINK `53004700090000393333574E` проверенная владельцем команда:

```bash
st-flash --serial 53004700090000393333574E \
  --hot-plug --reset \
  write "$PWD/build/debug/firmware.bin" 0x08000000
```

`st-flash` стирает только сектора, занятые образом, записывает BIN, сравнивает записанные данные и выполняет программный reset, когда NRST не подключён. Сообщение успеха: `Flash written and verified!`.

После маркировки MCU, проверки памяти/загрузчика, резервной копии и разрешения:

```bash
cmake --build --preset debug --target flash
```

Target сначала собирает актуальный HEX, затем выполняет запись через SWD с программным сбросом (`mode=NORMAL reset=SWrst`), проверку и reset. Режим NORMAL не требует отдельного провода NRST. Для явного выбора аппаратного under-reset используйте `cmake --preset debug -DSTM32_FLASH_CONNECT_MODE=UR`; ему нужен подключённый NRST. Он не вызывается обычной сборкой. Option Bytes и RDP не изменяются. В HEX присутствуют только адреса пользовательской Flash, SRAM/stack в него не попадают. Промежутки образа заполнены 0xFF, поэтому BIN и HEX согласованы для чтения назад.

Эквивалентная команда записи, **не выполнявшаяся при переносе**:

```bash
"$STM32_PROGRAMMER_CLI" -c port=SWD mode=NORMAL reset=SWrst \
  -w build/debug/firmware.hex -v -rst
```

При выбранном probe добавьте `sn=...` к `-c`. Дополнительные явные targets:

```bash
# Только чтение Flash текущего образа и сравнение с BIN:
cmake --build --preset debug --target verify
# Сброс — выполняйте лишь при отдельном разрешении на запуск платы:
cmake --build --preset debug --target reset
```

`verify` читает диапазон BIN в `verify-readback.bin` и сравнивает побайтово. Успех — exit code 0; расхождение — ошибка CMake. Этот target не подтверждает работу периферии.

## Конкретная процедура аппаратного тестирования

Все следующие проверки остаются невыполненными. До запуска робота зафиксируйте механику и используйте штатное отключение силовых приводов. Это требуется из-за реального управления двигателями и захватами в исходной прошивке.

| Проверка и зачем | Команда / действие | Ожидаемый результат | Если отличается |
|---|---|---|---|
| Наличие probe | `"$STM32_PROGRAMMER_CLI" -l stlink` | Серийный номер ST-LINK | Проверить USB data-кабель/USB-порт; повторить перечисление |
| MCU, питание, объём Flash | `cmake --build --preset debug --target probe`; прочитать маркировку | 0x413, 1024 КБ, STM32F407VG, напряжение цели около 3.3 В | Не записывать; проверить SWDIO/SWCLK/GND/VTref и схему платы |
| Сохранение старой прошивки | `cmake --build --preset debug --target backup`; `wc -c` и `shasum -a 256` для результата | 1 048 576 байт, сохранён SHA256, копия вне build | При RDP/ошибке остановиться без снятия защиты |
| Bootloader и соседние данные | Чтение `-r32 0x08000000 8`; анализ полного backup с владельцем/известной прошивкой | Подтверждены standalone-приложение с 0x08000000 и допустимость перезаписи секторов 0–4 | Сначала восстановить карту загрузчика/данных, не прошивать текущий HEX |
| Совпадение записанного образа после разрешённой загрузки | `cmake --build --preset debug --target verify` | Exit code 0 | Сохранить лог; повторно проверить serial probe, адрес, питание и образ; без mass erase |
| Старт FreeRTOS и тактирование после reset | Команды чтения ниже | Tick растёт примерно на 1000/с, SYSCLK от PLL, HSE=16 МГц → SYSCLK=168 МГц | Проверить кварц, RCC, остановку в I2C init; не менять PLL наугад |
| UART1 + DMA + терминальная задача | Скрипт GetSpeed ниже, 921600 8N1 | 17 байт, команда 0x05, 12 байт float, верная сумма | Проверить PA9/PA10, общий GND, TTL 3.3 В, частоту HSE и DMA USART1 |
| UART3/Dynamixel и USART6/Maxon | Существующий клиент: чтение `BOARD_GET_AX_ANGLE` 0x0A с `data=[ID]`; `BOARD_GET_WHEEL_TICKS` 0x0E без данных | AX: 3 байта данных ответа (статус + uint16); колёса: 13 байт, статус 1 и три int32; ID из `bus_device.c` | Сверить ID, питание, USART3 1 Мбит/с / USART6 2 Мбит/с и направления RS485 |
| Насосы и клапаны | Через штатный клиент `BOARD_TOGGLE_PUMPS` 0x21 с `[gripper_id,mask]`; выключение mask=0; аналогично 0x22 клапаны | Ответ `[01]`, работают выбранные каналы; после mask=0 выключены | Остановить силовую часть; сверить `grippers[]` и SPI_POWER_OUTPUT в исходниках |
| Захваты / PWM / PCA9685 / шаговый привод | Через штатный клиент 0x1F `[gripper_id,grips_cmd]`, 0x20 `[gripper_id,lift_cmd]`, 0x2B `[gripper_id,spread]` и 0x2C `[gripper_id,mask]` для PCA9685; параметры в исходниках и по допустимому механическому диапазону | Заданный захват доходит до положения, ответ `[01]`; PCA9685 50 Гц; шаги и концевики соответствуют старой прошивке | Остановить тест; проверить питание, механические границы, I2C2 и полярность GPIO; не менять распиновку |
| Одометрия/колёса | Штатный клиент 0x0E/0x07; разрешённый стендовый тест 0x28 с `[wheel_num]` для 1..3 | Счётчики/координаты меняются согласованно, одно колесо делает один оборот | Остановить движение; сверить ID Maxon, ticks/revolution и знак осей с Keil |
| UART4/BMS и UART5/Serial | Штатный запрос 0x08 BatteryState; штатный обмен Serial по UART5 | Ответ BMS 18 байт; прежний обмен Serial 115200 | Проверить BMS ID 0x11 и UART4 2 Мбит/с; UART5 и протокол `serial.c` |
| Запас стека, FPU при переключении задач | После разрешённой загрузки подключить отладчик: наблюдать SP/границы стеков и работающую одометрию/переключения задач | MSP не ниже 0x2001FC00; стеки задач не пересекают буферы; нет HardFault/NaN | Сохранить PC/LR/CFSR и дамп; не считать перенос аппаратно завершённым |

Команды чтения тактирования и tick (адрес tick определять **из текущего** ELF):

```bash
arm-none-eabi-nm -n build/debug/firmware.elf | rg ' xTickCount$| uxCurrentNumberOfTasks$'
"$STM32_PROGRAMMER_CLI" -c port=SWD mode=HOTPLUG -r32 0x40023800 12
# Для полученного Debug ELF xTickCount=0x20004A90; если ELF менялся, взять новый адрес:
"$STM32_PROGRAMMER_CLI" -c port=SWD mode=HOTPLUG -r32 0x20004A90 4
# Повторить спустя примерно секунду; tick должен увеличиться.
```

Если `main()` не дошёл до `vTaskStartScheduler`, tick не будет расти. В исходном коде есть ожидания готовности HSE/LSI и вызовы PCA9685 до запуска задач. Нельзя выводить работоспособность планировщика только из успешного линкования.

Проверка терминального обмена после разрешённой загрузки требует USB-UART TTL 3.3 В (PA9 TX, PA10 RX; соединить TX↔RX и GND; не подключать RS232 напрямую). Выберите фактический `/dev/cu.*`. PySerial нужен только для этого примера, а не для сборки:

```bash
python3 -m venv build/serial-test-env
build/serial-test-env/bin/python -m pip install pyserial
ls /dev/cu.*
# Подставьте порт своего USB-UART вместо /dev/cu.YOUR_ADAPTER:
build/serial-test-env/bin/python - /dev/cu.YOUR_ADAPTER <<'PYCODE'
import serial, struct, sys
with serial.Serial(sys.argv[1], 921600, timeout=2) as port:
    port.reset_input_buffer()
    port.write(bytes.fromhex('FF FF 02 05 F8'))  # BOARD_GetSpeed, без изменения выходов
    response = port.read(17)
    print(response.hex(' '))
    assert len(response)==17 and response[:4]==bytes.fromhex('FF FF 0E 05'), 'Неверный заголовок/длина'
    assert (sum(response[2:]) & 0xff)==0xff, 'Неверная контрольная сумма'
    print('dx, dy, dphi:', struct.unpack('<fff',response[4:16]))
PYCODE
```

Не используйте Ping как безусловно безвредный тест: существующий обработчик 0x00 переключает PD14. Показанный GetSpeed только запрашивает состояние. Команды с движением и насосами из таблицы выполняются отдельно через существующий клиент после допуска стенда.

## Предупреждения, которые нельзя принять за аппаратную проверку

Debug проходит с 229 предупреждениями, большинство повторяются из заголовков. В исходнике `robot_behavior_routine.c:115` есть неопределённое выражение `strategy = ... strategy++`; в Release также обнаружен потенциально неинициализированный `move_speed` в `gripper_task.c:1036..1089` при `dir`, отличном от 0/1. Эти существующие ошибки не исправлялись: выбор поведения изменил бы логику приложения, что запрещено заданием. До допуска движения следует отдельно определить правильное поведение и исправить их, затем повторить сравнение/стендовые проверки. Повторная сборка сама по себе их не устраняет.

22 предупреждения linker относятся к `wchar_t`: для сохранения локального ABI ARMCC оставлен 16-битный wchar, библиотека GNU собрана с 32-битным. В приложении нет wide-character строк/операций, в используемых внешних интерфейсах нет wchar. Предупреждения не скрыты. Добавление wide-character функций потребует единого ABI с библиотекой. Полный анализ, список файлов и логи — в `MIGRATION_REPORT.md`.

Важная особенность исходной таблицы: `BOARD_PCA9685_SET_SERVO_ANGLE` (0x1D) объявлена в заголовке, но обработчик в `terminal_cmd_handlers[]` не назначен. При тестировании PCA9685 используйте существующие 0x2B/0x2C, а не неработающую 0x1D. Это ограничение существовало до переноса. Функции Gripper_SetPumps/Valves сначала устанавливают всю маску выходов, затем последовательно снимают невыбранные биты; осциллографом учитывайте эти короткие промежуточные состояния, включая выключение mask=0.


Обновление 19 октября 2026: после DEV_TARGET_RESET_ERR в UR полный Flash readback подтвердил совпадение с резервной копией. Target flash теперь по умолчанию использует NORMAL/SWrst; автоматических попыток записи с разными режимами нет. Фактическую повторную запись выполняет владелец.
