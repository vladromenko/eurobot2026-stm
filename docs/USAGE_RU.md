# Прошивка Eurobot 2026: инструкция и история изменений

## Путь сборки

```text
cmake/sources.cmake + CMakeLists.txt
        ▼
CMake preset debug/release → Ninja → arm-none-eabi-gcc
        │ Cortex-M4F, fpv4-sp-d16, hard float
        ▼
GNU startup + GCC-порт FreeRTOS + STM32F407VG_FLASH.ld
        ▼
firmware.elf → firmware.bin / firmware.hex / firmware.map
        ▼
cmake/check_firmware.py — векторы, ISR, память, CPU-флаги, образы
        ▼
OpenOCD → ST-LINK → SWD → STM32F407VGTx
        ▼
tools/monitor_stm32.py — Flash, FreeRTOS, tick, DemoTask, PCA9685
        ▼
стендовые проверки робота
```

## Что изменено

### Переход с Keil на CMake/GCC

Изначально проект собирался ARMCC 5 в Keil MDK-ARM. Список исходников, defines, include-пути, CPU/ABI-флаги и раскладка памяти перенесены в CMake; ARMASM startup заменён GNU startup из CMSIS, RVDS-порт FreeRTOS — портом `GCC/ARM_CM4F`, добавлены linker script и `cmake/gcc_runtime.c` (`_init`/`_fini`). Файлы Keil и неиспользуемые части вендорских библиотек удалены; итоговый образ при этом побайтово не изменился.

Сохранены `cortex-m4`, Thumb, `fpv4-sp-d16`, hard float, short enums/wchar, unsigned char и `-O0` в Debug. Release использует `-Os`, кроме `l6474_drvr.c`, которому оставлен `-O0` (bit-bang тайминги).

### Запуск DemoTask

В `Core/App/main.c` включён `DemoTask.CreateTask(4)`. В `Core/OS_Tasks/demo_task.c` включён `SPI_GPO_DEMO`: выходы 1–4 включаются на 250 мс и выключаются на 250 мс. Период — около 500 мс.

### Исправление PCA9685/I2C2

В `Core/BSP/i2c2.c`:

- бесконечные циклы ожидания заменены ожиданиями с тайм-аутом 20 мс;
- обрабатываются NACK (`AF`), bus error, arbitration lost и overrun;
- при ошибке формируется STOP и восстанавливаются ACK/POS;
- исправлена последовательность чтения 1, 2 и нескольких байт;
- возвращаются диагностические коды вместо вечного зависания.

В `Core/BSP/board_hw_config.c` частота, передаваемая в `LL_I2C_ConfigSpeed`, исправлена с 84 МГц на 42 МГц. При APB1 divider 4 это реальная частота PCLK1; заданная скорость I2C остаётся 100 кГц.

В `Core/App/main.c` результат сохраняется в `PCA9685_InitStatus`. Ошибка PCA9685 не останавливает создание задач и запуск FreeRTOS:

| Статус | Значение |
|---|---|
| `0` | PCA9685 инициализирован |
| `-2` | тайм-аут I2C |
| `-3` | устройство не подтвердило адрес |
| `-4` | ошибка шины |
| `-5` | потеря арбитража |
| `-6` | переполнение |

Адрес `0x40`, PB10/PB11, 100 кГц и команды PCA9685 не менялись.

### Программирование платы

Вывод NRST к ST-LINK не подключён, поэтому reset выполняется программно (SYSRESETREQ через AIRCR). STM32CubeProgrammer на этой плате завершался `DEV_TARGET_RESET_ERR`; проверенная запись выполнялась `st-flash --hot-plug --reset`. OpenOCD по умолчанию тоже использует software reset (ST-LINK без `srst`); на самой плате прошивка через OpenOCD ещё не проверялась.

Перед первой записью был сохранён полный дамп Flash 1 МБ. Отдельного пользовательского bootloader нет: единственная таблица векторов начинается с `0x08000000`. Заводской ROM bootloader STM32 находится в system memory и записью не стирается.

## Ошибки исходной и первой собранной прошивки

| Ошибка | Проявление | Исправление |
|---|---|---|
| Проект зависел от ARMCC/Keil | CMake/GCC не могли воспроизвести сборку | Перенесены sources, defines, includes, CPU/ABI-флаги, startup, FreeRTOS port и linker layout |
| Не было GNU linker/runtime-конфигурации | Нельзя было корректно получить запускаемый ELF/BIN/HEX | Добавлены linker script, `_init`/`_fini`, objcopy и аудит секций/векторов |
| `DemoTask` был отключён | Тест выходов не запускался | Включено создание задачи и тест выходов 1–4 |
| Бесконечное ожидание I2C `ADDR` | CPU работал, но tick/tasks оставались нулевыми; FreeRTOS не запускался | Добавлены тайм-ауты, обработка NACK/ошибок и освобождение транзакции |
| В `LL_I2C_ConfigSpeed` передавалось 84 МГц вместо PCLK1=42 МГц | I2C работал примерно на 50 кГц вместо заданных 100 кГц | Передаётся правильная частота 42 МГц |
| Результат `PCA9685.Init()` игнорировался | Причину отказа нельзя было увидеть, а зависание происходило до задач | Результат хранится в `PCA9685_InitStatus`; запуск продолжается |
| CubeProgrammer требовал reset, которого не было на ST-LINK | `DEV_TARGET_RESET_ERR` до записи | Прошивка через OpenOCD с software reset |

## Сборка

```bash
cmake --preset debug
cmake --build --preset debug
cmake --build --preset debug --target check
```

Release — то же самое с `release`. Debug следует проверять первым. Успешный `check` подтверждает структуру образа, но не работу механизмов.

## Загрузка

```bash
cmake --build --preset debug --target flash
```

Цель пересобирает прошивку, записывает `firmware.elf` через OpenOCD, сверяет записанное и делает reset. Ожидаемые строки: `** Verified OK **` и `** Resetting Target **`. В VS Code то же делает кнопка Flash.

Перед записью другой платы сначала сохраните её Flash:

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "init; dump_image backup-flash.bin 0x08000000 0x100000; exit"
```

## Отладка

В VS Code: F5 → «Debug (OpenOCD)» (сборка, прошивка, остановка на `main`, список задач FreeRTOS) или «Attach (OpenOCD)» без перепрошивки. Нужно расширение Cortex-Debug и `arm-none-eabi-gdb` в `PATH`.

## Проверка после загрузки

```bash
python3 tools/monitor_stm32.py
```

Ожидается:

- Flash совпадает с локальным `firmware.bin`;
- `SystemCoreClock = 168000000`;
- `PCA9685_InitStatus = 0` при подключённом исправном модуле;
- `xSchedulerRunning = 1`;
- не менее 9 задач;
- tick растёт примерно на 1000 в секунду;
- наблюдаются состояния DemoTask `0x00000000` и `0x003c0000`.

Если `PCA9685_InitStatus = -3`, проверить общий GND, логическое питание VCC 3.3 В, PB10→SCL, PB11→SDA, подтяжки SDA/SCL и перемычки A0…A5. Питание сервоприводов `V+` не заменяет логическое VCC микросхемы.

UART1 можно проверить read-only запросами:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install pyserial
python3 tools/serial_read_test.py --list
python3 tools/serial_read_test.py --port /dev/cu.НУЖНЫЙ_ПОРТ --command speed
```

## Что ещё нужно проверить

Перед тестами движения зафиксировать робот, поднять колёса и обеспечить быстрое отключение силового питания.

1. Подтвердить `PCA9685_InitStatus = 0` и измерить SCL/SDA логическим анализатором: адрес `0x40`, частота около 100 кГц.
2. Осциллографом проверить реальные выходы SPI GPO 1–4: 250 мс ON, 250 мс OFF. SRAM-регистр является только программной тенью.
3. Проверить UART1 921600 8N1 и команды чтения скорости/координат/кнопок.
4. Проверить UART3 Dynamixel 1 Мбит/с, USART6 Maxon 2 Мбит/с, UART4 BMS и UART5 Serial.
5. По одному проверить концевики, энкодеры, PWM, шаговый привод, сервоприводы PCA9685, WS2812B, насосы и клапаны.
6. Проверить аварийную остановку и `StopAllActuators` до испытаний автономного поведения.
7. Измерить high-water mark стеков задач и MSP; проверить отсутствие HardFault, NaN и повреждения SRAM при длительной работе.
8. Выполнить длительный Debug-прогон, затем отдельно повторить весь стендовый тест Release-сборки.
9. Сравнить координаты, направления колёс и механику захватов с прежней Keil-прошивкой.

## Известные нерешённые проблемы

- В `robot_behavior_routine.c` выражение с `strategy++` изменяет и читает переменную без определённого порядка; компилятор предупреждает о неопределённом поведении.
- В `gripper_task.c` переменная `move_speed` может остаться неинициализированной для неожиданного значения направления.
- GCC собирает проект с `-fshort-wchar` (наследие Keil), но стандартные библиотеки GNU используют 32-битный `wchar_t`. В текущем коде wchar API не используется; это нужно помнить при будущих изменениях.
- Аппаратно подтверждён запуск Debug после исправления I2C. Полное поведение робота и Release ещё не квалифицированы.

Эти два предупреждения прикладного кода нужно исправить после согласования ожидаемого поведения, до допуска автономного движения.
