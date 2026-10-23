# Changelog

## 2026-10-24

### Переход на C++: драйверы BSP и Lib переведены на классы

- Каждый драйвер теперь класс, а не структура с указателями на функции. Глобальный объект называется как раньше (`Stepper`, `PWM`, `L6474H`, `WS2812B`, `USARTx`, `Dynamixel_AX`, `Maxon`, `BMS`, `Button`, `PCA9685`, `Serial`, `HL_RTU` и др.), поэтому вызовы вида `Stepper.StepToPosisition(...)` не изменились.
- Состояние, которое лежало в глобальных переменных файлов, перенесено в поля классов: позиции шаговых моторов, регистр SPI_GPO, таймауты USART, кнопки, FIFO, настройки PCA9685.
- Без динамической памяти и рантайма C++: все объекты объявлены `constinit`, кода инициализации при старте нет; `malloc`, `new`, исключения и RTTI не используются. Для виртуальных функций в [cmake/gcc_runtime.c](cmake/gcc_runtime.c) добавлена заглушка `__cxa_pure_virtual`.
- Из заголовков BSP и Lib убраны `extern "C"`: ни один C-файл их не подключает.
- FLASH (Debug): 70108 → 62728 Б. RAM: 19656 → 19384 Б.

### USART: новый интерфейс протоколов

- Таблицы `usartx_protocol_s`, `usartx_control_s` и `usartx_line_config_s` заменены классами в [usartx_communication.hpp](Core/BSP/Include/usartx_communication.hpp):
  - `USARTx_Line` — линия с буферами, FIFO и методами `StartTx`, `StopTx`, `StartRx`, `StopRx`, `SetRxTimeout`, `ResetRxTimeout`;
  - `USARTx_Protocol` — базовый класс протокола с виртуальными `InitLine`, `TxCompleteCallback`, `RxCompleteCallback`, `RxTimeoutCallback`.
- `DynamixelMaster`, `Serial` и `HL_RTU` наследуются от `USARTx_Protocol`. Привязка в [main.cpp](Core/App/main.cpp): `USARTx.BindProtocol(USART3, DynamixelMaster)`, без `.GetProtocol()`.
- FIFO (`fifo_routine`) стал классом `FIFO` с методами `Init`, `Add`, `Remove`, `IsFull` и т.д.
- Обработчики прерываний вызывают методы: `Stepper.TIM_IRQ_Process()`, `USARTx.IRQ_Process(...)`, `USARTx_Timeout.TIM_IRQ_Process()`, `WS2812B.DMA_IRQ_Process()`.

### Другие изменения API

- `SPI_GPO_SetOutputPin(x)` / `SPI_GPO_ResetOutputPin(x)` → `SPI_GPO.SetOutputPin(x)` / `SPI_GPO.ResetOutputPin(x)`.
- `PCA9685_I2C_Write/Read` → `I2C2_Bus.Write/Read`, новый заголовок [i2c2.hpp](Core/BSP/Include/i2c2.hpp). Имя `I2C2` занято макросом CMSIS.
- Удалены пустые заглушки `DynamixelMaster.RegWrite`, `Action`, `FactoryReset`, `Reboot`, `BulkRead`: они ничего не делали и нигде не вызывались.
- Четыре функции `StepperMotor_N_Init` объединены в `Stepper.Init(motor)` с таблицей настроек; значения не изменились.

### Исправлено

- **`DynamixelMaster.SyncWrite` портил стек.** Цикл `do {…} while (boards_number--)` делал лишний проход с индексом 255. `memcpy` писал примерно на 768 байт за пределы локальной переменной, а при `response_array == NULL` запись шла ещё и по адресу `NULL + 9180`. Вызывается из `gripper_task`.
- **`DynamixelMaster.Write` с `response = NULL` писал по адресу 0.** Так вызываются почти все команды AX (`SetAngle`, `TorqueEnable`, …).
- **`BMS.GetPowerConsumption` отсутствовал в таблице функций**: поле было `NULL`, вызов уронил бы прошивку.
- **`Dynamixel_MX.GetAngle`** читал 2 байта в неинициализированную 4-байтную переменную, поэтому угол мог получаться неверным.
- `PCA9685.SetServoAngleWithAddr` больше не подменяет на время вызова адрес платы, сохранённый в `Init`.

### Не проверено на железе

Сборка Debug и Release и `make check` проходят. Запуск на плате не проводился: шаговые моторы, Dynamixel, BMS, Maxon, терминал (UART1), кнопки и светодиодная лента требуют проверки.

## 2026-10-23

### Сборка: только CMake, Keil удалён

- Проект собирается только через CMake + Ninja + `arm-none-eabi-gcc`. Keil MDK-ARM больше не нужен.
- `cmake/keil_sources.cmake` переименован в [cmake/sources.cmake](cmake/sources.cmake). Новый `.c` файл добавляется в `EUROBOT_SOURCES`.
- Убраны include-пути `MDK-ARM/.cmsis/include` и `MDK-ARM/RTE/_Main_Board`, define `_RTE_`, Keil-директивы `#pragma O0` в `l6474_drvr.c` и `spi_gpo.c`. Для `l6474_drvr.c` `-O0` по-прежнему задаётся в CMake.
- [cmake/check_firmware.py](cmake/check_firmware.py) больше не сверяется с `Main_Board.uvprojx`. Вектора прерываний проверяются по GNU-стартапу. Опция `--check-originals` удалена.
- Прошивка через STM32CubeProgrammer (`stm32_flash.cmake`, `programmer.cmake`) заменена на OpenOCD: [cmake/openocd.cmake](cmake/openocd.cmake), цели `flash` и `reset`. ST-LINK выбирается через `-DOPENOCD_ADAPTER_SERIAL=<serial>`.
- [tools/monitor_stm32.py](tools/monitor_stm32.py) читает память через OpenOCD, а не через CubeProgrammer.
- Итоговые `firmware.bin` в Debug и Release побайтово совпадают со сборкой до чистки.

### Удалено

- `MDK-ARM/`, `ARMCC_Include/`, `cmake/migration/`, `README_CMAKE.md`. Всё это остаётся в истории git.
- FreeRTOS: все порты, кроме `portable/GCC/ARM_CM4F`; `MemMang` не нужен, так как выделение памяти только статическое.
- CMSIS: `Core_A`, дубликат `Core`, шаблоны стартапа для ARM/IAR, неиспользуемые исходники DSP.
- STM32F4 HAL/LL: все `.c` и `.h`, которые не участвуют в сборке. Если понадобится новая периферия, файлы берутся из пакета STM32CubeF4.

### Инструменты

- [Makefile](Makefile) поверх CMake-пресетов: `make`, `make release`, `make check`, `make check-release`, `make flash`, `make flash-release`, `make flash-remote`, `make reset`, `make monitor`, `make clean`, `make help`.
- `make flash-remote` собирает Release, копирует `firmware.hex` на `reset10` в `~/firmware.hex` и запускает там `~/flash_stm32.sh` (`st-flash --reset --format ihex write firmware.hex`). Другой хост: `REMOTE=user@host`.
- VS Code:
  - задачи Build Debug/Release, Flash Debug/Release, Clean;
  - кнопки в статус-баре через расширение Task Buttons;
  - отладка через Cortex-Debug и OpenOCD: «Debug (OpenOCD)» с остановкой на `main` и списком задач FreeRTOS, «Attach (OpenOCD)»;
  - IntelliSense берёт флаги из `build/debug/compile_commands.json`.

### Исправлено: STM32 не отвечал по UART1

**Симптом.** Пульт `~/test_low/start.sh` и `check_uart.py` на `reset10` не получали ни одного байта, даже ответа на PING.

**Причина.** В `demo_task.c` был закомментирован `#define SPI_GPO_DEMO`. После этого цикл `DemoTask` стал пустым: без `vTaskDelay`, с приоритетом 4. Задача забирала весь процессор, и задачи с приоритетом ниже 4, включая `TerminalTask` (приоритет 1), не получали ни одного кванта.

Что показало чтение платы по SWD:

- FreeRTOS работал: тик рос, 9 задач, `PCA9685_InitStatus = 0`;
- `pxCurrentTCB` всё время указывал на `DemoTask`;
- у `TerminalTask` сохранённый PC стоял на начале функции, LR — на `prvTaskExitError`, то есть задача ни разу не запускалась;
- запросы доходили до платы через DMA и копились в очереди, но DMA передачи ни разу не включался.

**Исправление.** `DemoTask.CreateTask(4)` в [main.c](Core/App/main.c) закомментирован, как было в исходной Keil-прошивке. В `monitor_stm32.py` убрана проверка мигания выходов, ожидаемое число задач снижено до 8.

**Правило на будущее.** `DemoTask` включать только вместе с каким-нибудь демо-режимом в `demo_task.c`. Любая задача с пустым циклом и высоким приоритетом блокирует терминал.

### I2C2: частота шины

Ненадолго пробовали вернуть 84 МГц в `LL_I2C_ConfigSpeed` для нового робота, но затем откатили. В коде осталось 42 МГц, что соответствует реальной частоте APB1 (168 МГц / 4), шина работает на 100 кГц. Значение 84 МГц из исходной прошивки давало около 50 кГц.

### Расследование: зависание при отключении разъёмов (не закрыто)

**Симптом.** Отключение под питанием разъёма I2C2 (используются GND, SCL, SDA и +5V; пины 4–7 ни к чему не подключены), пина 26 разъёма 28 GPIO, +5V и +12V разъёмов с сервами и другой нагрузкой: контроллер перестаёт отвечать по UART.

**Программную причину не нашли.**

- I2C2/PCA9685: каждое ожидание в [i2c2.c](Core/BSP/i2c2.c) ограничено 20 мс. Вызовы из `gripper_task` перемежаются `vTaskDelay`. Без PCA9685 команды серво просто возвращают ошибку.
- Кнопки и концевики: входы подтянуты к земле. `ButtonsCallback` только выставляет флаги в `robot_ctrl`.
- Ожидание захвата в терминале (`WaitForGripperCompletion`) ограничено 5 секундами.
- Все команды терминала отвечают всегда. `COMM_ENABLE = 0` блокирует только действия.

**Что указывает на железо.** После отключения ST-LINK не видел микроконтроллер: `chipid 0x0000`, `Can not connect to target`. При программном зависании SWD продолжает работать. Он пропадает, когда МК без питания, в просадке или в защёлкивании (latch-up).

Рабочая гипотеза — питание и земля при горячем отключении:

- если GND рвётся раньше сигнала и +5V, обратный ток нагрузки идёт через сигнальную линию в ножку STM32 (по схеме GPIO там только 1 кОм и диод на +5V);
- разрыв цепи с током мотора даёт индуктивный выброс и просадку +5V.

**Что сделать дальше.**

1. Передёрнуть питание.
2. Под питанием отключить только сигнальный пин, без +5V и GND.
3. При повторе сразу выполнить `ssh reset10 st-info --probe` и измерить 3.3V на плате.
4. До выяснения подключать и отключать разъёмы с питанием нагрузки только при выключенном питании.

### Стартовый шнур (PC15)

Вход стартового шнура — I2C2, пин 5 (PC15). Он же пин 27 разъёма 28 GPIO. Кнопка активна по высокому уровню (`BUTTON_POLARITY_HI`), вход подтянут к земле. Если на входе 0, прошивка считает шнур вытащенным:

1. `GameTask` запускает матч и, если `gripper_cmd == 0xFF`, отдаёт команду захвату.
2. Через 99 секунд выполняются `MaxonsStop()`, `StopAllActuators()` и `COMM_ENABLE = 0`.
3. После этого робот отвечает по UART, но команды движения (25 обработчиков с проверкой `COMM_ENABLE`) игнорирует.

На текущем роботе чтение по SWD показало PC15 = 1: шнур считается вставленным, матч сам не стартует. Пины 4–7 разъёма I2C2 не подключены, значит, высокий уровень приходит через общий пин 27 разъёма 28 GPIO. Если вход окажется неподключённым, робот через 99 секунд после включения перестанет выполнять команды движения.

Обходные пути:

- подключить стартовый шнур;
- на время тестов собрать прошивку с `robot_ctrl.robot_mode = DEBUG_MODE` в [game_task.c](Core/OS_Tasks/game_task.c). Остановка по таймеру работает только в `GAME_MODE`. Команды для смены режима по UART нет.

### Найдено: широковещательный ID Dynamixel у захвата 1

В [gripper_task.c](Core/OS_Tasks/gripper_task.c) у захвата 1 стоит `dyn_main_1 = 254`. ID 254 (`0xFE`) в протоколе Dynamixel — широковещательный адрес. Любая команда «первому мотору подъёма» уходит всем сервам на шине USART3, а ответа от этого адреса не бывает, поэтому `WaitDynamixelAngle` всегда ждёт до тайм-аута. Нужно выставить реальный ID сервы. Пока не исправлено.

### Сервоприводы: куда подключены

| Что | Интерфейс и разъём | Адресация в прошивке |
|---|---|---|
| Dynamixel AX | USART3 в полудуплексе: PD8 — данные, PD9 — направление. Разъёмы `GND / +12V / DIO` справа | Все разъёмы — одна шина. Какая серва отвечает, определяет ID, а не разъём. Захват 1: подъём 254 и 112, боковые 7 и 8. Захват 2: подъём 1 и 2, боковые 16 и 25. Рейка (pinion) — 13, cursor — 98 |
| Серво через PCA9685 | I2C2 (PB10/PB11), адрес 0x40 | Захват 1: блоки CH1–CH4, сдвиг CH5/CH6. Захват 2: блоки CH7–CH10, сдвиг CH11/CH12 |
| RC-серво (ШИМ) | разъёмы PWM | Используются только в `DemoTask` (`SERVO_DEMO`) |
| Шаговый двигатель 1 | Stepper Output 1 | Подъёмник, калибровка по `LINEAR_SWITCH_2` (пин 26) |

### Как читать распиновку основной платы

- **Разъёмы SPI2, SPI3, I2C1, I2C2.** Число на шелкографии — это «Connector Pin» в таблице с тем же именем разъёма. Пин 1 всегда GND, пин 8 всегда +5V.
- **Разъём «28 GPIO».** Каждый ряд: `+5V | нечётный | чётный | +5V | GND`. Число — это «Board Pin Number», по нему в таблице находится GPIOxx и ножка STM32.
- **«Shared with».** Одна ножка STM32 выведена на два разъёма и электрически это один провод. Например, GPIO26 (28 GPIO, пин 26) = PC13 = I2C1, пин 5.
- **Роль ножки в прошивке.** [board_gpio_defines.h](Core/BSP/Include/board_gpio_defines.h) и [robot_gpio_defines.h](Core/App/Include/robot_gpio_defines.h).

| Разъём, пин | STM32 | В прошивке |
|---|---|---|
| I2C2, пин 2 / 3 | PB10 / PB11 | SCL / SDA к PCA9685 |
| 28 GPIO, пин 26 (= I2C1, пин 5) | PC13 | `LINEAR_SWITCH_2`, концевик калибровки шагового мотора |

Пины 4–7 разъёма I2C2 на этом роботе не подключены. В прошивке они заведены как кнопки: PE5 — `STRATEGY_BTN`, PC15 — `STARTING_CORD_BTN`, PE6 — `PLAYGROUND_SIDE_BTN`, PE2 — `MAXON_ENABLE_BTN`. Это же пины 23, 27, 25 и 21 разъёма 28 GPIO.

Левый нижний блок 1–8 и разъёмы `GND / +12V / DIO` справа в таблицах не описаны. Их нужно смотреть в полной схеме или прозванивать мультиметром до пинов разъёма 28 GPIO.

### SSH к reset10

- На Mac создан новый ключ `~/.ssh/id_ed25519` без passphrase. Старый, с passphrase, сохранён как `~/.ssh/id_ed25519.old`. Публичный ключ добавлен в `authorized_keys` пользователя `reset10`.
- В `~/.ssh/config` добавлен хост `reset10` (`reset10.local`, пользователь `reset10`). Подключение: `ssh reset10`.

## 2026-10-19

- Перенос с Keil/ARMCC на CMake/GCC: linker script, GNU-стартап, GCC-порт FreeRTOS, проверка образа `check`.
- I2C2: тайм-ауты и обработка ошибок вместо бесконечного ожидания флага `ADDR`; частота APB1 исправлена с 84 на 42 МГц. Результат инициализации PCA9685 сохраняется в `PCA9685_InitStatus`, и ошибка PCA9685 не мешает запуску FreeRTOS.
- Подробности: [MIGRATION_REPORT.md](MIGRATION_REPORT.md), [docs/USAGE_RU.md](docs/USAGE_RU.md).
