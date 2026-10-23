# Changelog

## 2026-10-24

### Переход на C++: драйверы BSP и Lib переведены на классы

- Каждый драйвер теперь класс, а не структура с указателями на функции. Глобальный объект называется как раньше (`Stepper`, `PWM`, `L6474H`, `WS2812B`, `USARTx`, `Dynamixel_AX`, `Maxon`, `BMS`, `Button`, `PCA9685`, `Serial`, `HL_RTU` и др.), поэтому вызовы вида `Stepper.StepToPosisition(...)` не изменились.
- Состояние, которое лежало в глобальных переменных файлов, перенесено в поля классов: позиции шаговых моторов, регистр SPI_GPO, таймауты USART, кнопки, FIFO, настройки PCA9685.
- Без динамической памяти и рантайма C++: все объекты объявлены `constinit`, кода инициализации при старте нет; `malloc`, `new`, исключения и RTTI не используются. Для виртуальных функций в [cmake/gcc_runtime.c](../cmake/gcc_runtime.c) добавлена заглушка `__cxa_pure_virtual`.
- Из заголовков BSP и Lib убраны `extern "C"`: ни один C-файл их не подключает.
- FLASH (Debug): 70108 → 62548 Б. RAM: 19656 → 19416 Б.

### Переход на C++: App

| Было | Стало |
|---|---|
| `Kinematics` (таблица функций) + `Kinematics.MatrixInit()` при старте | `KinematicsModule Kinematics`: коэффициенты и матрицы считаются при компиляции, `MatrixInit()` удалён |
| `Odometry` + глобальные `path`, `ticks_old` | `OdometryModule Odometry{Maxon, Kinematics}`, состояние внутри класса |
| `MaxonsEnable()`, `MaxonsDisable()`, `MaxonsStop()`, `StopAllActuators()`, `ButtonsCallback()` | `RobotBehavior Robot`: `Robot.MaxonsEnable()` и т.д. |
| `Terminal`, `Debug_Terminal` (таблицы функций) | `TerminalModule Terminal` (HL_RTU, USART1), `DebugTerminalModule Debug_Terminal` (Serial, UART5); у debug-терминала методы `SendResponse` / `GetRequest` без префикса `Debug` |
| массивы `terminal_cmd_handlers[]` / `debug_terminal_cmd_handlers[]`, глобальные `COMM_ENABLE` / `DEBUG_COMM_ENABLE` | `TerminalCmd.Handle(&request)`, `DebugTerminalCmd.Handle(&request)`; обработчики стали методами, таблица методов лежит во FLASH; флаг связи — поле класса, `TerminalCmd.SetCommEnable(0)` в `game_task` |

- Зависимости модулей App передаются в `constexpr`-конструктор (`Odometry{Maxon, Kinematics}`, `Robot{Maxon}`), объекты `constinit`.
- `robot_ctrl`, `bus_devices` и `Robot_GPIO_Config()` остались как были.
- У `debug_terminal.hpp` и `debug_terminal_cmd_handlers.hpp` теперь свои include guard'ы. Раньше они совпадали с `terminal*.hpp`, и `debug_terminal.hpp` фактически не подключался.
- Удалены неиспользуемые `debug_terminal_cmd_table.hpp`, `PrepareRobot()`, `SpeedTo_TPM_2()` (было только объявление) и `GetWheelBusDevice()`.
- Обработчики концевиков в `ButtonsCallback` сведены к общему помощнику, инверсия «нажат → `LINEAR_SWITCH_RELEASED`» сохранена.
- Исправлено: `terminal_task` и `debug_task` читали `handlers[cmd]` до проверки `cmd < TERMINAL_CMD_TABLE_SIZE`, то есть за пределами таблицы при неизвестной команде. Теперь проверка в `Handle()` идёт первой.
- Исправлено: `Odometry.GetSpeed` при ошибке связи с Maxon считал скорость из неинициализированных значений; теперь для неответившего колеса берётся 0.
- FLASH (Debug): 62548 → 62884 Б. RAM: 19416 → 19208 Б.

### HwInstance: вся периферия в одном объекте

- Новая структура `HwInstance` в [hw_instance.hpp](../Core/App/Include/hw_instance.hpp) хранит все драйверы BSP и Lib: `stepper`, `l6474h`, `pwm`, `spi_gpo`, `ws2812b`, `usartx`, `usartx_timeout`, `i2c2`, `dynamixel_bus3/4/6`, `serial`, `hl_rtu`, `dynamixel_ax`, `dynamixel_mx`, `maxon`, `bms`, `pca9685`, `rc_servo`, `h_bridge`, `button`.
- Объект один, статический, создаётся при компиляции (`constinit`), копирование запрещено. Привязка устройств к шинам Dynamixel задаётся в конструкторе `HwInstance`.
- `GetHwInstance()` возвращает ссылку на него. Функция `constexpr`, поэтому поля можно передавать в конструкторы других `constinit`-объектов:

  ```cpp
  constinit OdometryModule Odometry{GetHwInstance().maxon, Kinematics};
  ```

- Так теперь создаются `Odometry`, `Robot`, `Terminal` (получает `hl_rtu`) и `Debug_Terminal` (получает `serial`). `main()` и обработчики прерываний обращаются к драйверам через `GetHwInstance()`.
- Старые глобальные имена (`Stepper`, `Maxon`, `DynamixelBus3`, …) остались как ссылки на поля `HwInstance` (определены в [hw_instance.cpp](../Core/App/hw_instance.cpp)). Существующий код работает без изменений, второй копии драйвера нет.
- Цена: `hw_instance` (3476 Б) лежит в `.data`, потому что в нём есть ненулевые поля (vtable протоколов, ссылки на шины, настройки PCA9685). Образ объекта хранится во FLASH и копируется в RAM при старте. FLASH (Debug): 62884 → 66784 Б. RAM не изменилась.

### USART: новый интерфейс протоколов

- Таблицы `usartx_protocol_s`, `usartx_control_s` и `usartx_line_config_s` заменены классами в [usartx_communication.hpp](../Core/BSP/Include/usartx_communication.hpp):
  - `USARTx_Line` — линия с буферами, FIFO и методами `StartTx`, `StopTx`, `StartRx`, `StopRx`, `SetRxTimeout`, `ResetRxTimeout`;
  - `USARTx_Protocol` — базовый класс протокола с виртуальными `InitLine`, `TxCompleteCallback`, `RxCompleteCallback`, `RxTimeoutCallback`. Протокол хранит свою линию: один объект — одна линия.
- `DynamixelMaster`, `Serial` и `HL_RTU` наследуются от `USARTx_Protocol`. Вместо общего `DynamixelMaster` теперь по объекту на линию: `DynamixelBus3` (USART3, сервы AX), `DynamixelBus4` (UART4, BMS), `DynamixelBus6` (USART6, Maxon).
- Привязка в [main.cpp](../Core/App/main.cpp): `hw.usartx.BindProtocol(USART3, hw.dynamixel_bus3)`, `hw.usartx.BindProtocol(UART5, hw.serial)` и т.д., где `hw = GetHwInstance()`; без `.GetProtocol()`.
- FIFO (`fifo_routine`) стал классом `FIFO` с методами `Init`, `Add`, `Remove`, `IsFull` и т.д.
- Обработчики прерываний вызывают методы драйверов: `GetHwInstance().stepper.TIM_IRQ_Process()`, `GetHwInstance().usartx.IRQ_Process(...)`, `GetHwInstance().usartx_timeout.TIM_IRQ_Process()`, `GetHwInstance().ws2812b.DMA_IRQ_Process()`.

### Шина внутри объекта, а не параметром

- Методы драйверов больше не принимают `USART_TypeDef* usart`: шина задаётся один раз при создании объекта. ID устройства остаётся параметром, так как ID серв вычисляются во время работы (сканирование, команды терминала).

  | Было | Стало |
  |---|---|
  | `Dynamixel_AX.SetAngle(USART3, id, angle)` | `Dynamixel_AX.SetAngle(id, angle)` |
  | `Maxon.Set_TPM(bus_devices[M].usart, bus_devices[M].id, tpm)` | `Maxon.Set_TPM(bus_devices[M].id, tpm)` |
  | `BMS.GetBatteryState(bus_devices[B].usart, ...)` | `BMS.GetBatteryState(bus_devices[B].id, ...)` |
  | `DynamixelMaster.SyncWrite(usart, ids, ...)` | `DynamixelBus3.SyncWrite(ids, ...)` |
  | `Serial.WriteBytes(UART5, bytes, n)` | `Serial.WriteBytes(bytes, n)` |
  | `HL_RTU.SendResponse(USART1, response)` | `HL_RTU.SendResponse(response)` |

- Шины устройств: `Dynamixel_AX` и `Dynamixel_MX` — `DynamixelBus3`, `Maxon` — `DynamixelBus6`, `BMS` — `DynamixelBus4`. Шина передаётся в `constexpr`-конструктор, объекты по-прежнему `constinit`.
- Из `bus_device_s` удалено поле `usart`: таблица [bus_device.cpp](../Core/App/bus_device.cpp) хранит только ID.
- `RotateTwoDynamixels` в `gripper_task` принимает шину (`DynamixelMasterDriver&`), а не `USART_TypeDef*`.
- Индексы моторов и выходов (`Stepper`, `L6474H`, `PWM`, `RC_Servo`, `H_Bridge`, `Button`) и адрес платы PCA9685 остались параметрами методов по тому же принципу.

### Другие изменения API

- `SPI_GPO_SetOutputPin(x)` / `SPI_GPO_ResetOutputPin(x)` → `SPI_GPO.SetOutputPin(x)` / `SPI_GPO.ResetOutputPin(x)`.
- `PCA9685_I2C_Write/Read` → `I2C2_Bus.Write/Read`, новый заголовок [i2c2.hpp](../Core/BSP/Include/i2c2.hpp). Имя `I2C2` занято макросом CMSIS.
- Удалены пустые заглушки Dynamixel `RegWrite`, `Action`, `FactoryReset`, `Reboot`, `BulkRead`: они ничего не делали и нигде не вызывались.
- Четыре функции `StepperMotor_N_Init` объединены в `Stepper.Init(motor)` с таблицей настроек; значения не изменились.

### Исправлено

- **`SyncWrite` (Dynamixel) портил стек.** Цикл `do {…} while (boards_number--)` делал лишний проход с индексом 255. `memcpy` писал примерно на 768 байт за пределы локальной переменной, а при `response_array == NULL` запись шла ещё и по адресу `NULL + 9180`. Вызывается из `gripper_task`.
- **`Write` (Dynamixel) с `response = NULL` писал по адресу 0.** Так вызываются почти все команды AX (`SetAngle`, `TorqueEnable`, …).
- **`BMS.GetPowerConsumption` отсутствовал в таблице функций**: поле было `NULL`, вызов уронил бы прошивку.
- **`Dynamixel_MX.GetAngle`** читал 2 байта в неинициализированную 4-байтную переменную, поэтому угол мог получаться неверным.
- `PCA9685.SetServoAngleWithAddr` больше не подменяет на время вызова адрес платы, сохранённый в `Init`.

### Не проверено на железе

Сборка Debug и Release и `make check` проходят. Запуск на плате не проводился: шаговые моторы, Dynamixel, BMS, Maxon, терминал (UART1), кнопки и светодиодная лента требуют проверки.

## 2026-10-23

### Сборка: только CMake, Keil удалён

- Проект собирается только через CMake + Ninja + `arm-none-eabi-gcc`. Keil MDK-ARM больше не нужен.
- `cmake/keil_sources.cmake` переименован в [cmake/sources.cmake](../cmake/sources.cmake). Новый `.c` файл добавляется в `EUROBOT_SOURCES`.
- Убраны include-пути `MDK-ARM/.cmsis/include` и `MDK-ARM/RTE/_Main_Board`, define `_RTE_`, Keil-директивы `#pragma O0` в `l6474_drvr.c` и `spi_gpo.c`. Для `l6474_drvr.c` `-O0` по-прежнему задаётся в CMake.
- [cmake/check_firmware.py](../cmake/check_firmware.py) больше не сверяется с `Main_Board.uvprojx`. Вектора прерываний проверяются по GNU-стартапу. Опция `--check-originals` удалена.
- Прошивка через STM32CubeProgrammer (`stm32_flash.cmake`, `programmer.cmake`) заменена на OpenOCD: [cmake/openocd.cmake](../cmake/openocd.cmake), цели `flash` и `reset`. ST-LINK выбирается через `-DOPENOCD_ADAPTER_SERIAL=<serial>`.
- [tools/monitor_stm32.py](../tools/monitor_stm32.py) читает память через OpenOCD, а не через CubeProgrammer.
- Итоговые `firmware.bin` в Debug и Release побайтово совпадают со сборкой до чистки.

### Удалено

- `MDK-ARM/`, `ARMCC_Include/`, `cmake/migration/`, `README_CMAKE.md`. Всё это остаётся в истории git.
- FreeRTOS: все порты, кроме `portable/GCC/ARM_CM4F`; `MemMang` не нужен, так как выделение памяти только статическое.
- CMSIS: `Core_A`, дубликат `Core`, шаблоны стартапа для ARM/IAR, неиспользуемые исходники DSP.
- STM32F4 HAL/LL: все `.c` и `.h`, которые не участвуют в сборке. Если понадобится новая периферия, файлы берутся из пакета STM32CubeF4.

### Инструменты

- [Makefile](../Makefile) поверх CMake-пресетов: `make`, `make release`, `make check`, `make check-release`, `make flash`, `make flash-release`, `make flash-remote`, `make reset`, `make monitor`, `make clean`, `make help`.
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

**Исправление.** `DemoTask.CreateTask(4)` в [main.c](../Core/App/main.cpp) закомментирован, как было в исходной Keil-прошивке. В `monitor_stm32.py` убрана проверка мигания выходов, ожидаемое число задач снижено до 8.

**Правило на будущее.** `DemoTask` включать только вместе с каким-нибудь демо-режимом в `demo_task.c`. Любая задача с пустым циклом и высоким приоритетом блокирует терминал.

### I2C2: частота шины

Ненадолго пробовали вернуть 84 МГц в `LL_I2C_ConfigSpeed` для нового робота, но затем откатили. В коде осталось 42 МГц, что соответствует реальной частоте APB1 (168 МГц / 4), шина работает на 100 кГц. Значение 84 МГц из исходной прошивки давало около 50 кГц.

### Расследование: зависание при отключении разъёмов (не закрыто)

**Симптом.** Отключение под питанием разъёма I2C2 (используются GND, SCL, SDA и +5V; пины 4–7 ни к чему не подключены), пина 26 разъёма 28 GPIO, +5V и +12V разъёмов с сервами и другой нагрузкой: контроллер перестаёт отвечать по UART.

**Программную причину не нашли.**

- I2C2/PCA9685: каждое ожидание в [i2c2.c](../Core/BSP/i2c2.cpp) ограничено 20 мс. Вызовы из `gripper_task` перемежаются `vTaskDelay`. Без PCA9685 команды серво просто возвращают ошибку.
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
- на время тестов собрать прошивку с `robot_ctrl.robot_mode = DEBUG_MODE` в [game_task.c](../Core/OS_Tasks/game_task.cpp). Остановка по таймеру работает только в `GAME_MODE`. Команды для смены режима по UART нет.

### Найдено: широковещательный ID Dynamixel у захвата 1

В [gripper_task.c](../Core/OS_Tasks/gripper_task.cpp) у захвата 1 стоит `dyn_main_1 = 254`. ID 254 (`0xFE`) в протоколе Dynamixel — широковещательный адрес. Любая команда «первому мотору подъёма» уходит всем сервам на шине USART3, а ответа от этого адреса не бывает, поэтому `WaitDynamixelAngle` всегда ждёт до тайм-аута. Нужно выставить реальный ID сервы. Пока не исправлено.

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
- **Роль ножки в прошивке.** [board_gpio_defines.h](../Core/BSP/Include/board_gpio_defines.hpp) и [robot_gpio_defines.h](../Core/App/Include/robot_gpio_defines.hpp).

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

### Сборка перенесена с Keil/ARMCC на CMake/GCC

**Итог.** Сборка перенесена и проверена компиляцией: чистая Debug, Release и прямая сборка в `build/` проходят аудит `check`. Создаются `firmware.elf`, `.hex`, `.bin`, `.map`; FLASH и RAM не переполнены. На этом этапе запуск на STM32, периферия и поведение робота не проверялись.

- Исходный проект — единственный Keil-target `Main_Board`: MCU `STM32F407VGTx`, ARMCC 5.06 update 7 build 960. Отдельного target для синей платы нет. Совпадение с физической платой без маркировки и подключения установить нельзя.
- GCC 14.3.1 и STM32CubeProgrammer 2.23.0 взяты из установленной STM32CubeIDE. CubeMX и CubeCLT не понадобились.
- Все 2514 файлов, существовавших до начала работы (Keil, старые AXF/MAP, vendor, `ARMCC_Include`), сохранены побайтово; проверка шла по SHA256, а не по Git HEAD. В исходной рабочей копии уже было 1900 незакоммиченных изменений, они не сбрасывались.
- ST-LINK не был подключён (`-l stlink` → `No ST-Link detected!`). `flash`, `backup`, `verify` и `reset` не запускались; erase, RDP и Option Bytes не трогались.
- Список исходников в CMake явный, без GLOB, и совпадает с Keil-target (82 файла), кроме двух замен: FreeRTOS `portable/RVDS/ARM_CM4F/port.c` → `portable/GCC/ARM_CM4F/port.c` и ARMASM-стартап `MDK-ARM/startup_stm32f407xx.s` → GNU-стартап из `Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/`.

#### Параметры Keil → GCC

| Параметр | Keil | GCC/CMake |
|---|---|---|
| Device | STM32F407VGTx, DFP 2.17.1 | `STM32F407xx` |
| CPU | Cortex-M4.fp.sp, little-endian, Thumb-2 | `-mcpu=cortex-m4 -mthumb` |
| FPU / float ABI | FPv4-SP-D16, аргументы в VFP-регистрах | `-mfpu=fpv4-sp-d16 -mfloat-abi=hard` |
| Flash | 0x08000000, 1 МБ | те же начало и размер |
| SRAM | 0x20000000, 128 КБ | данные и heap до 0x2001FC00, верхние 0x400 — отдельный STACK |
| CCM | 0x10000000, 64 КБ, RW_IRAM2 пуст | описана и остаётся пустой; ASSERT запрещает необработанную CCM-секцию |
| enum / wchar | small enum, wchar = 2 | `-fshort-enums -fshort-wchar` |
| char | unsigned по умолчанию ARMCC | `-funsigned-char` |
| C | `--c99` | `-std=c99` |
| Оптимизация | `-O0`, split sections | Debug `-O0`, Release `-Os`; function/data sections, `--gc-sections` |
| LTO | выключен | выключен |
| Отладка | debug = 1 | Debug `-g3`, Release `-g` |
| Stack | 0x2001FC00..0x2001FFFF, начальный SP 0x20020000 | те же адреса и размер |
| Heap | 0x200 в стартапе, но Keil удалял HEAP как неиспользуемый | зарезервировано 0x200 после bss; аллокатора FreeRTOS нет |
| Linker | ArmLink, `STM32F407.sct`, MicroLIB | GNU ld, новый [STM32F407VG_FLASH.ld](../cmake/STM32F407VG_FLASH.ld), `--gc-sections`, MAP |
| Библиотеки | MicroLIB | newlib-nano, libm, libgcc, nosys, без semihosting |
| Boot offset | нет | 0, векторы в 0x08000000 |

- ABI определён по сохранённому `MDK-ARM/build/Keil/main.o` (`readelf -A`), а не по имени MCU: в исходном AXF нет блока attributes.
- `-fno-strict-aliasing` добавлен из-за найденных обращений к хранилищу указателей через `uint32_t*` в Dynamixel и WS2812B. Исходники при этом не переписывались.
- ARMCC помечает модель FP как Finite/denormal sign only, GCC работает по IEEE 754. Fast-math для имитации не включался. Совпадение численных краевых случаев не установлено, одометрию и FPU нужно сравнить на стенде.
- Defines: перенесены `USE_FULL_LL_DRIVER`, `ARM_MATH_CM4`, `__FPU_PRESENT=1U` и добавленные Keil `STM32F407xx`, `_RTE_`. `__MICROLIB`, `__UVISION_VERSION`, `__CC_ARM` не подделывались. `ROBOT_BIG` и `ROBOT_GUIDO` в этом target не определены.
- Include-пути перенесены в прежнем порядке (14 путей). Единственная замена — `portable/RVDS/ARM_CM4F` на `portable/GCC/ARM_CM4F`. `ARMCC_Include` не подключён, чтобы GCC не подхватил стандартные заголовки чужой runtime.

#### Совместимость с GCC

| Компонент | Проблема | Решение |
|---|---|---|
| `MDK-ARM/startup_stm32f407xx.s` | синтаксис ARMASM, `__main`, heap MicroLIB | взят уже имеющийся GNU-стартап |
| FreeRTOS `portable/RVDS/ARM_CM4F` | inline asm ARMCC | порт `portable/GCC/ARM_CM4F` той же версии |
| CMSIS и CMSIS-DSP | ветки по компилятору | используется существующая ветка `__GNUC__`, заголовки не менялись |
| `l6474_drvr.c` | `#pragma O0` GCC игнорирует | `-O0` для этого файла задан в CMake |
| `dynamixel_master.c`, `ws2812b.c` | type punning хранилища указателей | `-fno-strict-aliasing` |
| GNU-стартап → `__libc_init_array` | с `-nostartfiles` нет `_init` | новый [gcc_runtime.c](../cmake/gcc_runtime.c) с пустыми `_init` / `_fini` |
| wchar ABI | у Keil `wchar_t` 2 байта, у библиотек GNU — 4 | ABI приложения сохранён; 22 предупреждения линкера оставлены открытыми, wide-char API в приложении нет |

В Core нет активных `__irq`, `__declspec`, Keil-only inline asm, `#pragma pack` и нестандартных бинарных библиотек. Побайтовая идентичность исполняемого кода с Keil не заявляется.

#### FreeRTOS и прерывания

- Ядро — V10.4.3 LTS Patch 3 (комментарий в `FreeRTOSConfig.h` устарел и упоминает 10.0.1). Ядро и конфигурация не обновлялись. Heap_1..5 не добавлялись: `configSUPPORT_STATIC_ALLOCATION=1`, `DYNAMIC=0`.
- Тик 1000 Гц, `configMAX_PRIORITIES=7`, приоритет ядра 0xF0, max syscall 0xB0. `NVIC_SetPriorityGrouping(0)` в `main` не менялся. Порт сохраняет контекст FPU, включён lazy stacking.
- SVC, PendSV и SysTick направлены на обработчики порта. 14 обработчиков из `stm32f4xx_it.c` и 3 обработчика RTOS проверены как strong-символы, а не default-алиасы.
- Все 98 векторов совпадают по позиции и смыслу. Единственное различие имён — IRQ48: `FMC_IRQHandler` в ARMASM-шаблоне и `FSMC_IRQHandler` в GNU-шаблоне; оба ведут в default handler.

#### Память и стартап

- Linker script составлен по scatter-файлу Keil и двум сохранённым MAP. Все данные, включая DMA-буферы, лежат в SRAM; автоматического переноса в CCM нет, чтобы DMA не получил адрес CCM.
- Векторы в 0x08000000 (392 байта, KEEP). `.data` хранится во FLASH и копируется в SRAM; `.bss`, `.heap`, `.stack` — NOLOAD, в HEX адресов SRAM нет.
- Стартап копирует `.data`, обнуляет `.bss`, вызывает `SystemInit` (FPU, VTOR), `__libc_init_array` и `main`. Reset_Handler проверен по дизассемблеру, выполнение на MCU — нет.
- Резерв heap 512 байт — небольшое осознанное отличие от Keil, который удалял неиспользуемый HEAP. `malloc` и `_sbrk` приложению не нужны.

#### Периферия (настройки не менялись)

`.ioc` нет, настройки взяты из `main.c`, `board_hw_config.c` и GPIO-файлов:

- тактирование: HSE 16 МГц, PLL M=8 / N=168 / P=2 / Q=4, SYSCLK 168 МГц, APB1 /4, APB2 /2, Flash latency 5, включается LSI. Поле CLOCK(12000000) в Keil — метаданные отладчика, а не настройка PLL;
- I2C2 и `PCA9685.Init(0x40, 50, 500, 2500, 180)`;
- SPI1 и `SPI_GPO_ResetOutputPin(SPI_GPO_ALL)`;
- USART1 (терминал) 921600, USART3 (Dynamixel) 1000000, UART4 (RS485) 2000000, UART5 (Serial) 115200, USART6 (RS485) 2000000;
- TIM1 — шаговые моторы, TIM4/TIM5 — группы PWM, TIM8 — WS2812B, TIM14 — тайм-ауты USART;
- `IWDG_Init` в `main` закомментирован исходно и не включался;
- компилируются те же 18 LL-файлов STM32F4 HAL 1.7.13, что и в Keil, HAL `.c` в target нет.

#### Добавленные файлы

Старые файлы не изменялись. Добавлены:

| Файл | Назначение |
|---|---|
| `CMakeLists.txt`, `CMakePresets.json` | цель `firmware`, флаги CPU/FPU/ABI, пресеты Debug/Release, цель `check` |
| `cmake/arm-none-eabi.cmake` | toolchain: GCC/G++ и инструменты ARM, автопоиск пакета |
| `cmake/keil_sources.cmake` | 82 исходника и 14 include-путей из Keil (2026-10-23 переименован в `sources.cmake`) |
| `cmake/STM32F407VG_FLASH.ld` | FLASH, SRAM, CCM, stack и символы стартапа |
| `cmake/gcc_runtime.c` | пустые `_init` / `_fini` для newlib |
| `cmake/stm32_flash.cmake`, `cmake/programmer.cmake` | ручные цели `probe`, `backup`, `flash`, `verify`, `reset` через CubeProgrammer (2026-10-23 заменены на OpenOCD) |
| `cmake/check_firmware.py` | аудит состава сборки, ABI, векторов, памяти, HEX/BIN и хэшей исходных файлов |
| `cmake/migration/*.json` | SHA256 и инвентаризация всех 2514 исходных файлов (удалены 2026-10-23) |
| `README_CMAKE.md`, `MIGRATION_REPORT.md` | инструкция и отчёт о переносе (`README_CMAKE.md` удалён 2026-10-23, отчёт перенесён в этот файл) |

#### Проблемы при переносе и их решение

1. Все 82 исходника сразу скомпилировались GCC: ошибок из-за пропавших файлов, include или defines не было.
2. Линковка остановилась на `undefined reference to _init` из `__libc_init_array` при `-nostartfiles` → добавлен `gcc_runtime.c`.
3. CMake не находил CubeProgrammer: проверка шла по платформе цели `APPLE`, а не хоста → исправлено на `CMAKE_HOST_APPLE`.
4. Различие FMC/FSMC в векторах между двумя шаблонами ST учтено в аудите, стартап не правился.
5. Release-диагностика указала на type punning в Dynamixel/WS2812B → отключена strict-aliasing-оптимизация (это параметр генерации кода, а не подавление предупреждения).
6. objcopy заполняет промежутки FLASH-образа значением 0xFF в HEX и BIN; иначе BIN содержит нули и `verify` даёт ложное расхождение.
7. Аудит проверяет фактические адреса и роли символов, а не только написание имён.

#### Открытые предупреждения на момент переноса

Флаги `-w`, `--no-wchar-size-warning` и массовые `-Wno-*` не применялись. Debug — 229 предупреждений (`-Wcomment` 70, `-Wunused-parameter` 128, `-Wmisleading-indentation` 3, `-Wsign-compare` 2 и др.) плюс 22 предупреждения линкера о wchar. Release добавляет одно `-Wmaybe-uninitialized`.

- `robot_behavior_routine.c:115` дважды меняет `strategy` в одном выражении (UB). Логика не менялась, решение за владельцем кода.
- `gripper_task.c:1036..1089`: `move_speed` не задаётся при `dir` не равном 0 или 1.
- `-Wcomment` вызваны строками `// ... \` в `FreeRTOSConfig.h`.
- `-Wmisleading-indentation` — в `VerifyPacket` Dynamixel/HL_RTU и в ожидании LSI.

#### Размеры

| Сборка | text | data | bss (с резервами heap/stack) | BIN | Результат |
|---|---:|---:|---:|---:|---|
| Debug `-O0` | 65 604 | 2 812 | 17 860 | 68 424 | `check` PASS |
| Release `-Os` | 39 596 | 2 764 | 17 852 | 42 368 | `check` PASS |

Debug: FLASH 6.53 %, RAM без STACK 19 648 / 130 048 байт, STACK 1024 / 1024, CCM 0. Keil MAP: load region 0x405CC, ER_IROM1 0x3FA38, RW_IRAM1 0x55B4. Keil и GCC по-разному удаляют неиспользуемые секции и размещают библиотеки, поэтому меньший размер GCC не означает потерю функциональности.

Проверка `check` охватывает: 82 исходника, отсутствие дублей, все include/defines и флаги CPU/FPU/ABI, ELF ARM32 little-endian hard-float, 98 векторов, Thumb Reset и MSP, 17 strong ISR, отсутствие пересечения данных со стеком, начальные значения `.data` в BIN, контрольные суммы HEX и совпадение HEX с BIN. Это статическая проверка, а не запуск на MCU.

Команда PCA9685 `0x1D` объявлена в заголовке, но не имела обработчика в `terminal_cmd_handlers[]`; перенос это состояние сохранил.

#### Что оставалось сделать перед первой загрузкой

1. Подключить ST-LINK и питание, выполнить `probe`, подтвердить STM32F407VG, Device ID 0x413 и FLASH 1024 КБ.
2. Выполнить `backup` и сохранить 1 МБ вне `build/`. При включённом RDP защиту не снимать и память не стирать.
3. Убедиться, что в секторах 0–4 нет загрузчика и нужных данных.
4. Получить разрешение владельца на запись, затем `flash` с `verify` и `reset`.
5. Проверить тик, тактирование, UART, RTOS, FPU, запас MSP и стеков задач.
6. Разобраться с ошибками `strategy` и `move_speed` до допуска движения.
7. На стенде по очереди проверить RS485/Maxon/BMS, Dynamixel, GPIO/PWM/DMA, PCA9685/I2C, шаговый механизм, насосы, клапаны и захваты; Release — отдельным прогоном после Debug.

Справочные источники: [GCC ARM Options](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html), [GCC Code Gen Options](https://gcc.gnu.org/onlinedocs/gcc/Code-Gen-Options.html), [Keil ARM compiler options](https://www.keil.com/support/man/docs/uv4/uv4_dg_adscc.asp), [ST RM0090](https://www.st.com/resource/en/reference_manual/rm0090-stm32f4xx-reference-manual-stmicroelectronics.pdf).

### Подключение программатора: режим NORMAL по умолчанию

После ручного запуска `flash` подключение в режиме UR завершилось `DEV_TARGET_RESET_ERR` ещё до программирования. Весь 1 МБ FLASH повторно прочитан через HOTPLUG и совпал с резервной копией (SHA256 `ff5de93b152beaf3a8719db26fc893373702379cddbbe50feb01436de1af01d2`).

- В `cmake/stm32_flash.cmake` добавлен `STM32_FLASH_CONNECT_MODE`, по умолчанию `NORMAL`. `cmake/programmer.cmake` использует NORMAL с reset=SWrst или явно выбранный UR с reset=HWrst; UR требует подключённого NRST.
- Автоматического повтора записи при ошибке нет. `probe`, `backup` и `verify` подключаются через HOTPLUG.

### Исправлено: зависание при старте без PCA9685

**Симптом.** После записи через `st-flash` CPU работал без HardFault, но FreeRTOS не запускался.

**Причина.** Прошивка ждала флаг `I2C2 ADDR` без ограничения времени. В регистрах: `SR1.AF=1`, `SR1.ADDR=0`, `SR2.MSL=1`, `SR2.BUSY=1` — устройство по адресу 0x40 не подтвердило адрес.

**Исправление.**

- В `Core/BSP/i2c2.c` добавлены тайм-аут 20 мс, распознавание NACK/BERR/ARLO/OVR, STOP и восстановление ACK/POS после ошибки, корректная последовательность чтения 1, 2 и нескольких байт.
- В `Core/BSP/board_hw_config.c` входная частота I2C исправлена на реальную APB1 = 42 МГц. Адрес 0x40, PB10/PB11 и скорость 100 кГц сохранены.
- `main` сохраняет результат в `PCA9685_InitStatus` и запускает задачи, даже если модуль недоступен. Значения: 0 — успех, −2 — тайм-аут, −3 — NACK, −4 — ошибка шины, −5 — потеря арбитража, −6 — переполнение.

Debug и Release прошли `check`. Debug: 69 516 байт, reset `0x0800e155`, SHA256 BIN `360eadd1a9368601bc39f8fcbaf20d0532582d73822365f54fac48cfc0898d7c`. Release: 42 992 байта, reset `0x08007d21`. Работу нового образа на плате нужно было подтвердить после ручной загрузки.

Инструкция по сборке и прошивке: [docs/USAGE_RU.md](USAGE_RU.md).
