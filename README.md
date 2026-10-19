# Eurobot 2026 — STM32F407 firmware

Прошивка основной платы Eurobot 2026 для STM32F407VGTx. Проект сохраняет исходную структуру Keil MDK-ARM и дополнительно собирается на macOS/Linux через CMake, Ninja и `arm-none-eabi-gcc`.

Актуальная аппаратная конфигурация:

- STM32F407VGTx, Cortex-M4F, hard float;
- Flash 1 МБ, SRAM 128 КБ, CCM 64 КБ;
- FreeRTOS 10.4.3 LTS Patch 3;
- адрес PCA9685 `0x40`, I2C2: PB10/SCL и PB11/SDA, 100 кГц;
- Debug включает `DemoTask`, который переключает `SPI_POWER_OUTPUT_1…4` каждые 250 мс.

## Зачем понадобились изменения

Исходный проект собирался только ARMCC 5 из Keil. Для GCC были добавлены CMake, linker script, GNU startup, GCC-порт FreeRTOS и минимальные runtime-функции. Список исходников, include-пути, defines, параметры Cortex-M4F и размещение памяти перенесены из единственной Keil-цели `Main_Board`.

Первая записанная CMake-сборка не запускала FreeRTOS: процессор навсегда оставался в инициализации PCA9685. Модуль не подтвердил I2C-адрес, а драйвер бесконечно ожидал флаг `ADDR`. В драйвер добавлены тайм-ауты и обработка NACK/ошибок, поэтому отказ PCA9685 больше не блокирует запуск прошивки. Одновременно исправлена входная частота I2C с ошибочных 84 МГц на реальные 42 МГц APB1. Последняя версия успешно запустилась на плате по результату аппаратной проверки владельца.

> **Уточнение о Keil → GCC:** смена компилятора не меняла физическое тактирование микроконтроллера. I2C2 и в Keil-, и в GCC-сборке получает 42 МГц от APB1; при переносе лишь обнаружили старое ошибочное значение 84 МГц в расчёте делителя I2C. Частоты UART уже соответствуют APB1/APB2 и из-за перехода на GCC не изменялись.

Подробное описание ошибок, изменений, команд и оставшихся тестов: [docs/USAGE_RU.md](docs/USAGE_RU.md).

## Быстрый старт

Требуются CMake ≥ 3.24, Ninja и ARM GNU Toolchain. На проверенном Mac используется GCC 14.3.1 из STM32CubeIDE.

```bash
git clone https://github.com/vladromenko/eurobot2026-stm.git
cd eurobot2026-stm

cmake --preset debug
cmake --build --preset debug
cmake --build --preset debug --target check
```

Результаты находятся в `build/debug/`:

- `firmware.elf` — ELF с символами;
- `firmware.bin` — образ для `st-flash`;
- `firmware.hex` — Intel HEX;
- `firmware.map` — карта размещения.

Загрузка на проверенную плату:

```bash
st-flash --serial 53004700090000393333574E \
  --hot-plug --reset \
  write "$PWD/build/debug/firmware.bin" 0x08000000
```

Для другой платы удалите `--serial` или подставьте серийный номер своего ST-LINK. До первой записи сохраните Flash и проверьте карту загрузки своей платы.

После загрузки:

```bash
python3 tools/monitor_stm32.py --project "$PWD"
```

Монитор только читает Flash/SRAM через SWD. Он проверяет совпадение образа, запуск FreeRTOS, число задач, tick, результат инициализации PCA9685 и переключение DemoTask.

## Состояние проверки

Debug и Release проходят встроенный аудит `check`. Debug был записан владельцем через `st-flash`; последние исправления PCA9685 позволили прошивке запуститься. Полный робот, механизмы, двигатели, UART/RS485, стек задач и Release-сборка ещё требуют стендовой проверки по [чек-листу](docs/USAGE_RU.md#что-еще-нужно-проверить).

Подробности переноса сохранены в [MIGRATION_REPORT.md](MIGRATION_REPORT.md), расширенная инструкция — в [README_CMAKE.md](README_CMAKE.md).
