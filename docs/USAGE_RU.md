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
- не менее 8 задач;
- tick растёт примерно на 1000 в секунду;

UART1 можно проверить read-only запросами:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install pyserial
python3 tools/serial_read_test.py --list
python3 tools/serial_read_test.py --port /dev/cu.НУЖНЫЙ_ПОРТ --command speed
```