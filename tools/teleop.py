#!/usr/bin/env python3
"""Keyboard teleoperation over USART1 using the terminal commands from terminal_cmd_table.h."""
import argparse
import curses
import math
import struct
import sys
import time

BOARD_PING = 0x00
BOARD_SET_SPEED = 0x04
BOARD_GET_SPEED = 0x05
BOARD_SET_COORD = 0x06
BOARD_GET_COORD = 0x07
BOARD_GET_BATTERY_STATE = 0x08
BOARD_IMPELLER_ON = 0x30
BOARD_IMPELLER_OFF = 0x31
BOARD_IMPELLER_SET_MV = 0x32
BOARD_IMPELLER_PRESS = 0x33
BOARD_IMPELLER_SYNC = 0x34

# enum impeller_state_e in impeller.h
IMPELLER_STATES = {0: 'выкл', 1: 'включается', 2: 'ВКЛ', 3: 'выключается'}
IMPELLER_SPEED_STEP = 5
IMPELLER_MV_MAX = 3300
IMPELLER_MV_DEFAULT = 3300  # IMPELLER_CALIBRATION_MV after reset
IMPELLER_PRESS_MAX_MS = 2000  # IMPELLER_DEBUG_PRESS_MAX_MS
# The firmware keeps one pending impeller request and takes it every 10 ms.
IMPELLER_CMD_GAP = 0.05

# key -> (dx, dy, dphi) direction; x forward, y left, phi counter-clockwise
MOTION_KEYS = {
    ord('w'): (1, 0, 0), curses.KEY_UP: (1, 0, 0),
    ord('s'): (-1, 0, 0), curses.KEY_DOWN: (-1, 0, 0),
    ord('a'): (0, 1, 0), curses.KEY_LEFT: (0, 1, 0),
    ord('d'): (0, -1, 0), curses.KEY_RIGHT: (0, -1, 0),
    ord('q'): (0, 0, 1),
    ord('e'): (0, 0, -1),
}

HELP = [
    'W/S, ↑/↓   вперёд / назад          A/D, ←/→  влево / вправо (боком)',
    'Q/E        поворот влево / вправо  Пробел    стоп',
    'M          моторы вкл/выкл (арм)   +/-       линейная скорость, [ ]  угловая',
    'T / Y      импеллер вкл / выкл     < / >     скорость импеллера -/+ 5 %',
    'J / K      опорное напряжение -/+  H         опорное 0 (управление скоростью)',
    'P          нажать кнопку импеллера U / N     считать импеллер вкл / выкл (без нажатия)',
    'I          пинг                    B         батарея',
    'V          скорость с энкодеров    C / R     координаты / сброс в 0',
    'X, Esc     выход (нулевая скорость и выключение импеллера)',
]


def frame(command, payload=b''):
    body = bytes([len(payload) + 2, command]) + payload
    return b'\xff\xff' + body + bytes([(~sum(body)) & 0xff])


class Robot:
    def __init__(self, port, timeout):
        self.port = port
        self.timeout = timeout

    def read_exact(self, length, deadline):
        result = b''
        while len(result) < length and time.monotonic() < deadline:
            result += self.port.read(length - len(result))
        if len(result) != length:
            raise RuntimeError(f'таймаут: получено {len(result)} из {length} байт')
        return result

    def request(self, command, payload=b''):
        # Firmware splits frames by line idle time, so send one frame and wait for its answer.
        self.port.write(frame(command, payload))
        self.port.flush()
        deadline = time.monotonic() + self.timeout
        previous = b''
        while time.monotonic() < deadline:
            current = self.port.read(1)
            if previous == b'\xff' and current == b'\xff':
                length_byte = self.read_exact(1, deadline)
                if length_byte[0] < 2:
                    previous = b''
                    continue
                rest = self.read_exact(length_byte[0], deadline)
                if (length_byte[0] + sum(rest)) & 0xff != 0xff:
                    raise RuntimeError('неверная контрольная сумма ответа')
                if rest[0] == command:
                    return rest[1:-1]
                previous = b''
                continue
            if current:
                previous = current
        raise RuntimeError(f'нет ответа на команду 0x{command:02X}')

    def ping(self):
        begin = time.monotonic()
        self.request(BOARD_PING)
        return (time.monotonic() - begin) * 1000

    def set_speed(self, dx, dy, dphi):
        self.request(BOARD_SET_SPEED, struct.pack('<fff', dx, dy, dphi))

    def get_speed(self):
        return struct.unpack('<fff', self.request(BOARD_GET_SPEED))

    def set_coord(self, x, y, phi):
        self.request(BOARD_SET_COORD, struct.pack('<fff', x, y, phi))

    def get_coord(self):
        return struct.unpack('<fff', self.request(BOARD_GET_COORD))

    def get_battery(self):
        data = self.request(BOARD_GET_BATTERY_STATE)
        if len(data) != 18:
            raise RuntimeError(f'длина данных батареи {len(data)} вместо 18')
        power, c1, c2, c3, c4, voltage, current, soc, cells = struct.unpack('<I4HHHBB', data)
        return power, (c1, c2, c3, c4)[:cells], voltage, current, soc

    def impeller(self, on, speed=0):
        """Returns (state, current speed %); the answer comes before the press and the ramp end."""
        if on:
            data = self.request(BOARD_IMPELLER_ON, bytes([speed]))
        else:
            data = self.request(BOARD_IMPELLER_OFF)
        if len(data) != 3:
            raise RuntimeError(f'длина ответа импеллера {len(data)} вместо 3')
        if not data[0]:
            raise RuntimeError('команда отклонена (COMM_ENABLE = 0: конец матча?)')
        return data[1], data[2]

    def impeller_answer(self, command, payload, title):
        data = self.request(command, payload)
        if len(data) != 3:
            raise RuntimeError(f'длина ответа импеллера {len(data)} вместо 3')
        if not data[0]:
            raise RuntimeError(f'{title} отклонено (неверный параметр или COMM_ENABLE = 0)')
        return data[1], data[2]

    def impeller_press(self, ms):
        """Button press only, the firmware state is unchanged; returns (state, speed %)."""
        return self.impeller_answer(BOARD_IMPELLER_PRESS, struct.pack('<H', ms), 'нажатие')

    def impeller_sync(self, on):
        """Set the firmware state to the real one without a press; returns (state, speed %)."""
        return self.impeller_answer(BOARD_IMPELLER_SYNC, bytes([1 if on else 0]), 'синхронизация')

    def impeller_mv(self, mv):
        """Signal voltage while on, mV (0 = speed control); returns (state, speed %, applied mV)."""
        data = self.request(BOARD_IMPELLER_SET_MV, struct.pack('<H', mv))
        if len(data) != 5:
            raise RuntimeError(f'длина ответа импеллера {len(data)} вместо 5')
        state, speed, applied = data[1], data[2], struct.unpack('<H', data[3:5])[0]
        if not data[0]:
            raise RuntimeError(f'напряжение отклонено (выше предела IMPELLER_MAX_SPEED_PCT или COMM_ENABLE = 0), '
                               f'осталось {applied} мВ')
        return state, speed, applied


class Teleop:
    def __init__(self, robot, args):
        self.robot = robot
        self.args = args
        self.linear = args.linear
        self.angular = args.angular
        self.armed = False
        self.impeller_speed = args.impeller_speed
        self.impeller_state = None
        self.impeller_current = None
        self.impeller_mv = None  # unknown until the first BOARD_IMPELLER_SET_MV answer
        self.held_key = None
        self.held_deadline = 0.0
        self.sent_speed = None
        self.sent_time = 0.0
        self.log = []

    def message(self, text):
        self.log = (self.log + [time.strftime('%H:%M:%S ') + text])[-8:]

    def call(self, title, action):
        try:
            result = action()
            if result is not None:
                self.message(f'{title}: {result}')
            return True
        except (OSError, RuntimeError, ValueError) as error:
            self.message(f'{title}: ОШИБКА {error}')
            return False

    def impeller(self, on):
        def action():
            self.impeller_state, self.impeller_current = self.robot.impeller(on, self.impeller_speed)
            return f'{IMPELLER_STATES.get(self.impeller_state, self.impeller_state)}, сейчас {self.impeller_current}%'
        self.call(f'импеллер вкл {self.impeller_speed}%' if on else 'импеллер выкл', action)

    def set_impeller_mv(self, mv):
        mv = min(max(mv, 0), IMPELLER_MV_MAX)
        def action():
            self.impeller_state, self.impeller_current, self.impeller_mv = self.robot.impeller_mv(mv)
            return f'{self.impeller_mv} мВ' + (' (управление скоростью)' if not self.impeller_mv else '')
        self.call(f'опорное {mv} мВ', action)

    def impeller_state_call(self, title, action):
        def run():
            self.impeller_state, self.impeller_current = action()
            return IMPELLER_STATES.get(self.impeller_state, self.impeller_state)
        self.call(title, run)

    def target_speed(self, now):
        if not self.armed or self.held_key is None or now > self.held_deadline:
            self.held_key = None
            return (0.0, 0.0, 0.0)
        dx, dy, dphi = MOTION_KEYS[self.held_key]
        return (dx * self.linear, dy * self.linear, dphi * self.angular)

    def update_speed(self, now):
        speed = self.target_speed(now)
        if speed == self.sent_speed and now - self.sent_time < self.args.resend:
            return
        # Motors keep the last setpoint, so zero is resent until acknowledged.
        if self.call('скорость', lambda: self.robot.set_speed(*speed)):
            if speed != self.sent_speed and speed == (0.0, 0.0, 0.0):
                self.message('стоп')
            self.sent_speed = speed
        self.sent_time = now

    def press_motion(self, key, now):
        if not self.armed:
            self.message('моторы выключены: нажмите M')
            return
        if self.args.latch:
            self.held_key, self.held_deadline = key, math.inf
        elif key == self.held_key and now <= self.held_deadline:
            self.held_deadline = now + self.args.repeat_timeout
        else:
            # First press waits for the OS key-repeat delay, later repeats keep it alive.
            self.held_key, self.held_deadline = key, now + self.args.first_timeout

    def handle(self, key, now):
        if key in MOTION_KEYS:
            self.press_motion(key, now)
            return True
        if key in (ord('x'), 27):
            return False
        char = chr(key).lower() if 0 <= key < 256 else ''
        if char == ' ':
            self.held_key = None
        elif char == 'm':
            self.armed = not self.armed
            self.held_key = None
            self.message('моторы ВКЛЮЧЕНЫ' if self.armed else 'моторы выключены')
        elif char in '+=':
            self.linear = min(self.linear + self.args.linear_step, self.args.max_linear)
        elif char in '-_':
            self.linear = max(self.linear - self.args.linear_step, self.args.linear_step)
        elif char == ']':
            self.angular = min(self.angular + self.args.angular_step, self.args.max_angular)
        elif char == '[':
            self.angular = max(self.angular - self.args.angular_step, self.args.angular_step)
        elif char == 't':
            self.impeller(True)
        elif char == 'y':
            self.impeller(False)
        elif char in ',<.>':
            step = IMPELLER_SPEED_STEP if char in '.>' else -IMPELLER_SPEED_STEP
            self.impeller_speed = min(max(self.impeller_speed + step, 0), 100)
            # Repeated BOARD_IMPELLER_ON on a running impeller only changes its speed.
            if self.impeller_state in (1, 2):
                self.impeller(True)
        elif char in 'jk':
            base = IMPELLER_MV_DEFAULT if self.impeller_mv is None else self.impeller_mv
            self.set_impeller_mv(base + (self.args.impeller_mv_step if char == 'k' else -self.args.impeller_mv_step))
        elif char == 'h':
            self.set_impeller_mv(0)
        elif char == 'p':
            ms = self.args.impeller_press or 200
            self.impeller_state_call(f'нажатие {ms} мс', lambda: self.robot.impeller_press(ms))
        elif char in 'un':
            on = char == 'u'
            self.impeller_state_call('считать ' + ('включённым' if on else 'выключенным'),
                                     lambda: self.robot.impeller_sync(on))
        elif char == 'i':
            self.call('пинг', lambda: f'{self.robot.ping():.1f} мс')
        elif char == 'b':
            self.call('батарея', lambda: '{} мВт, ячейки {} мВ, {} мВ, {} мА, {}%'.format(*self.robot.get_battery()))
        elif char == 'v':
            self.call('скорость', lambda: 'dx={:.3f} dy={:.3f} dphi={:.3f}'.format(*self.robot.get_speed()))
        elif char == 'c':
            self.call('координаты', lambda: 'x={:.3f} y={:.3f} phi={:.3f}'.format(*self.robot.get_coord()))
        elif char == 'r':
            self.call('сброс координат', lambda: self.robot.set_coord(0.0, 0.0, 0.0))
        return True

    def draw(self, screen):
        screen.erase()
        height, width = screen.getmaxyx()
        speed = self.sent_speed or (0.0, 0.0, 0.0)
        impeller = IMPELLER_STATES.get(self.impeller_state, '?')
        if self.impeller_mv is None:
            reference = f'? (после сброса {IMPELLER_MV_DEFAULT} мВ)'
        else:
            reference = f'{self.impeller_mv} мВ' if self.impeller_mv else '0 — управление скоростью'
        lines = [f'Eurobot teleop  {self.args.port}  ' + ('режим фиксации' if self.args.latch else 'режим удержания'), '']
        lines += HELP + ['']
        lines.append(f'Моторы: {"ВКЛЮЧЕНЫ" if self.armed else "выключены"}   '
                     f'Импеллер: {impeller}, задано {self.impeller_speed}%, опорное {reference}')
        lines.append(f'Лимиты: {self.linear:.2f} м/с, {self.angular:.2f} рад/с')
        lines.append(f'Отправлено: dx={speed[0]:+.2f} dy={speed[1]:+.2f} dphi={speed[2]:+.2f}')
        lines += [''] + self.log
        for row, text in enumerate(lines[:height]):
            screen.addnstr(row, 0, text, max(width - 1, 0))
        screen.refresh()

    def run(self, screen):
        curses.curs_set(0)
        screen.keypad(True)
        screen.timeout(20)
        self.call('пинг', lambda: f'{self.robot.ping():.1f} мс')
        try:
            while True:
                now = time.monotonic()
                key = screen.getch()
                while key != -1:
                    if not self.handle(key, now):
                        return
                    key = screen.getch()
                self.update_speed(time.monotonic())
                self.draw(screen)
        finally:
            self.stop()

    def stop(self):
        for title, action in (('нулевая скорость', lambda: self.robot.set_speed(0.0, 0.0, 0.0)),
                              ('выключение импеллера', lambda: self.robot.impeller(False))):
            for _ in range(5):
                try:
                    action()
                    break
                except (OSError, RuntimeError):
                    time.sleep(0.05)
            else:
                print(f'ВНИМАНИЕ: {title} не подтверждено, остановите робота вручную', file=sys.stderr)


def impeller_once(robot, args):
    """One impeller command without the keyboard UI, e.g. over ssh."""
    if args.impeller_mv is not None:
        # Sent before 'on': the firmware applies it before pressing the button.
        state, current, applied = robot.impeller_mv(args.impeller_mv)
        print(f'OK: опорное напряжение {applied} мВ' + (' (управление скоростью)' if not applied else '')
              + f'; состояние {IMPELLER_STATES.get(state, state)}, текущая скорость {current}%')
        time.sleep(IMPELLER_CMD_GAP)
    if args.impeller_sync:
        state, current = robot.impeller_sync(args.impeller_sync == 'on')
        print(f'OK: прошивка считает импеллер {IMPELLER_STATES.get(state, state)} (без нажатия)')
        time.sleep(IMPELLER_CMD_GAP)
    if args.impeller_press:
        robot.impeller_press(args.impeller_press)
        print(f'OK: нажатие кнопки {args.impeller_press} мс')
        time.sleep(args.impeller_press / 1000 + IMPELLER_CMD_GAP)
    if not args.impeller:
        return
    on = args.impeller == 'on'
    state, current = robot.impeller(on, args.impeller_speed)
    print(f'OK: импеллер {"вкл " + str(args.impeller_speed) + "%" if on else "выкл"}; '
          f'состояние {IMPELLER_STATES.get(state, state)}, текущая скорость {current}%')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list', action='store_true', help='показать последовательные порты')
    parser.add_argument('--port')
    parser.add_argument('--baudrate', type=int, default=921600)
    parser.add_argument('--timeout', type=float, default=0.3, help='ожидание ответа, с')
    parser.add_argument('--linear', type=float, default=0.2, help='начальная линейная скорость, м/с')
    parser.add_argument('--angular', type=float, default=1.0, help='начальная угловая скорость, рад/с')
    parser.add_argument('--linear-step', type=float, default=0.05)
    parser.add_argument('--angular-step', type=float, default=0.25)
    parser.add_argument('--max-linear', type=float, default=0.8)
    parser.add_argument('--max-angular', type=float, default=4.0)
    parser.add_argument('--first-timeout', type=float, default=0.6,
                        help='сколько ждать автоповтора клавиши после первого нажатия, с')
    parser.add_argument('--repeat-timeout', type=float, default=0.2,
                        help='остановка, если автоповтор не пришёл за это время, с')
    parser.add_argument('--resend', type=float, default=0.2, help='период повторной отправки скорости, с')
    parser.add_argument('--latch', action='store_true',
                        help='клавиша движения задаёт скорость до пробела (без удержания)')
    parser.add_argument('--impeller', choices=('on', 'off'),
                        help='отправить одну команду импеллеру и выйти, без клавиатурного управления')
    parser.add_argument('--impeller-mv', type=int,
                        help='опорное напряжение сигнала при включённом импеллере, мВ (0..3300, 0 = управление '
                             'скоростью); сохраняется до сброса платы, с --impeller on отправляется первым')
    parser.add_argument('--impeller-press', type=int, metavar='MS',
                        help='нажать кнопку импеллера на MS мс (1..2000), состояние в прошивке не меняется; '
                             'в клавиатурном режиме — длина нажатия для P (по умолчанию 200)')
    parser.add_argument('--impeller-sync', choices=('on', 'off'),
                        help='сказать прошивке реальное состояние импеллера, без нажатия кнопки')
    parser.add_argument('--impeller-mv-step', type=int, default=100,
                        help='шаг опорного напряжения для клавиш J / K, мВ')
    parser.add_argument('--impeller-speed', type=int, default=100,
                        help='скорость импеллера, %% (0..100, прошивка ограничивает IMPELLER_MAX_SPEED_PCT)')
    args = parser.parse_args()
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        raise RuntimeError('Требуется PySerial. Установите в отдельном venv: python -m pip install pyserial')
    if args.list:
        ports = list(list_ports.comports())
        for port in ports:
            print(port.device, port.description)
        if not ports:
            print('Последовательные порты не найдены.')
        return
    if not args.port:
        parser.error('укажите --port из списка --list')
    if not 0 <= args.impeller_speed <= 100:
        parser.error('--impeller-speed должна быть 0..100')
    if args.impeller_press is not None and not 0 < args.impeller_press <= IMPELLER_PRESS_MAX_MS:
        parser.error('--impeller-press должно быть 1..2000 мс')
    if not 0 < args.impeller_mv_step <= IMPELLER_MV_MAX:
        parser.error('--impeller-mv-step должен быть 1..3300')
    if args.impeller_mv is not None and not 0 <= args.impeller_mv <= IMPELLER_MV_MAX:
        parser.error('--impeller-mv должно быть 0..3300')
    if min(args.linear, args.angular, args.linear_step, args.angular_step, args.timeout) <= 0:
        parser.error('скорости, шаги и таймаут должны быть > 0')
    args.linear = min(args.linear, args.max_linear)
    args.angular = min(args.angular, args.max_angular)
    with serial.Serial(args.port, args.baudrate, bytesize=8, parity='N', stopbits=1,
                       timeout=0.01, write_timeout=1, xonxoff=False, rtscts=False, dsrdtr=False) as port:
        port.reset_input_buffer()
        if args.impeller or args.impeller_mv is not None or args.impeller_sync or args.impeller_press:
            impeller_once(Robot(port, args.timeout), args)
            return
        curses.wrapper(Teleop(Robot(port, args.timeout), args).run)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, RuntimeError) as error:
        print('FAIL: ' + str(error), file=sys.stderr)
        sys.exit(1)
