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
BOARD_TOGGLE_GRIPPER = 0x0A
BOARD_POWER_ON_PUMP = 0x0B
BOARD_POWER_OFF_PUMP = 0x0C
BOARD_IMPELLER_ON = 0x0D
BOARD_IMPELLER_OFF = 0x0E
BOARD_IMPELLER_GET_STATE = 0x0F
BOARD_IMPELLER_SET_DUTY = 0x10
BOARD_TOGGLE_GRAIL_PICK_GRIPPER = 0x11
BOARD_TOGGLE_GRAIL_SHELF_GRIPPER = 0x12
BOARD_GRAIL_LIFT_UP = 0x13
BOARD_GRAIL_LIFT_DOWN = 0x14
BOARD_GRIPPER_VERTICAL = 0x15
BOARD_TOGGLE_CURSOR = 0x16
BOARD_GRIPPER_SLIDE = 0x17
BOARD_GET_IMU = 0x18

# The firmware answers the grail lift command after the move, it gives up after 10 s.
GRAIL_LIFT_TIMEOUT = 12.0
# The firmware answers after the move: vertical gives up after 4 s, cursor and rack after 1.5 s.
GRIPPER_TIMEOUT = 6.0
GRIPPER_MAX_MM = 255
# Pinion range 0..300 deg from 148 deg at 3.979 deg/mm
RACK_MIN_MM, RACK_MAX_MM = -37, 38

# enum impeller_state_e in impeller.h
IMPELLER_STATES = {0: 'выкл', 1: 'включается', 2: 'ВКЛ', 3: 'выключается'}
IMPELLER_DUTY_STEP = 5
IMPELLER_DEFAULT_DUTY = 75  # IMPELLER_DEFAULT_DUTY_PCT, used if the board does not answer

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
    'P / O      насосы вкл / выкл       G / F     захват: цикл / закрыть (0)',
    'I          пинг                    B         батарея',
    'V          скорость с энкодеров    C / R     координаты / сброс в 0',
    'U / J      гриппер вверх / вниз    H         курсор горизонт. / вертик.',
    'K / L      гриппер по рейке - / +  5         IMU: ускорения, скорости, температура',
    'T / Y      импеллер вкл / выкл     < / >     скважность импеллера -/+ 5 %',
    '/          состояние импеллера',
    '1 / 2      захват / полка грааля   3 / 4      лифт грааля вверх / вниз',
    'X, Esc     выход (нулевая скорость; импеллер выключается, если включён)',
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

    def gripper(self, state):
        self.request(BOARD_TOGGLE_GRIPPER, bytes([state]))

    def pump(self, on):
        self.request(BOARD_POWER_ON_PUMP if on else BOARD_POWER_OFF_PUMP)

    def gripper_move(self, command, payload, title):
        timeout, self.timeout = self.timeout, GRIPPER_TIMEOUT
        try:
            data = self.request(command, payload)
        finally:
            self.timeout = timeout
        if data != b'\x01':
            raise RuntimeError(f'{title}: не дошёл (таймаут, ошибка шины, занят или едет другой шаговик)')
        return 'ок'

    def get_imu(self):
        data = self.request(BOARD_GET_IMU)
        if len(data) != 29:
            raise RuntimeError(f'длина данных IMU {len(data)} вместо 29')
        if not data[0]:
            raise RuntimeError('датчик не отвечает')
        ax, ay, az, gx, gy, gz, temp = struct.unpack('<7f', data[1:])
        return (f'a=({ax:+.2f} {ay:+.2f} {az:+.2f}) g  '
                f'w=({gx:+.1f} {gy:+.1f} {gz:+.1f}) °/с  {temp:.1f} °C')

    def grail_toggle(self, command):
        data = self.request(command)
        if data != b'\x01':
            raise RuntimeError(f'ответ {data.hex()} вместо 01')
        return 'ок'

    def grail_lift(self, up, step_mm):
        timeout, self.timeout = self.timeout, GRAIL_LIFT_TIMEOUT
        try:
            data = self.request(BOARD_GRAIL_LIFT_UP if up else BOARD_GRAIL_LIFT_DOWN, bytes([step_mm]))
        finally:
            self.timeout = timeout
        if data != b'\x01':
            raise RuntimeError('не дошёл: таймаут, идёт калибровка, лифт занят или едет гриппер')
        return 'ок'

    def impeller_answer(self, command, payload, title):
        """Returns (state, duty on the pin %, set duty %); the answer comes before the press."""
        data = self.request(command, payload)
        if len(data) != 4:
            raise RuntimeError(f'длина ответа импеллера {len(data)} вместо 4')
        if not data[0]:
            raise RuntimeError(f'{title} отклонено (неверный параметр)')
        return data[1], data[2], data[3]

    def impeller(self, on, duty=None):
        """duty None: turn on with the duty already set in the firmware."""
        if on:
            return self.impeller_answer(BOARD_IMPELLER_ON, b'' if duty is None else bytes([duty]), 'включение')
        return self.impeller_answer(BOARD_IMPELLER_OFF, b'', 'выключение')

    def impeller_status(self):
        return self.impeller_answer(BOARD_IMPELLER_GET_STATE, b'', 'чтение')

    def impeller_set_duty(self, duty):
        """Duty only, the impeller is not turned on or off."""
        return self.impeller_answer(BOARD_IMPELLER_SET_DUTY, bytes([duty]), 'скважность')



class Teleop:
    def __init__(self, robot, args):
        self.robot = robot
        self.args = args
        self.linear = args.linear
        self.angular = args.angular
        self.armed = False
        self.pump = None
        self.gripper_height = 0  # mm, the firmware counts from the power-on position
        self.cursor_horizontal = False  # vertical after start
        self.rack_mm = 0
        self.impeller_duty = args.impeller_duty  # None: read from the board at start
        self.impeller_state = None
        self.impeller_current = None
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
            self.impeller_state, self.impeller_current, self.impeller_duty = self.robot.impeller(on, self.impeller_duty)
            return f'{IMPELLER_STATES.get(self.impeller_state, self.impeller_state)}, на выходе {self.impeller_current}%'
        self.call(f'импеллер вкл {self.impeller_duty}%' if on else 'импеллер выкл', action)

    def read_impeller(self):
        def action():
            self.impeller_state, self.impeller_current, duty = self.robot.impeller_status()
            if self.impeller_duty is None:
                self.impeller_duty = duty
            return (f'{IMPELLER_STATES.get(self.impeller_state, self.impeller_state)}, на выходе {self.impeller_current}%, '
                    f'задано в прошивке {duty}%')
        self.call('импеллер', action)

    def set_impeller_duty(self):
        def action():
            self.impeller_state, self.impeller_current, self.impeller_duty = self.robot.impeller_set_duty(self.impeller_duty)
            return f'{IMPELLER_STATES.get(self.impeller_state, self.impeller_state)}, на выходе {self.impeller_current}%'
        self.call(f'скважность импеллера {self.impeller_duty}%', action)

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
        elif char == 'p':
            if self.call('насосы вкл', lambda: self.robot.pump(True)):
                self.pump = True
        elif char == 'o':
            if self.call('насосы выкл', lambda: self.robot.pump(False)):
                self.pump = False
        elif char == 'g':
            self.call('захват цикл', lambda: self.robot.gripper(1))
        elif char == 'f':
            self.call('захват 0', lambda: self.robot.gripper(0))
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
        elif char == 't':
            self.impeller(True)
        elif char == 'y':
            self.impeller(False)
        elif char in ',<.>':
            step = IMPELLER_DUTY_STEP if char in '.>' else -IMPELLER_DUTY_STEP
            self.impeller_duty = min(max(self.impeller_duty + step, 0), 100)
            self.set_impeller_duty()
        elif char in '/?':
            self.read_impeller()
        elif char == '5':
            self.call('IMU', self.robot.get_imu)
        elif char == '1':
            self.call('захват грааля', lambda: self.robot.grail_toggle(BOARD_TOGGLE_GRAIL_PICK_GRIPPER))
        elif char == '2':
            self.call('полка грааля', lambda: self.robot.grail_toggle(BOARD_TOGGLE_GRAIL_SHELF_GRIPPER))
        elif char in '34':
            up = char == '3'
            self.call(f'лифт грааля {"вверх" if up else "вниз"} {self.args.grail_step} мм',
                      lambda: self.robot.grail_lift(up, self.args.grail_step))
        elif char in 'uj':
            height = min(max(self.gripper_height + (self.args.gripper_step if char == 'u' else -self.args.gripper_step), 0), GRIPPER_MAX_MM)
            if self.call(f'гриппер на {height} мм',
                         lambda: self.robot.gripper_move(BOARD_GRIPPER_VERTICAL, bytes([height]), 'гриппер')):
                self.gripper_height = height
        elif char == 'h':
            horizontal = not self.cursor_horizontal
            if self.call('курсор ' + ('горизонтально' if horizontal else 'вертикально'),
                         lambda: self.robot.gripper_move(BOARD_TOGGLE_CURSOR, bytes([1 if horizontal else 0]), 'курсор')):
                self.cursor_horizontal = horizontal
        elif char in 'kl':
            rack = min(max(self.rack_mm + (self.args.rack_step if char == 'l' else -self.args.rack_step), RACK_MIN_MM), RACK_MAX_MM)
            if self.call(f'рейка {rack:+d} мм',
                         lambda: self.robot.gripper_move(BOARD_GRIPPER_SLIDE, struct.pack('<i', rack), 'рейка')):
                self.rack_mm = rack
        return True

    def draw(self, screen):
        screen.erase()
        height, width = screen.getmaxyx()
        speed = self.sent_speed or (0.0, 0.0, 0.0)
        pump = {None: '?', True: 'ВКЛ', False: 'выкл'}[self.pump]
        impeller = IMPELLER_STATES.get(self.impeller_state, 'нет ответа')
        lines = [f'Eurobot teleop  {self.args.port}  ' + ('режим фиксации' if self.args.latch else 'режим удержания'), '']
        lines += HELP + ['']
        lines.append(f'Моторы: {"ВКЛЮЧЕНЫ" if self.armed else "выключены"}   Насосы: {pump}')
        lines.append(f'Гриппер: {self.gripper_height} мм, курсор {"горизонт." if self.cursor_horizontal else "вертик."}, рейка {self.rack_mm:+d} мм')
        lines.append(f'Импеллер: {impeller}, скважность {self.impeller_duty}% (≈ {self.impeller_duty * 3.3 / 100:.2f} В)')
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
        self.read_impeller()
        if self.impeller_duty is None:
            self.impeller_duty = IMPELLER_DEFAULT_DUTY
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
        actions = [('нулевая скорость', lambda: self.robot.set_speed(0.0, 0.0, 0.0))]
        if self.impeller_state in (1, 2):
            actions.append(('выключение импеллера', lambda: self.robot.impeller(False)))
        for title, action in actions:
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
    if args.impeller == 'status':
        state, current, duty = robot.impeller_status()
        print(f'OK: состояние {IMPELLER_STATES.get(state, state)}, на выходе {current}%, задано в прошивке {duty}%')
        return
    if args.impeller is None:
        state, current, duty = robot.impeller_set_duty(args.impeller_duty)
        print(f'OK: скважность {duty}%; состояние {IMPELLER_STATES.get(state, state)}, на выходе {current}%')
        return
    on = args.impeller == 'on'
    state, current, duty = robot.impeller(on, args.impeller_duty)
    print(f'OK: импеллер {"вкл " + str(duty) + "%" if on else "выкл"}; '
          f'состояние {IMPELLER_STATES.get(state, state)}, на выходе {current}%')


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
    parser.add_argument('--gripper-step', type=int, default=10, help='ход гриппера на клавишу U/J, мм (1..255)')
    parser.add_argument('--rack-step', type=int, default=5, help='ход по рейке на клавишу K/L, мм')
    parser.add_argument('--grail-step', type=int, default=10, help='ход лифта грааля на клавишу 3/4, мм (1..255)')
    parser.add_argument('--impeller', choices=('on', 'off', 'status'),
                        help='отправить одну команду импеллеру и выйти, без клавиатурного управления')
    parser.add_argument('--impeller-duty', '--impeller-speed', dest='impeller_duty', type=int,
                        help='скважность ШИМ импеллера, %% (0..100): на фильтре 3,3 В × %%; '
                             'по умолчанию — заданная в прошивке (после сброса 75); '
                             'без --impeller — только задать скважность, не включая и не выключая')
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
    if args.impeller_duty is not None and not 0 <= args.impeller_duty <= 100:
        parser.error('--impeller-duty должна быть 0..100')
    if not 0 < args.gripper_step <= 255 or args.rack_step <= 0:
        parser.error('--gripper-step должен быть 1..255 мм, --rack-step > 0')
    if not 0 < args.grail_step <= 255:
        parser.error('--grail-step должен быть 1..255 мм')
    if min(args.linear, args.angular, args.linear_step, args.angular_step, args.timeout) <= 0:
        parser.error('скорости, шаги и таймаут должны быть > 0')
    args.linear = min(args.linear, args.max_linear)
    args.angular = min(args.angular, args.max_angular)
    with serial.Serial(args.port, args.baudrate, bytesize=8, parity='N', stopbits=1,
                       timeout=0.01, write_timeout=1, xonxoff=False, rtscts=False, dsrdtr=False) as port:
        port.reset_input_buffer()
        if args.impeller or args.impeller_duty is not None:
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
