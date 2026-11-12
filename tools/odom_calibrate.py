#!/usr/bin/env python3
"""Step-by-step odometry calibration of the Maxon wheels over USART1 (terminal commands 0x0E-0x14)."""
import argparse
import json
import math
import re
import struct
import sys
import time
from pathlib import Path

from teleop import Robot

BOARD_SET_MAXONS_STATE = 0x0E
BOARD_GET_WHEEL_TICKS = 0x0F
BOARD_ROTATE_WHEEL = 0x10
BOARD_GET_WHEEL_ROTATION = 0x11
BOARD_GET_ODOMETRY_GEOMETRY = 0x12
BOARD_SET_ODOMETRY_CALIBRATION = 0x13
BOARD_GET_ODOMETRY_CALIBRATION = 0x14

# enum wheel_rotation_state_e in odometry.h
ROTATION_RUNNING, ROTATION_DONE = 1, 2
ROTATION_STATES = {0: 'ожидание', 1: 'вращается', 2: 'готово', 3: 'таймаут', 4: 'прервано'}

# order matches struct odometry_calibration_s
COEF_NAMES = ['WHEEL_1_ODOMETRY_COEF', 'WHEEL_2_ODOMETRY_COEF', 'WHEEL_3_ODOMETRY_COEF',
              'ODOMETRY_X_COEF', 'ODOMETRY_Y_COEF', 'ODOMETRY_PHI_COEF']
COEF_MIN, COEF_MAX = 0.5, 1.5  # CALIBRATION_COEF_MIN/MAX in kinematics.c
KINEMATICS_C = Path(__file__).resolve().parent.parent / 'Core' / 'App' / 'kinematics.c'
COS30 = math.sqrt(3) / 2


class StepCancelled(Exception):
    pass


class CalibrationRobot(Robot):
    def checked(self, command, payload, title):
        data = self.request(command, payload)
        if data != b'\x01':
            raise RuntimeError(f'{title}: плата отклонила команду (ответ {data.hex(" ") or "пустой"})')

    def set_maxons(self, enabled):
        self.checked(BOARD_SET_MAXONS_STATE, bytes([1 if enabled else 0]), 'моторы')

    def wheel_ticks(self):
        status, *ticks = struct.unpack('<Biii', self.request(BOARD_GET_WHEEL_TICKS))
        if not status:
            raise RuntimeError('не удалось прочитать энкодеры Maxon (ошибка шины)')
        return ticks

    def rotate_wheel(self, wheel, ticks, tpm):
        self.checked(BOARD_ROTATE_WHEEL, struct.pack('<Bii', wheel, ticks, tpm), 'вращение колеса')

    def wheel_rotation(self):
        return struct.unpack('<Bi', self.request(BOARD_GET_WHEEL_ROTATION))

    def geometry(self):
        return struct.unpack('<4f', self.request(BOARD_GET_ODOMETRY_GEOMETRY))

    def calibration(self):
        return list(struct.unpack('<6f', self.request(BOARD_GET_ODOMETRY_CALIBRATION)))

    def set_calibration(self, coefs):
        self.checked(BOARD_SET_ODOMETRY_CALIBRATION, struct.pack('<6f', *coefs), 'коэффициенты')


def ask(prompt):
    answer = input(prompt + ' ').strip()
    if answer.lower() == 'q':
        raise StepCancelled()
    return answer


def ask_float(prompt, default=None):
    while True:
        answer = ask(prompt + (f' [{default}]' if default is not None else '')).replace(',', '.')
        if not answer and default is not None:
            return default
        try:
            return float(answer)
        except ValueError:
            print('  нужно число (q — прервать шаг)')


def ask_yes(prompt):
    return ask(prompt + ' [y/N]').lower() in ('y', 'д', 'yes', 'да')


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def mean(values):
    return sum(values) / len(values)


def spread(values):
    if len(values) < 2:
        return 0.0
    return max(values) - min(values)


class Calibration:
    def __init__(self, robot, args):
        self.robot = robot
        self.args = args
        self.radius, self.center_to_wheel, self.gear_ratio, self.ticks_per_turn = robot.geometry()
        self.ticks_per_wheel_turn = self.gear_ratio * self.ticks_per_turn
        self.ticks_to_m = 2 * math.pi * self.radius / self.ticks_per_wheel_turn
        self.coefs = robot.calibration()
        self.log = []

    # ---------- kinematics, same formulas as Core/App/kinematics.c ----------

    def wheel_paths(self, d_ticks, coefs=None):
        wheel_coefs = coefs[:3] if coefs else [1.0, 1.0, 1.0]
        return [t * self.ticks_to_m * k for t, k in zip(d_ticks, wheel_coefs)]

    def body_from_wheels(self, s):
        dx = (s[0] - s[1]) / math.sqrt(3)
        dy = (s[0] + s[1] - 2 * s[2]) / 3
        dphi = (s[0] + s[1] + s[2]) / (3 * self.center_to_wheel)
        return dx, dy, dphi

    @staticmethod
    def wheels_from_body(dx, dy):
        return [COS30 * dx + 0.5 * dy, -COS30 * dx + 0.5 * dy, -dy]

    def angle_from_ticks(self, d_ticks):
        # Heading is linear in wheel paths, so it does not wrap like the firmware phi
        return self.body_from_wheels(self.wheel_paths(d_ticks, self.coefs))[2] * self.coefs[5]

    # ---------- helpers ----------

    def record(self, step, **data):
        self.log.append({'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'step': step, **data})
        if self.args.log:
            Path(self.args.log).write_text(json.dumps(self.log, ensure_ascii=False, indent=2))

    def refresh(self):
        self.coefs = self.robot.calibration()

    def show(self):
        print('\nГеометрия в прошивке: r = {:.4f} м, L = {:.4f} м, редуктор {:g}, энкодер {:g} тиков'.format(
            self.radius, self.center_to_wheel, self.gear_ratio, self.ticks_per_turn))
        print(f'Тиков на оборот колеса: {self.ticks_per_wheel_turn:.0f}, 1 тик = {self.ticks_to_m * 1e6:.3f} мкм')
        print('Коэффициенты в RAM платы:')
        for name, value in zip(COEF_NAMES, self.coefs):
            print(f'  {name:<24} {value:.4f}')

    def upload(self, coefs, title):
        for name, value in zip(COEF_NAMES, coefs):
            if not COEF_MIN <= value <= COEF_MAX:
                raise RuntimeError(f'{name} = {value:.4f} вне диапазона {COEF_MIN}..{COEF_MAX}: '
                                   'измерение явно ошибочно, повторите шаг')
        print(f'\n{title}:')
        for name, old, new in zip(COEF_NAMES, self.coefs, coefs):
            mark = '' if abs(old - new) < 5e-5 else f'  ({(new / old - 1) * 100:+.2f}%)'
            print(f'  {name:<24} {old:.4f} -> {new:.4f}{mark}')
        if not ask_yes('Загрузить в плату (до перезагрузки)?'):
            print('Не загружено.')
            return False
        self.robot.set_calibration(coefs)
        self.refresh()
        print('Загружено. Чтобы сохранить навсегда, используйте пункт w в меню и перепрошейте плату.')
        return True

    def stop(self):
        for _ in range(5):
            try:
                self.robot.set_speed(0.0, 0.0, 0.0)
                return
            except (OSError, RuntimeError):
                time.sleep(0.05)
        print('ВНИМАНИЕ: остановка не подтверждена, выключите моторы кнопкой!', file=sys.stderr)

    def ramp(self, current, remaining, limit, accel, minimum, dt):
        # Trapezoid: accelerate from the previous value, brake so that speed reaches zero at the target
        return min(limit, current + accel * dt, max(minimum, math.sqrt(2 * accel * max(remaining, 0.0))))

    def drive_to(self, tx, ty, timeout=None):
        """Closed loop on firmware odometry to (tx, ty) in the odometry frame, heading held at 0."""
        args = self.args
        x, y, _ = self.robot.get_coord()
        timeout = timeout or math.hypot(tx - x, ty - y) / args.speed * 3 + 5
        begin = last = time.monotonic()
        v = 0.0
        try:
            while True:
                x, y, phi = self.robot.get_coord()
                now = time.monotonic()
                dt, last = now - last, now
                ex, ey = tx - x, ty - y
                dist = math.hypot(ex, ey)
                if dist < args.tolerance:
                    break
                if now - begin > timeout:
                    raise RuntimeError('не доехал за отведённое время')
                v = self.ramp(v, dist, args.speed, args.accel, 0.03, dt)
                vx, vy = v * ex / dist, v * ey / dist
                c, s = math.cos(phi), math.sin(phi)
                dphi = max(-0.5, min(0.5, -2.0 * wrap(phi)))
                self.robot.set_speed(c * vx + s * vy, -s * vx + c * vy, dphi)
        finally:
            self.stop()
        time.sleep(0.5)
        return self.robot.get_coord()

    def rotate_by(self, angle):
        """Rotate in place until the encoder heading changes by angle (rad, signed)."""
        args = self.args
        start = self.robot.wheel_ticks()
        timeout = abs(angle) / args.angular * 3 + 5
        begin = last = time.monotonic()
        w = 0.0
        try:
            while True:
                ticks = self.robot.wheel_ticks()
                now = time.monotonic()
                dt, last = now - last, now
                done = self.angle_from_ticks([a - b for a, b in zip(ticks, start)])
                remaining = abs(angle) - done * math.copysign(1, angle)
                if remaining < math.radians(0.2):
                    break
                if now - begin > timeout:
                    raise RuntimeError('не довернулся за отведённое время')
                w = self.ramp(w, remaining, args.angular, args.angular_accel, 0.1, dt)
                self.robot.set_speed(0.0, 0.0, math.copysign(w, angle))
        finally:
            self.stop()
        time.sleep(0.5)
        ticks = self.robot.wheel_ticks()
        return self.angle_from_ticks([a - b for a, b in zip(ticks, start)])

    # ---------- steps ----------

    def step_ticks_per_turn(self):
        print('''
ШАГ 1. Тики на оборот колеса.
Проверяет GEAR_RATIO и ENCODER_TICKS_PER_TURN: колесо должно сделать ровно заданное число оборотов.
  • Поднимите робот, чтобы колёса не касались пола.
  • Поставьте маркером метку на каждое колесо и запомните её положение.
Небольшой перебег метки — это выбег мотора. Шаг ищет грубую ошибку (например, редуктор 21 вместо 28).''')
        turns = self.args.wheel_turns
        target = round(turns * self.ticks_per_wheel_turn)
        tpm = round(self.args.wheel_rpm * self.ticks_per_wheel_turn)
        self.robot.set_maxons(True)
        ratios = []
        for wheel in (1, 2, 3):
            if ask(f'\nКолесо {wheel}: Enter — повернуть на {turns:g} об., s — пропустить:').lower() == 's':
                continue
            self.robot.rotate_wheel(wheel, target, tpm)
            deadline = time.monotonic() + turns / self.args.wheel_rpm * 60 + 25
            while True:
                state, ticks = self.robot.wheel_rotation()
                print(f'\r  {ROTATION_STATES.get(state, state)}: {ticks} тиков', end='', flush=True)
                if state != ROTATION_RUNNING:
                    break
                if time.monotonic() > deadline:
                    self.stop()
                    raise RuntimeError('плата не завершила вращение')
                time.sleep(0.05)
            print()
            if state != ROTATION_DONE:
                print(f'  ВНИМАНИЕ: вращение завершилось со статусом «{ROTATION_STATES.get(state, state)}»')
                continue
            if ticks < 0:
                print('  ВНИМАНИЕ: энкодер считает в обратную сторону относительно команды скорости!')
            print(f'  Задано {target} тиков, с выбегом {abs(ticks)} (+{abs(ticks) - target})')
            real_turns = ask_float('  Сколько оборотов реально сделало колесо?', default=turns)
            if real_turns <= 0:
                print('  пропущено')
                continue
            measured = abs(ticks) / real_turns
            ratio = measured / self.ticks_per_wheel_turn
            ratios.append(ratio)
            print(f'  Тиков на оборот: {measured:.0f} (в прошивке {self.ticks_per_wheel_turn:.0f}, x{ratio:.3f})')
            self.record('ticks_per_turn', wheel=wheel, target=target, ticks=ticks, real_turns=real_turns)
        if not ratios:
            return
        ratio = mean(ratios)
        if abs(ratio - 1) > 0.05:
            print(f'\nНЕСООТВЕТСТВИЕ x{ratio:.3f}. Исправьте константы в kinematics.c и перепрошейте, '
                  'остальные шаги до этого бессмысленны.')
            print(f'  Если энкодер {self.ticks_per_turn:g} тиков верный: GEAR_RATIO ≈ {self.gear_ratio * ratio:.2f}')
            print(f'  Если редуктор {self.gear_ratio:g} верный: ENCODER_TICKS_PER_TURN ≈ {self.ticks_per_turn * ratio:.0f}')
        else:
            print(f'\nТики на оборот совпадают с прошивкой (x{ratio:.3f}). Можно переходить к шагу 2.')

    def step_wheels(self):
        print('''
ШАГ 2. Коэффициенты колёс (разброс диаметров).
Моторы будут ОТКЛЮЧЕНЫ, робот толкаете руками вдоль ровной направляющей (стена, рейка) на 1–2 м.
Ось X — вперёд, ось Y — влево. Робот не должен поворачиваться. Смещение меряйте рулеткой.
Толчок по X калибрует колёса 1 и 2, толчок по Y — все три (колесо 3 сильнее всего).''')
        self.robot.set_maxons(False)
        plan = [('X', 1, 'ВПЕРЁД'), ('X', -1, 'НАЗАД'), ('Y', 1, 'ВЛЕВО'), ('Y', -1, 'ВПРАВО')] * self.args.repeats
        pairs = [[], [], []]
        for axis, sign, title in plan:
            answer = ask(f'\nПрогон {axis}{"+" if sign > 0 else "-"}: поставьте робот у направляющей, '
                         'отметьте положение. Enter — начать, s — пропустить, f — закончить прогоны:').lower()
            if answer == 's':
                continue
            if answer == 'f':
                break
            start = self.robot.wheel_ticks()
            ask(f'  Толкните робот {title} вдоль направляющей и нажмите Enter:')
            end = self.robot.wheel_ticks()
            distance = ask_float('  Фактическое смещение по рулетке, мм:') / 1000
            if distance <= 0.2:
                print('  слишком короткий прогон (< 200 мм), пропущено')
                continue
            s = self.wheel_paths([b - a for a, b in zip(start, end)])
            dx, dy, dphi = self.body_from_wheels(s)
            along, across = (dx, dy) if axis == 'X' else (dy, dx)
            # Uncalibrated heading is unreliable: 1% wheel mismatch over 1.5 m already reads as several degrees
            print(f'  Одометрия без коэффициентов: вдоль {along * 1000:+.1f} мм, поперёк {across * 1000:+.1f} мм, '
                  f'поворот {math.degrees(dphi):+.2f}°')
            if along * sign <= 0:
                print('  ОШИБКА: по одометрии робот поехал не туда. Проверьте порядок и направление моторов '
                      '(MAXON_MOTOR_1..3 в bus_device.c) и знаки в kinematics.c. Прогон отброшен.')
                continue
            if abs(across) > 0.15 * abs(along):
                print('  ВНИМАНИЕ: большое поперечное смещение — возможно, перепутаны колёса.')
            expected = self.wheels_from_body(sign * distance, 0.0) if axis == 'X' else \
                self.wheels_from_body(0.0, sign * distance)
            for wheel in range(3):
                if abs(expected[wheel]) > 0.3 * distance:
                    pairs[wheel].append((expected[wheel], s[wheel]))
                    print(f'  колесо {wheel + 1}: должно {expected[wheel] * 1000:+.1f} мм, '
                          f'по тикам {s[wheel] * 1000:+.1f} мм, k = {expected[wheel] / s[wheel]:.4f}')
            self.record('wheels', axis=axis, sign=sign, distance_m=distance, ticks_start=start, ticks_end=end)
        new = list(self.coefs)
        for wheel in range(3):
            if not pairs[wheel]:
                print(f'Колесо {wheel + 1}: нет данных, коэффициент не меняется')
                continue
            # Least squares fit of expected = k * measured over all runs
            new[wheel] = sum(e * m for e, m in pairs[wheel]) / sum(m * m for _, m in pairs[wheel])
            ratios = [e / m for e, m in pairs[wheel]]
            print(f'Колесо {wheel + 1}: k = {new[wheel]:.4f} по {len(ratios)} прогонам, разброс {spread(ratios) * 100:.2f}%')
        if new[:3] == self.coefs[:3]:
            return
        # Body coefs were fitted on top of the old wheel coefs, so they start over
        new[3:] = [1.0, 1.0, 1.0]
        if self.upload(new, 'Новые коэффициенты колёс (X/Y/угол сброшены в 1.0 — пройдите шаги 3 и 4)'):
            self.robot.set_maxons(True)

    def step_linear(self):
        distance = self.args.distance
        print(f'''
ШАГ 3. Масштаб по X и Y.
Робот САМ проедет {distance:g} м по одометрии вперёд, назад, влево и вправо со скоростью {self.args.speed:g} м/с.
Освободите место. Каждый раз меряйте рулеткой фактическое смещение вдоль оси движения.
Ctrl+C — аварийная остановка.''')
        self.robot.set_maxons(True)
        plan = [('X', 1, 'вперёд'), ('X', -1, 'назад'), ('Y', 1, 'влево'), ('Y', -1, 'вправо')] * self.args.repeats
        ratios = {'X': [], 'Y': []}
        for axis, sign, title in plan:
            answer = ask(f'\nПрогон {title}: отметьте положение робота. Enter — поехали, s — пропустить, '
                         'f — закончить:').lower()
            if answer == 's':
                continue
            if answer == 'f':
                break
            self.robot.set_coord(0.0, 0.0, 0.0)
            target = sign * distance
            x, y, phi = self.drive_to(target, 0.0) if axis == 'X' else self.drive_to(0.0, target)
            odom = abs(x if axis == 'X' else y)
            print(f'  Одометрия: x = {x * 1000:+.1f} мм, y = {y * 1000:+.1f} мм, phi = {math.degrees(phi):+.2f}°')
            real = ask_float('  Фактическое смещение вдоль оси по рулетке, мм:') / 1000
            if real <= 0:
                print('  пропущено')
                continue
            ratios[axis].append(real / odom)
            print(f'  real/odom = {real / odom:.4f}')
            self.record('linear', axis=axis, sign=sign, odom_m=[x, y, phi], real_m=real, coefs=self.coefs)
        new = list(self.coefs)
        for axis, index in (('X', 3), ('Y', 4)):
            if ratios[axis]:
                new[index] = self.coefs[index] * mean(ratios[axis])
                print(f'{axis}: поправка x{mean(ratios[axis]):.4f}, разброс {spread(ratios[axis]) * 100:.2f}%')
        if new != self.coefs:
            self.upload(new, 'Новые коэффициенты X/Y')

    def step_angular(self):
        turns = self.args.rotation_turns
        print(f'''
ШАГ 4. Масштаб угла.
Робот САМ повернётся на месте на {turns:g} об. против часовой, затем по часовой.
Совместите край или метку робота с линией на полу (или с лучом лазерного уровня).
Ctrl+C — аварийная остановка.''')
        self.robot.set_maxons(True)
        ratios = []
        for direction, title in [(1, 'против часовой'), (-1, 'по часовой')] * self.args.repeats:
            answer = ask(f'\nПоворот {title}: совместите метку с линией. Enter — поехали, s — пропустить, '
                         'f — закончить:').lower()
            if answer == 's':
                continue
            if answer == 'f':
                break
            odom = abs(self.rotate_by(direction * turns * 2 * math.pi))
            print(f'  Одометрия: {math.degrees(odom):.1f}°')
            residual = ask_float('  На сколько градусов робот ушёл ДАЛЬШЕ линии по ходу вращения '
                                 '(не довернул — с минусом)?', default=0.0)
            real = math.radians(turns * 360 + residual)
            ratios.append(real / odom)
            print(f'  real/odom = {real / odom:.4f}')
            self.record('angular', direction=direction, odom_rad=odom, real_rad=real, coefs=self.coefs)
        if ratios:
            new = list(self.coefs)
            new[5] = self.coefs[5] * mean(ratios)
            print(f'Угол: поправка x{mean(ratios):.4f}, разброс {spread(ratios) * 100:.2f}%')
            self.upload(new, 'Новый коэффициент угла')

    def step_square(self):
        side = self.args.square
        print(f'''
ШАГ 5. Проверка: квадрат {side:g} x {side:g} м (вперёд, влево, назад, вправо) с возвратом в старт.
Отметьте положение и ориентацию робота. Ctrl+C — аварийная остановка.''')
        self.robot.set_maxons(True)
        ask('Enter — поехали:')
        self.robot.set_coord(0.0, 0.0, 0.0)
        for x, y in [(side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]:
            self.drive_to(x, y)
        x, y, phi = self.robot.get_coord()
        print(f'Одометрия в конце: x = {x * 1000:+.1f} мм, y = {y * 1000:+.1f} мм, phi = {math.degrees(phi):+.2f}°')
        error = ask_float('Расстояние от стартовой метки по рулетке, мм:')
        print(f'Ошибка замыкания: {error:.1f} мм на {4 * side:g} м пути ({error / (40 * side):.2f}%)')
        self.record('square', side_m=side, error_mm=error, coefs=self.coefs)

    # ---------- results ----------

    def defines(self):
        return [f'#define {name:<24} {value:.4f}f' for name, value in zip(COEF_NAMES, self.coefs)]

    def write_source(self):
        path = Path(self.args.source)
        text = path.read_text()
        for name, value in zip(COEF_NAMES, self.coefs):
            pattern = re.compile(rf'^(#define\s+{name}\s+)[-+0-9.eE]+f?', re.M)
            if not pattern.search(text):
                raise RuntimeError(f'в {path} нет #define {name}')
            text = pattern.sub(lambda m: f'{m.group(1)}{value:.4f}f', text)
        print('\n'.join(self.defines()))
        if ask_yes(f'Записать эти значения в {path}?'):
            path.write_text(text)
            print('Записано. Соберите и прошейте: make flash')

    def reset(self):
        if self.upload([1.0] * 6, 'Сброс коэффициентов'):
            print('Коэффициенты в RAM сброшены.')

    def run(self):
        actions = {'1': self.step_ticks_per_turn, '2': self.step_wheels, '3': self.step_linear,
                   '4': self.step_angular, '5': self.step_square, 'p': lambda: print('\n'.join(self.defines())),
                   'w': self.write_source, 'r': self.refresh, '0': self.reset}
        self.show()
        while True:
            print('''
  1  Тики на оборот колеса (робот поднят)
  2  Коэффициенты колёс (толкать руками)
  3  Масштаб X/Y (робот едет сам)
  4  Масштаб угла (робот крутится сам)
  5  Проверка квадратом
  p  Показать #define      w  Записать в kinematics.c
  r  Перечитать с платы    0  Сбросить в 1.0
  q  Выход''')
            choice = input('> ').strip().lower()
            if choice == 'q':
                return
            if choice not in actions:
                continue
            try:
                self.refresh()
                actions[choice]()
            except StepCancelled:
                print('Шаг прерван.')
            except KeyboardInterrupt:
                self.stop()
                print('\nОстановлено (Ctrl+C).')
            except (OSError, RuntimeError) as error:
                self.stop()
                print(f'ОШИБКА: {error}')
            if choice in '01234':
                self.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list', action='store_true', help='показать последовательные порты')
    parser.add_argument('--port')
    parser.add_argument('--baudrate', type=int, default=921600)
    parser.add_argument('--timeout', type=float, default=0.3, help='ожидание ответа, с')
    parser.add_argument('--repeats', type=int, default=1, help='сколько раз повторить каждый прогон')
    parser.add_argument('--wheel-turns', type=float, default=1.0, help='шаг 1: оборотов колеса')
    parser.add_argument('--wheel-rpm', type=float, default=10.0, help='шаг 1: скорость колеса, об/мин')
    parser.add_argument('--distance', type=float, default=1.0, help='шаг 3: длина прогона, м')
    parser.add_argument('--speed', type=float, default=0.15, help='шаги 3, 5: скорость, м/с')
    parser.add_argument('--accel', type=float, default=0.2, help='шаги 3, 5: ускорение, м/с²')
    parser.add_argument('--tolerance', type=float, default=0.002, help='шаги 3, 5: точность остановки, м')
    parser.add_argument('--rotation-turns', type=float, default=3.0, help='шаг 4: оборотов робота')
    parser.add_argument('--angular', type=float, default=1.0, help='шаг 4: угловая скорость, рад/с')
    parser.add_argument('--angular-accel', type=float, default=1.0, help='шаг 4: угловое ускорение, рад/с²')
    parser.add_argument('--square', type=float, default=0.5, help='шаг 5: сторона квадрата, м')
    parser.add_argument('--source', default=str(KINEMATICS_C), help='файл с #define коэффициентов')
    parser.add_argument('--log', help='JSON-файл для сохранения всех измерений')
    args = parser.parse_args()
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        raise RuntimeError('Требуется PySerial. Установите в отдельном venv: python -m pip install pyserial')
    if args.list:
        for port in list_ports.comports():
            print(port.device, port.description)
        return
    if not args.port:
        parser.error('укажите --port из списка --list')
    if min(args.repeats, args.wheel_turns, args.wheel_rpm, args.distance, args.speed, args.accel,
           args.tolerance, args.rotation_turns, args.angular, args.angular_accel, args.square) <= 0:
        parser.error('все параметры должны быть > 0')
    with serial.Serial(args.port, args.baudrate, bytesize=8, parity='N', stopbits=1,
                       timeout=0.01, write_timeout=1, xonxoff=False, rtscts=False, dsrdtr=False) as port:
        port.reset_input_buffer()
        robot = CalibrationRobot(port, args.timeout)
        try:
            calibration = Calibration(robot, args)
        except RuntimeError as error:
            raise RuntimeError(f'{error}. Возможно, прошивка без команд калибровки 0x0E–0x14')
        try:
            calibration.run()
        finally:
            calibration.stop()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, RuntimeError) as error:
        print('FAIL: ' + str(error), file=sys.stderr)
        sys.exit(1)
