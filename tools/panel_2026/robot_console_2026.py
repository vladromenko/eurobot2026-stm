#!/usr/bin/env python3
"""Entry point for the modular Eurobot 2026 SSH keyboard control panel."""

from __future__ import annotations

import argparse
import curses
import sys
import time
from datetime import datetime
from pathlib import Path

from robot_panel.command_codes import BAUD_RATE
from robot_panel.controller import FirmwareLinkError, Robot2026
from robot_panel.demo import DemoPort
from robot_panel.logs import TestResultLog, TraceLog
from robot_panel.protocol import (
    ProtocolError,
    list_serial_ports,
    require_pyserial,
    resolve_port,
)
from robot_panel.selftest import self_test
from robot_panel.ui import Teleop

def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Control Eurobot 2026 from an SSH terminal (no Docker/ROS required)."
    )
    parser.add_argument("--port", help="Serial port, for example /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    parser.add_argument("--timeout", type=float, default=7.0)
    parser.add_argument("--linear-speed", type=float, default=0.12, metavar="M/S")
    parser.add_argument("--angular-speed", type=float, default=0.50, metavar="RAD/S")
    parser.add_argument("--drive-timeout", type=float, default=0.30, metavar="SEC")
    parser.add_argument("--vertical-step", type=int, default=5, metavar="MM")
    parser.add_argument("--slide-step", type=int, default=5, metavar="MM")
    parser.add_argument(
        "--cursor-jog-step",
        type=int,
        default=5,
        metavar="DEG",
        help=(
            "Small jog of the U/J cursor motor used by keys 7/8 "
            "(default: 5 degrees)"
        ),
    )
    parser.add_argument(
        "--vertical-start",
        type=int,
        default=0,
        choices=range(256),
        metavar="0..255",
        help="Current/last commanded vertical position (default: 0 after fresh boot)",
    )
    parser.add_argument(
        "--slide-start",
        type=int,
        default=0,
        choices=range(-37, 38),
        metavar="-37..37",
        help="Current/last commanded slide position (default: 0 after fresh boot)",
    )
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Check PING, GET_SPEED and PWM8 status, then exit without actuator commands",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Open the full interface with a simulated controller; no hardware access",
    )
    parser.add_argument(
        "--log-file",
        type=Path,
        help="JSONL transaction log (default: timestamped file beside this script)",
    )
    parser.add_argument(
        "--test-results-file",
        type=Path,
        help="JSONL guided-test results (default: timestamped file beside this script)",
    )
    parser.add_argument(
        "--confirm-2026",
        action="store_true",
        help="Required confirmation that STM32 is running the 2026 firmware",
    )
    return parser


def main() -> int:
    args = make_parser().parse_args()
    if args.self_test:
        return self_test()
    if args.list_ports:
        return list_serial_ports()
    if not args.demo and not args.confirm_2026:
        raise SystemExit(
            "Refusing to actuate hardware without --confirm-2026. "
            "This program is not compatible with firmware 2025."
        )
    if not (0.02 <= args.linear_speed <= 0.60):
        raise SystemExit("--linear-speed must be in range 0.02..0.60 m/s")
    if not (0.10 <= args.angular_speed <= 3.00):
        raise SystemExit("--angular-speed must be in range 0.10..3.00 rad/s")
    if not (0.10 <= args.drive_timeout <= 1.00):
        raise SystemExit("--drive-timeout must be in range 0.10..1.00 s")
    if not (1 <= args.vertical_step <= 50 and 1 <= args.slide_step <= 20):
        raise SystemExit("Invalid mechanism step; see --help")
    if not 1 <= args.cursor_jog_step <= 20:
        raise SystemExit("--cursor-jog-step must be in range 1..20 degrees")

    if args.log_file is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.log_file = (
            Path(__file__).resolve().parent
            / f"robot_2026_console_log_{timestamp}.jsonl"
        )
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.test_results_file is None:
        args.test_results_file = (
            Path(__file__).resolve().parent
            / f"robot_2026_test_results_{timestamp}.jsonl"
        )

    if args.demo:
        device = "DEMO"
        port_context = DemoPort(args.timeout)
        print("Opening safe demo mode (no hardware access)...")
    else:
        serial, _ = require_pyserial()
        device = resolve_port(args.port)
        print(f"Opening {device} at {args.baud} baud...")
        try:
            port_context = serial.Serial(
                device,
                args.baud,
                timeout=args.timeout,
                write_timeout=args.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
                exclusive=True,
            )
        except (OSError, serial.SerialException) as exc:
            print(f"ERROR: cannot open {device}: {exc}", file=sys.stderr)
            return 2

    robot = None
    trace = TraceLog(args.log_file)
    test_results = TestResultLog(args.test_results_file)
    try:
        with port_context as port:
            robot = Robot2026(port, trace)
            if not args.demo:
                print("Проверяю связь с прошивкой 2026 (PING + GET_SPEED)...")
                time.sleep(0.30)
                port.reset_input_buffer()
                robot.verify_firmware(attempts=3, timeout=1.0)
                print("Связь подтверждена: STM32 отвечает по протоколу прошивки 2026.")
            initial_pwm = robot.impeller_status()
            print(f"PWM8: {initial_pwm}")
            if args.check_only:
                print("CHECK OK: управляющие команды не отправлялись.")
                return 0
            # The first actuator writes after a proven link always make the base safe.
            robot.set_speed(0.0, 0.0, 0.0, action="Startup / stop base")
            robot.set_motors(False, action="Startup / disable motors")
            try:
                def run_panel(screen) -> None:
                    Teleop(screen, robot, test_results, device, args, initial_pwm).run()

                curses.wrapper(run_panel)
            finally:
                try:
                    robot.impeller_stop()
                except Exception as cleanup_error:
                    print(f"PWM8 stop unconfirmed: {cleanup_error}", file=sys.stderr)
                # Do not release suction automatically: it could drop a held object.
                # Always stop and disable the base on any exit path.
                try:
                    robot.set_speed(
                        0.0, 0.0, 0.0, action="Shutdown / stop base"
                    )
                    robot.set_motors(False, action="Shutdown / disable motors")
                except Exception as cleanup_error:
                    print(f"Safety cleanup warning: {cleanup_error}", file=sys.stderr)
    except KeyboardInterrupt:
        print("Stopped by user; base stop was requested.")
        return 130
    except (OSError, TimeoutError, ProtocolError, FirmwareLinkError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    finally:
        trace.close()
        test_results.close()

    print(f"Closed safely. Transaction log: {args.log_file}")
    print(f"Guided-test results: {args.test_results_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
