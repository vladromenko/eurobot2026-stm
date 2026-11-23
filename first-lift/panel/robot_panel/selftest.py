"""Offline checks for framing, CRC, and known 2026 packets."""

import struct

from .command_codes import (
    CMD_GET_AX_ANGLE,
    CMD_PING,
    CMD_PUMPS,
    CMD_SET_AX_ANGLE,
    CMD_SET_SPEED,
    CMD_VALVES,
    CMD_LIFT2_ARM,
    CMD_LIFT2_MOVE_REL,
    CMD_LIFT2_SET_SERVO,
)
from .protocol import checksum, make_packet


def self_test() -> int:
    vectors = (
        (CMD_PING, b"", "ff ff 02 00 fd"),
        (CMD_GET_AX_ANGLE, bytes((106,)), "ff ff 03 0a 6a 88"),
        (
            CMD_SET_AX_ANGLE,
            bytes((106,)) + struct.pack("<HH", 123, 150),
            "ff ff 07 0b 6a 7b 00 96 00 72",
        ),
        (CMD_VALVES, bytes((1, 15)), "ff ff 04 22 01 0f c9"),
        (CMD_PUMPS, bytes((1, 15)), "ff ff 04 21 01 0f ca"),
        (CMD_LIFT2_ARM, b"", "ff ff 02 30 cd"),
        (CMD_LIFT2_MOVE_REL, struct.pack("<i", 25), "ff ff 06 31 19 00 00 00 af"),
        (CMD_LIFT2_SET_SERVO, bytes((1,)) + struct.pack("<H", 1500), "ff ff 05 34 01 dc 05 e4"),
        (CMD_SET_SPEED, struct.pack("<fff", 0.0, 0.0, 0.0), None),
    )
    for command, payload, expected in vectors:
        packet = make_packet(command, payload)
        if packet[-1] != checksum(packet[2], packet[3], packet[4:-1]):
            raise AssertionError("Packet CRC self-test failed")
        if expected is not None and packet.hex(" ") != expected:
            raise AssertionError(f"Packet mismatch: {packet.hex(' ')}")
    print("Self-test OK: packet framing, CRC and known 2026 vectors are valid.")
    return 0
