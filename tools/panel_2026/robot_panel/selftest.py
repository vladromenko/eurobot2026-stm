"""Offline checks for framing, CRC, and known 2026 packets."""

import struct

from .command_codes import (
    CMD_GET_AX_ANGLE,
    CMD_PING,
    CMD_IMPELLER_START, CMD_IMPELLER_STOP, CMD_IMPELLER_STATUS, CMD_IMPELLER_DUTY,
    CMD_PUMPS,
    CMD_SET_AX_ANGLE,
    CMD_SET_SPEED,
    CMD_VALVES,
)
from .protocol import checksum, make_packet


def self_test() -> int:
    vectors = (
        (CMD_PING, b"", "ff ff 02 00 fd"),
        (CMD_IMPELLER_START, bytes((50,)), "ff ff 03 30 32 9a"),
        (CMD_IMPELLER_STOP, b"", "ff ff 02 31 cc"),
        (CMD_IMPELLER_STATUS, b"", "ff ff 02 32 cb"),
        (CMD_IMPELLER_DUTY, bytes((50,)), "ff ff 03 33 32 97"),
        (CMD_GET_AX_ANGLE, bytes((106,)), "ff ff 03 0a 6a 88"),
        (
            CMD_SET_AX_ANGLE,
            bytes((106,)) + struct.pack("<HH", 123, 150),
            "ff ff 07 0b 6a 7b 00 96 00 72",
        ),
        (CMD_VALVES, bytes((1, 15)), "ff ff 04 22 01 0f c9"),
        (CMD_PUMPS, bytes((1, 15)), "ff ff 04 21 01 0f ca"),
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
