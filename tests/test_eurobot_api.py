import struct
import unittest

from tools.eurobot_api import (
    Command,
    EurobotAPI,
    Pose,
    ProtocolError,
    Velocity,
    decode_frame,
    encode_frame,
)


class FakeStream:
    def __init__(self, responses=b""):
        self.responses = bytearray(responses)
        self.written = bytearray()

    def read(self, size=1):
        chunk = self.responses[:size]
        del self.responses[:size]
        return bytes(chunk)

    def write(self, data):
        self.written.extend(data)
        return len(data)

    def flush(self):
        pass


class FrameTests(unittest.TestCase):
    def test_ping_frame_matches_firmware_format(self):
        self.assertEqual(encode_frame(Command.PING), bytes.fromhex("ffff0200fd"))

    def test_round_trip_with_payload(self):
        frame = encode_frame(Command.SET_SPEED, struct.pack("<fff", 0.2, -0.1, 1.0))
        command, payload = decode_frame(frame)
        self.assertEqual(command, Command.SET_SPEED)
        for actual, expected in zip(struct.unpack("<fff", payload), (0.2, -0.1, 1.0)):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_bad_checksum_is_rejected(self):
        frame = bytearray(encode_frame(Command.PING))
        frame[-1] ^= 1
        with self.assertRaises(ProtocolError):
            decode_frame(bytes(frame))


class APITests(unittest.TestCase):
    def test_get_pose_ignores_noise_before_header(self):
        response = b"noise" + encode_frame(Command.GET_COORD, struct.pack("<fff", 1.0, 2.0, 3.0))
        api = EurobotAPI(FakeStream(response), timeout=0.1)
        self.assertEqual(api.get_pose(), Pose(1.0, 2.0, 3.0))

    def test_set_velocity_encodes_little_endian_floats(self):
        stream = FakeStream(encode_frame(Command.SET_SPEED))
        api = EurobotAPI(stream, timeout=0.1)
        api.set_velocity(Velocity(0.25, -0.5, 1.5))
        command, payload = decode_frame(bytes(stream.written))
        self.assertEqual(command, Command.SET_SPEED)
        self.assertEqual(struct.unpack("<fff", payload), (0.25, -0.5, 1.5))

    def test_response_length_is_checked(self):
        api = EurobotAPI(FakeStream(encode_frame(Command.GET_SPEED, b"bad")), timeout=0.1)
        with self.assertRaises(ProtocolError):
            api.get_velocity()


if __name__ == "__main__":
    unittest.main()
