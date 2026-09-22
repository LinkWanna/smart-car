#!/usr/bin/env python3
"""Conformance tests for ``tools/src/protocol.py`` (the Python mirror).

    python3 tools/tests/test_protocol.py

Little-endian fields, exact payload lengths and the frame parser are pinned
here; the Rust side pins the same bytes in ``protocol/src/tests.rs``.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

# The shared library modules live in tools/src/, next to this test.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import protocol as p

ALL_REQUESTS = [
    p.Init(),
    p.SetSpeed(p.MotorId.LEFT, 50),
    p.SetSpeed(p.MotorId.RIGHT, -100),
    p.Stop(p.MotorTarget.BOTH),
    p.Brake(p.MotorTarget.LEFT),
    p.SetSpeeds(40, -40),
    p.SetPid(p.MotorId.RIGHT, 150, 20, -5),
    p.GetPid(p.MotorId.LEFT),
    p.Move(p.MoveDir.FORWARD, 50, 500),
    p.Rotate(p.RotateDir.LEFT, 40, 900),
    p.Heartbeat(),
    p.Reset(),
]

ALL_RESPONSES = [
    p.Ack(0x01),
    p.Nack(0x10, p.ErrorCode.INVALID_PARAM),
    p.Status(p.SysState.RUNNING, (120, -120), True, 0),
    p.PidData(150, 20, 0),
]


class Requests(unittest.TestCase):
    def test_all_roundtrip(self) -> None:
        for request in ALL_REQUESTS:
            frame = p.encode_request(request)
            self.assertEqual(p.decode_request(frame[2], frame[4:-1]), request)

    def test_exact_length(self) -> None:
        with self.assertRaises(p.ProtocolError) as ctx:
            p.decode_request(p.RequestType.INIT, b"\x00")
        self.assertEqual(ctx.exception.error_code, p.ErrorCode.INVALID_PARAM)
        with self.assertRaises(p.ProtocolError):
            p.decode_request(p.RequestType.SET_SPEED, b"\x00\x00")

    def test_unknown_command(self) -> None:
        with self.assertRaises(p.ProtocolError) as ctx:
            p.decode_request(0x42, b"")
        self.assertEqual(ctx.exception.error_code, p.ErrorCode.UNKNOWN_REQUEST)

    def test_invalid_values(self) -> None:
        with self.assertRaises(p.ProtocolError):
            p.decode_request(p.RequestType.STOP, b"\x03")  # target > 2
        with self.assertRaises(p.ProtocolError):
            p.decode_request(
                p.RequestType.MOVE, bytes([0x00, 0x00, 0x00, 0x00, 0x01, 0xF4])
            )  # speed = 0
        with self.assertRaises(p.ProtocolError):
            p.decode_request(
                p.RequestType.MOVE, bytes([0x00, 0x65, 0x00, 0x00, 0x01, 0xF4])
            )  # speed > 100
        with self.assertRaises(p.ProtocolError):
            p.decode_request(
                p.RequestType.MOVE, bytes([0x00, 0x32, 0x00, 0x00, 0x00, 0x00])
            )  # target = 0
        with self.assertRaises(p.ProtocolError):
            p.encode_request(p.Move(p.MoveDir.FORWARD, 0, 500))
        with self.assertRaises(p.ProtocolError):
            p.encode_request(p.Rotate(p.RotateDir.LEFT, 40, 0))

    def test_length_table(self) -> None:
        for request in ALL_REQUESTS:
            frame = p.encode_request(request)
            self.assertEqual(frame[3], p.RequestType(frame[2]).payload_len)
            self.assertEqual(
                len(frame), p.RequestType(frame[2]).payload_len + p.FRAME_OVERHEAD
            )

    def test_little_endian_is_pinned(self) -> None:
        """Hardcoded bytes: a roundtrip cannot hide an endian swap."""
        self.assertEqual(
            p.encode_request(p.SetSpeed(p.MotorId.RIGHT, -100)),
            bytes([0xAA, 0x55, 0x10, 0x03, 0x01, 0x9C, 0xFF, 0x71]),
        )
        self.assertEqual(
            p.encode_request(p.Move(p.MoveDir.FORWARD, 50, 500)),
            bytes([0xAA, 0x55, 0x20, 0x06, 0x00, 0x32, 0xF4, 0x01, 0x00, 0x00, 0xE1]),
        )
        self.assertEqual(
            p.encode_response(p.Status(p.SysState.RUNNING, (120, -120), True, 0)),
            bytes(
                [0xAA, 0x55, 0x91, 0x07, 0x02, 0x78, 0x00, 0x88, 0xFF, 0x01, 0x00, 0x9A]
            ),
        )


class Responses(unittest.TestCase):
    def test_all_roundtrip(self) -> None:
        for response in ALL_RESPONSES:
            frame = p.encode_response(response)
            self.assertEqual(p.decode_response(frame[2], frame[4:-1]), response)

    def test_status_done(self) -> None:
        reached = p.Status(p.SysState.RUNNING, (0, 0), False, 1)
        self.assertTrue(reached.done)
        self.assertFalse(p.Status(p.SysState.RUNNING, (0, 0), True, 0).done)

    def test_status_is_strict(self) -> None:
        with self.assertRaises(p.ProtocolError):
            p.Status.parse(bytes([p.SysState.READY, 0, 0, 0, 0]))  # v1 payload
        with self.assertRaises(p.ProtocolError):
            p.decode_response(p.ResponseType.NACK, b"\x10")  # truncated
        with self.assertRaises(p.ProtocolError):
            p.decode_response(0x90, b"")  # unknown response


class ParserBehaviour(unittest.TestCase):
    def test_resync_after_garbage(self) -> None:
        stream = bytes([0x00, 0xAA, 0xAA, 0x55, 0x01, 0x00, 0x01, 0xFF, 0xFF])
        stream += bytes([0xAA, 0x55, 0xFE, 0x00, 0xFE])
        events = p.FrameParser().feed(stream)
        self.assertEqual(events, [p.Frame(0x01, b""), p.Frame(0xFE, b"")])

    def test_too_long_resyncs_immediately(self) -> None:
        stream = bytes([0xAA, 0x55, 0x10, 0x11, 0xAA, 0x55, 0xFE, 0x00, 0xFE])
        events = p.FrameParser().feed(stream)
        self.assertEqual(events, [p.TooLong(0x10, 17), p.Frame(0xFE, b"")])


if __name__ == "__main__":
    unittest.main()
