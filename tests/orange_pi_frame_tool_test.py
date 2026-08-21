"""Tests for the host-side Orange Pi frame construction tool."""

from __future__ import annotations

import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import orange_pi_frame_tool as protocol  # noqa: E402


class OrangePiFrameToolTest(unittest.TestCase):
    def test_standard_crc_vector(self) -> None:
        self.assertEqual(protocol.crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_task_round_trip(self) -> None:
        payload = protocol.parse_task_text("452+321+254+312")
        frame = protocol.build_frame(0x10, 7, 1234, payload)
        decoded = protocol.decode_frame(frame)

        self.assertEqual(decoded.version, 1)
        self.assertEqual(decoded.message_type, 0x10)
        self.assertEqual(decoded.sequence, 7)
        self.assertEqual(decoded.timestamp_ms, 1234)
        self.assertEqual(decoded.payload, bytes([4, 5, 2, 3, 2, 1, 2, 5, 4, 3, 1, 2]))

    def test_invalid_task_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            protocol.parse_task_text("452+321+257+312")

    def test_crc_error_is_rejected(self) -> None:
        frame = bytearray(protocol.build_frame(0x01, 1, 0))
        frame[-1] ^= 0x01
        with self.assertRaises(ValueError):
            protocol.decode_frame(bytes(frame))


if __name__ == "__main__":
    unittest.main()
