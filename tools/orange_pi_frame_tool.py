"""Build and inspect the first-version Orange Pi serial protocol frames.

This tool only prints bytes. It does not open a serial port or move the robot.
"""

from __future__ import annotations

import argparse
import struct
from dataclasses import dataclass


SOF = b"\xA5\x5A"
PROTOCOL_VERSION = 1
PAYLOAD_MAX_LENGTH = 32

MESSAGE_TYPES = {
    "heartbeat": 0x01,
    "task": 0x10,
    "start": 0x11,
    "stop": 0x12,
}


@dataclass(frozen=True)
class DecodedFrame:
    version: int
    message_type: int
    sequence: int
    timestamp_ms: int
    payload: bytes


def crc16_ccitt_false(data: bytes) -> int:
    """Return CRC16-CCITT-FALSE (init 0xFFFF, poly 0x1021)."""

    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_task_text(text: str) -> bytes:
    """Convert 452+321+254+312 into twelve color IDs."""

    groups = text.split("+")
    if len(groups) != 4 or any(len(group) != 3 for group in groups):
        raise ValueError("task code must contain four groups of three digits")

    payload = bytearray()
    for group in groups:
        for character in group:
            if character not in "123456":
                raise ValueError("every task color must be in the range 1..6")
            payload.append(int(character))
    return bytes(payload)


def build_frame(
    message_type: int,
    sequence: int,
    timestamp_ms: int,
    payload: bytes = b"",
) -> bytes:
    """Build one frame matching template/Hardware/orange_pi_link.c."""

    if not 0 <= message_type <= 0xFF:
        raise ValueError("message type must fit in one byte")
    if not 0 <= sequence <= 0xFFFF:
        raise ValueError("sequence must fit in uint16")
    if not 0 <= timestamp_ms <= 0xFFFFFFFF:
        raise ValueError("timestamp must fit in uint32")
    if len(payload) > PAYLOAD_MAX_LENGTH:
        raise ValueError("payload exceeds protocol limit")

    protected = struct.pack(
        "<BBHHI",
        PROTOCOL_VERSION,
        message_type,
        sequence,
        len(payload),
        timestamp_ms,
    ) + payload
    crc = crc16_ccitt_false(protected)
    return SOF + protected + struct.pack("<H", crc)


def decode_frame(frame: bytes) -> DecodedFrame:
    """Validate and decode one complete frame."""

    if len(frame) < 14 or frame[:2] != SOF:
        raise ValueError("invalid frame header or length")

    version, message_type, sequence, payload_length, timestamp_ms = struct.unpack(
        "<BBHHI", frame[2:12]
    )
    expected_length = 14 + payload_length
    if payload_length > PAYLOAD_MAX_LENGTH or len(frame) != expected_length:
        raise ValueError("invalid payload length")

    received_crc = struct.unpack("<H", frame[-2:])[0]
    calculated_crc = crc16_ccitt_false(frame[2:-2])
    if received_crc != calculated_crc:
        raise ValueError("CRC mismatch")
    if version != PROTOCOL_VERSION:
        raise ValueError("unsupported protocol version")

    return DecodedFrame(
        version=version,
        message_type=message_type,
        sequence=sequence,
        timestamp_ms=timestamp_ms,
        payload=frame[12:-2],
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("message", choices=MESSAGE_TYPES)
    parser.add_argument("task_code", nargs="?", help="for example 452+321+254+312")
    parser.add_argument("--seq", type=int, default=1)
    parser.add_argument("--timestamp", type=int, default=0)
    args = parser.parse_args()

    payload = b""
    if args.message == "task":
        if args.task_code is None:
            parser.error("task message requires a task code")
        payload = parse_task_text(args.task_code)
    elif args.task_code is not None:
        parser.error("only task messages accept a task code")

    frame = build_frame(
        MESSAGE_TYPES[args.message],
        args.seq,
        args.timestamp,
        payload,
    )
    print(" ".join(f"{value:02X}" for value in frame))


if __name__ == "__main__":
    main()
