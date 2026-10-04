import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from radar_direct_protocol import (  # noqa: E402
    AlarmFrame,
    Lds50cParser,
    MeasurementFrame,
    RawMeasurement,
    ScanAssembler,
    StatusFrame,
    normalize_measurement,
)


def measurement_frame(header, start_angle, measurements, sector_angle=None):
    point_count = len(measurements)
    payload = bytearray(header)
    payload.extend(point_count.to_bytes(2, "little"))
    payload.extend(start_angle.to_bytes(2, "little"))
    checksum = point_count + start_angle
    if sector_angle is not None:
        payload.extend(sector_angle.to_bytes(2, "little"))
        checksum += sector_angle
    for energy, distance in measurements:
        payload.append(energy)
        payload.extend(distance.to_bytes(2, "little"))
        checksum += energy + distance
    payload.extend((checksum & 0xFFFF).to_bytes(2, "little"))
    return bytes(payload)


def status_frame(flags=0x0F):
    return b"ST" + bytes((flags, 0x00, 0x00, 0x00)) + b"ED"


def alarm_frame(code):
    return b"\xCE\xCE\xCE\xCE" + code.to_bytes(2, "little")


def decoded_frame(start, count, *, fixed=True, sector=0, distance_base=100):
    measurements = tuple(
        RawMeasurement(energy=index + 1, distance_mm=distance_base + index)
        for index in range(count)
    )
    return MeasurementFrame(
        fixed_resolution=fixed,
        point_count=count,
        start_angle_tenths=start,
        sector_angle_tenths=sector if fixed else None,
        measurements=measurements,
    )


class Lds50cParserTests(unittest.TestCase):
    def test_accepts_all_reference_frame_types_across_arbitrary_chunks(self):
        normal = measurement_frame(
            b"\xCE\xFA",
            123,
            ((7, 456), (8, 789)),
        )
        fixed = measurement_frame(
            b"\xCF\xFA",
            900,
            ((10, 1000), (20, 2000), (30, 3000)),
            sector_angle=100,
        )
        stream = b"\x00\x99" + normal + fixed + status_frame(0x0F) + alarm_frame(0x1234)
        parser = Lds50cParser()

        frames = []
        chunk_sizes = (1, 2, 5, 3, 8, 1, 13)
        offset = 0
        index = 0
        while offset < len(stream):
            size = chunk_sizes[index % len(chunk_sizes)]
            frames.extend(parser.feed(stream[offset:offset + size]))
            offset += size
            index += 1

        self.assertEqual(
            [type(frame) for frame in frames],
            [MeasurementFrame, MeasurementFrame, StatusFrame, AlarmFrame],
        )
        self.assertEqual(frames[0].start_angle_tenths, 123)
        self.assertFalse(frames[0].fixed_resolution)
        self.assertEqual(
            frames[0].measurements,
            (RawMeasurement(energy=7, distance_mm=456), RawMeasurement(energy=8, distance_mm=789)),
        )
        self.assertTrue(frames[1].fixed_resolution)
        self.assertEqual(frames[1].sector_angle_tenths, 100)
        self.assertTrue(frames[2].uses_millimetres)
        self.assertTrue(frames[2].energy_enabled)
        self.assertTrue(frames[2].trailing_removal_enabled)
        self.assertTrue(frames[2].filter_enabled)
        self.assertEqual(frames[3].code, 0x1234)
        self.assertEqual(parser.valid_frames, 4)
        self.assertEqual(parser.discarded_bytes, 2)

    def test_preserves_partial_header_prefix_between_feed_calls(self):
        parser = Lds50cParser()

        self.assertEqual(parser.feed(b"noise\xCE"), [])
        frames = parser.feed(b"\xFA\x01\x00\x00\x00\x02\x64\x00\x67\x00")

        self.assertEqual(len(frames), 1)
        self.assertIsInstance(frames[0], MeasurementFrame)
        self.assertEqual(frames[0].measurements[0].distance_mm, 100)

    def test_bounds_bad_length_and_resynchronizes_after_bad_checksum(self):
        oversized_header = b"\xCE\xFA" + (4097).to_bytes(2, "little") + b"\x00\x00"
        damaged = bytearray(
            measurement_frame(b"\xCE\xFA", 100, ((1, 200), (2, 300)))
        )
        damaged[-1] ^= 0xFF
        parser = Lds50cParser()

        frames = parser.feed(oversized_header + bytes(damaged) + status_frame(0x01))

        self.assertEqual(frames, [StatusFrame(flags=0x01)])
        self.assertEqual(parser.length_errors, 1)
        self.assertEqual(parser.checksum_errors, 1)
        self.assertGreater(parser.discarded_bytes, 0)
        self.assertEqual(parser.valid_frames, 1)


class MeasurementNormalizationTests(unittest.TestCase):
    def test_millimetre_frame_is_preserved(self):
        frame = decoded_frame(123, 2, sector=456, distance_base=1000)

        normalized = normalize_measurement(frame, uses_millimetres=True)

        self.assertIs(normalized, frame)

    def test_centimetres_normalize_to_mm_and_overflow_becomes_zero(self):
        frame = MeasurementFrame(
            fixed_resolution=True,
            point_count=3,
            start_angle_tenths=123,
            sector_angle_tenths=456,
            measurements=(
                RawMeasurement(energy=31, distance_mm=100),
                RawMeasurement(energy=47, distance_mm=6553),
                RawMeasurement(energy=52, distance_mm=7000),
            ),
        )

        normalized = normalize_measurement(frame, uses_millimetres=False)

        self.assertEqual(
            normalized.measurements,
            (
                RawMeasurement(energy=31, distance_mm=1000),
                RawMeasurement(energy=47, distance_mm=65530),
                RawMeasurement(energy=52, distance_mm=0),
            ),
        )
        self.assertEqual(normalized.start_angle_tenths, 123)
        self.assertEqual(normalized.sector_angle_tenths, 456)


class ScanAssemblerTests(unittest.TestCase):
    @staticmethod
    def arm(assembler):
        assembler.push(decoded_frame(3500, 0))
        assembler.push(decoded_frame(0, 0))

    def test_fixed_resolution_uses_inclusive_sector_endpoints(self):
        assembler = ScanAssembler(minimum_points_per_scan=1)
        self.arm(assembler)

        self.assertIsNone(assembler.push(decoded_frame(100, 3, sector=100)))
        completed = assembler.push(decoded_frame(0, 0))

        self.assertEqual([point.angle_deg for point in completed], [10.0, 15.0, 20.0])

    def test_normal_frame_waits_for_next_start_and_divides_delta_by_count(self):
        assembler = ScanAssembler(minimum_points_per_scan=1)
        self.arm(assembler)

        self.assertIsNone(assembler.push(decoded_frame(100, 2, fixed=False)))
        self.assertIsNone(assembler.push(decoded_frame(200, 1, fixed=False)))
        completed = assembler.push(decoded_frame(0, 0, fixed=False))

        self.assertEqual([point.angle_deg for point in completed], [10.0, 15.0, 20.0])

    def test_first_wrap_arms_and_only_a_later_complete_revolution_is_published(self):
        assembler = ScanAssembler(minimum_points_per_scan=30)
        completed = []

        for frame in (
            decoded_frame(3500, 10, sector=40),
            decoded_frame(0, 10, sector=40),
            decoded_frame(1200, 10, sector=40),
            decoded_frame(2400, 10, sector=40),
            decoded_frame(3500, 10, sector=40),
            decoded_frame(0, 10, sector=40),
        ):
            scan = assembler.push(frame)
            if scan is not None:
                completed.append(scan)

        self.assertEqual(len(completed), 1)
        self.assertEqual(len(completed[0]), 40)
        self.assertEqual(completed[0][0].angle_deg, 0.0)
        self.assertEqual(completed[0][30].angle_deg, 350.0)
        self.assertTrue(all(0.0 <= point.angle_deg < 360.0 for point in completed[0]))

    def test_minimum_point_count_and_reset_prevent_partial_publication(self):
        assembler = ScanAssembler(minimum_points_per_scan=30)
        self.arm(assembler)
        assembler.push(decoded_frame(100, 2, sector=50))
        self.assertIsNone(assembler.push(decoded_frame(0, 0)))

        assembler.push(decoded_frame(1800, 30, sector=100))
        assembler.reset()
        self.assertIsNone(assembler.push(decoded_frame(0, 0)))
        self.assertIsNone(assembler.push(decoded_frame(3500, 30, sector=100)))
        self.assertIsNone(assembler.push(decoded_frame(0, 0)))

    def test_declared_point_count_must_match_measurements(self):
        assembler = ScanAssembler()
        mismatched = MeasurementFrame(
            fixed_resolution=True,
            point_count=2,
            start_angle_tenths=0,
            sector_angle_tenths=10,
            measurements=(RawMeasurement(energy=1, distance_mm=100),),
        )

        with self.assertRaises(ValueError):
            assembler.push(mismatched)


if __name__ == "__main__":
    unittest.main()
