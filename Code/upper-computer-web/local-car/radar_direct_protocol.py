"""LDS50C direct-serial frame parsing and scan assembly primitives."""

from __future__ import annotations

from dataclasses import dataclass, replace


@dataclass(frozen=True)
class RawMeasurement:
    energy: int
    distance_mm: int


@dataclass(frozen=True)
class MeasurementFrame:
    fixed_resolution: bool
    point_count: int
    start_angle_tenths: int
    sector_angle_tenths: int | None
    measurements: tuple[RawMeasurement, ...]


@dataclass(frozen=True)
class StatusFrame:
    flags: int
    reserved_bytes: tuple[int, int, int] = (0, 0, 0)

    @property
    def uses_millimetres(self) -> bool:
        return bool(self.flags & 0b0001)

    @property
    def energy_enabled(self) -> bool:
        return bool(self.flags & 0b0010)

    @property
    def trailing_removal_enabled(self) -> bool:
        return bool(self.flags & 0b0100)

    @property
    def filter_enabled(self) -> bool:
        return bool(self.flags & 0b1000)


@dataclass(frozen=True)
class AlarmFrame:
    code: int


@dataclass(frozen=True)
class RadarPoint:
    angle_deg: float
    distance_mm: int
    energy: int


def normalize_measurement(
    frame: MeasurementFrame,
    uses_millimetres: bool,
) -> MeasurementFrame:
    """Return a millimetre-valued frame using the reference overflow rule."""
    if uses_millimetres:
        return frame
    measurements = tuple(
        RawMeasurement(
            energy=measurement.energy,
            distance_mm=(measurement.distance_mm * 10)
            if measurement.distance_mm * 10 <= 0xFFFF
            else 0,
        )
        for measurement in frame.measurements
    )
    return replace(frame, measurements=measurements)


class ScanAssembler:
    """Publish only complete revolutions after observing the first wrap."""

    _FULL_CIRCLE_TENTHS = 3600

    def __init__(self, minimum_points_per_scan: int = 30):
        if minimum_points_per_scan < 1:
            raise ValueError("minimum_points_per_scan must be positive")
        self._minimum_points_per_scan = minimum_points_per_scan
        self._current_points: list[RadarPoint] = []
        self._pending_normal_frame: MeasurementFrame | None = None
        self._last_start_angle_tenths: int | None = None
        self._is_armed = False

    def push(self, frame: MeasurementFrame) -> tuple[RadarPoint, ...] | None:
        if frame.point_count != len(frame.measurements):
            raise ValueError("declared point count must match the measurements")

        start_angle_tenths = self._normalize_tenths(frame.start_angle_tenths)
        wrapped = (
            self._last_start_angle_tenths is not None
            and start_angle_tenths < self._last_start_angle_tenths
        )

        if self._pending_normal_frame is not None and self._is_armed:
            self._current_points.extend(
                self._interpolate_normal(self._pending_normal_frame, start_angle_tenths)
            )

        completed_scan = None
        if wrapped:
            if self._is_armed and len(self._current_points) >= self._minimum_points_per_scan:
                completed_scan = tuple(self._current_points)
            self._current_points.clear()
            if not self._is_armed:
                self._is_armed = True

        if frame.fixed_resolution:
            self._pending_normal_frame = None
            if self._is_armed:
                self._current_points.extend(self._interpolate_fixed(frame))
        else:
            self._pending_normal_frame = frame

        self._last_start_angle_tenths = start_angle_tenths
        return completed_scan

    def reset(self) -> None:
        self._current_points.clear()
        self._pending_normal_frame = None
        self._last_start_angle_tenths = None
        self._is_armed = False

    @classmethod
    def _interpolate_fixed(cls, frame: MeasurementFrame) -> tuple[RadarPoint, ...]:
        if not frame.measurements:
            return ()
        if frame.sector_angle_tenths is None:
            raise ValueError("fixed-resolution frame must include a sector angle")
        step_tenths = (
            0.0
            if len(frame.measurements) == 1
            else frame.sector_angle_tenths / (len(frame.measurements) - 1)
        )
        return cls._create_points(frame, step_tenths)

    @classmethod
    def _interpolate_normal(
        cls,
        frame: MeasurementFrame,
        next_start_angle_tenths: int,
    ) -> tuple[RadarPoint, ...]:
        if not frame.measurements:
            return ()
        start = cls._normalize_tenths(frame.start_angle_tenths)
        clockwise_delta = (
            next_start_angle_tenths - start + cls._FULL_CIRCLE_TENTHS
        ) % cls._FULL_CIRCLE_TENTHS
        return cls._create_points(frame, clockwise_delta / len(frame.measurements))

    @classmethod
    def _create_points(
        cls,
        frame: MeasurementFrame,
        step_tenths: float,
    ) -> tuple[RadarPoint, ...]:
        start = cls._normalize_tenths(frame.start_angle_tenths)
        return tuple(
            RadarPoint(
                angle_deg=((start + step_tenths * index) / 10.0) % 360.0,
                distance_mm=measurement.distance_mm,
                energy=measurement.energy,
            )
            for index, measurement in enumerate(frame.measurements)
        )

    @classmethod
    def _normalize_tenths(cls, angle_tenths: int) -> int:
        return angle_tenths % cls._FULL_CIRCLE_TENTHS


class Lds50cParser:
    """Incrementally parse the four LDS50C frame families from byte chunks."""

    _HEADERS = (b"\xCE\xFA", b"\xCF\xFA", b"ST", b"\xCE\xCE\xCE\xCE")

    def __init__(self, maximum_point_count: int = 4096):
        if not 1 <= maximum_point_count <= 0xFFFF:
            raise ValueError("maximum_point_count must be between 1 and 65535")
        self._maximum_point_count = maximum_point_count
        self._buffer = bytearray()
        self.valid_frames = 0
        self.checksum_errors = 0
        self.length_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[MeasurementFrame | StatusFrame | AlarmFrame]:
        self._buffer.extend(data)
        frames: list[MeasurementFrame | StatusFrame | AlarmFrame] = []

        while self._buffer:
            header_index = self._find_complete_header()
            if header_index < 0:
                self._retain_potential_header_prefix()
                break
            if header_index:
                self._remove_prefix(header_index, count_as_discarded=True)

            if self._buffer.startswith(b"\xCE\xFA"):
                result = self._parse_measurement(frames, fixed_resolution=False)
            elif self._buffer.startswith(b"\xCF\xFA"):
                result = self._parse_measurement(frames, fixed_resolution=True)
            elif self._buffer.startswith(b"ST"):
                result = self._parse_status(frames)
            elif self._buffer.startswith(b"\xCE\xCE\xCE\xCE"):
                result = self._parse_alarm(frames)
            else:
                result = "invalid"

            if result == "need_more":
                break
            if result == "invalid":
                self._remove_prefix(1, count_as_discarded=True)

        return frames

    def reset(self) -> None:
        self._buffer.clear()
        self.valid_frames = 0
        self.checksum_errors = 0
        self.length_errors = 0
        self.discarded_bytes = 0

    def _parse_measurement(self, frames, fixed_resolution: bool) -> str:
        if len(self._buffer) < 6:
            return "need_more"

        point_count = self._read_u16(2)
        if point_count > self._maximum_point_count:
            self.length_errors += 1
            return "invalid"

        metadata_length = 8 if fixed_resolution else 6
        frame_length = metadata_length + point_count * 3 + 2
        if len(self._buffer) < frame_length:
            return "need_more"

        start_angle_tenths = self._read_u16(4)
        sector_angle_tenths = self._read_u16(6) if fixed_resolution else None
        measurements = []
        offset = metadata_length
        for _ in range(point_count):
            measurements.append(
                RawMeasurement(
                    energy=self._buffer[offset],
                    distance_mm=self._read_u16(offset + 1),
                )
            )
            offset += 3

        expected_checksum = self._read_u16(offset)
        checksum = point_count + start_angle_tenths + (sector_angle_tenths or 0)
        checksum += sum(item.energy + item.distance_mm for item in measurements)
        if (checksum & 0xFFFF) != expected_checksum:
            self.checksum_errors += 1
            self._remove_prefix(frame_length, count_as_discarded=True)
            return "consumed"

        frames.append(
            MeasurementFrame(
                fixed_resolution=fixed_resolution,
                point_count=point_count,
                start_angle_tenths=start_angle_tenths,
                sector_angle_tenths=sector_angle_tenths,
                measurements=tuple(measurements),
            )
        )
        self.valid_frames += 1
        self._remove_prefix(frame_length, count_as_discarded=False)
        return "consumed"

    def _parse_status(self, frames) -> str:
        frame_length = 8
        if len(self._buffer) < frame_length:
            return "need_more"
        if self._buffer[6:8] != b"ED":
            self.length_errors += 1
            return "invalid"

        frames.append(
            StatusFrame(
                flags=self._buffer[2],
                reserved_bytes=tuple(self._buffer[3:6]),
            )
        )
        self.valid_frames += 1
        self._remove_prefix(frame_length, count_as_discarded=False)
        return "consumed"

    def _parse_alarm(self, frames) -> str:
        frame_length = 6
        if len(self._buffer) < frame_length:
            return "need_more"
        frames.append(AlarmFrame(code=self._read_u16(4)))
        self.valid_frames += 1
        self._remove_prefix(frame_length, count_as_discarded=False)
        return "consumed"

    def _find_complete_header(self) -> int:
        for offset in range(len(self._buffer)):
            for header in self._HEADERS:
                if self._buffer.startswith(header, offset):
                    return offset
        return -1

    def _retain_potential_header_prefix(self) -> None:
        maximum = min(len(self._buffer), max(len(header) - 1 for header in self._HEADERS))
        for length in range(maximum, 0, -1):
            suffix = self._buffer[-length:]
            if any(len(header) > length and header.startswith(suffix) for header in self._HEADERS):
                self._remove_prefix(len(self._buffer) - length, count_as_discarded=True)
                return
        self._remove_prefix(len(self._buffer), count_as_discarded=True)

    def _read_u16(self, offset: int) -> int:
        return self._buffer[offset] | (self._buffer[offset + 1] << 8)

    def _remove_prefix(self, count: int, count_as_discarded: bool) -> None:
        if count <= 0:
            return
        del self._buffer[:count]
        if count_as_discarded:
            self.discarded_bytes += count
