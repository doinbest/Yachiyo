"""Independent USB-UART owner for the LDS50C direct-radar workspace."""

from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import re
import threading
import time
import uuid

from radar_direct_protocol import (
    AlarmFrame,
    Lds50cParser,
    MeasurementFrame,
    ScanAssembler,
    StatusFrame,
    normalize_measurement,
)


SUPPORTED_BAUDRATES = (384000, 500000, 768000, 921600)
STARTUP_COMMANDS = (b"LMDMMH", b"LOCONH", b"LFFF1H", b"LSSS1H", b"LSTARH")
STOP_COMMAND = b"LSTOPH"


def open_radar_serial(**options):
    try:
        import serial
    except ImportError as error:
        raise RuntimeError(
            "pyserial is missing. Install it with: python -m pip install pyserial"
        ) from error
    return serial.Serial(**options)


class RadarDirectBridge:
    """Own the direct radar port and publish immutable complete-scan snapshots."""

    def __init__(
        self,
        serial_factory=open_radar_serial,
        *,
        first_scan_timeout=8.0,
        event_limit=200,
    ):
        if first_scan_timeout <= 0:
            raise ValueError("first_scan_timeout must be positive")
        if event_limit < 1:
            raise ValueError("event_limit must be positive")

        self.cv = threading.Condition(threading.RLock())
        self._lifecycle_lock = threading.RLock()
        self._serial_factory = serial_factory
        self._first_scan_timeout = float(first_scan_timeout)
        self._instance_id = uuid.uuid4().hex
        self._events = deque(maxlen=event_limit)
        self._serial = None
        self._cancel = threading.Event()
        self._worker = None
        self._shutdown = False

        self._connected = False
        self._port = ""
        self._baudrate = None
        self._operation = "idle"
        self._continuous = False
        self._continuous_error = None
        self._scan_revision = 0
        self._scan_received_at = None
        self._latest_points = ()
        self._radar_status = None
        self._parser_counters = self._empty_parser_counters()
        self._last_event_id = 0

    def connect(self, port: str, baudrate: int) -> dict:
        if not isinstance(port, str) or not re.fullmatch(r"COM[1-9]\d*", port, re.I):
            raise ValueError("Specify a Windows COM port explicitly, for example COM7")
        if type(baudrate) is not int or baudrate not in SUPPORTED_BAUDRATES:
            supported = ", ".join(str(value) for value in SUPPORTED_BAUDRATES)
            raise ValueError(f"Radar baudrate must be one of: {supported}")

        port = port.upper()
        with self._lifecycle_lock:
            with self.cv:
                if self._shutdown:
                    raise ValueError("Radar service is closing")
                if self._connected:
                    raise ValueError("The radar serial port is already open")
                if self._operation != "idle":
                    raise ValueError("Another radar operation is already active")
            try:
                serial_port = self._serial_factory(
                    port=port,
                    baudrate=baudrate,
                    bytesize=8,
                    parity="N",
                    stopbits=1,
                    timeout=0,
                    write_timeout=1,
                    xonxoff=False,
                    rtscts=False,
                    dsrdtr=False,
                )
            except Exception as error:
                self._event(
                    "connection",
                    f"Cannot open radar serial: {error}. If busy/access denied, close the other serial program.",
                    error=str(error),
                )
                raise ValueError(f"Cannot open radar serial: {error}") from error

            with self.cv:
                self._serial = serial_port
                self._connected = True
                self._port = port
                self._baudrate = baudrate
                self._event(
                    "connection",
                    f"Radar connected {port} {baudrate} 8N1",
                    connected=True,
                    port=port,
                    baudrate=baudrate,
                )
                return self._status_locked()

    def disconnect(self) -> dict:
        with self._lifecycle_lock:
            if self._shutdown and self._serial is None:
                return self.status()
            self._stop_active_session()
            self._close_serial()
            return self.status()

    def start_rotation(self) -> dict:
        return self._manual_command(b"LSTARH", "Radar rotation started")

    def stop_rotation(self) -> dict:
        return self._manual_command(STOP_COMMAND, "Radar stop command sent")

    def capture_single(self) -> dict:
        with self._lifecycle_lock:
            self._begin_session("single", continuous=False)
            with self.cv:
                self._worker = threading.current_thread()
        try:
            self._run_scan_session(continuous=False)
            return self.status()
        finally:
            self._finish_session(threading.current_thread())

    def start_continuous(self) -> dict:
        with self._lifecycle_lock:
            self._begin_session("continuous", continuous=True)
            worker = threading.Thread(
                target=self._continuous_worker,
                name="lds50c-direct-radar",
                daemon=True,
            )
            with self.cv:
                self._worker = worker
            worker.start()
            return self.status()

    def stop_continuous(self) -> dict:
        with self._lifecycle_lock:
            with self.cv:
                if self._operation != "continuous":
                    return self._status_locked()
            self._stop_active_session()
            return self.status()

    def status(self) -> dict:
        with self.cv:
            return self._status_locked()

    def latest_scan(self, after_revision: int = 0) -> dict:
        if type(after_revision) is not int or after_revision < 0:
            raise ValueError("after_revision must be a non-negative integer")
        with self.cv:
            points = None
            if self._scan_revision > after_revision:
                points = [
                    {
                        "angle_deg": point.angle_deg,
                        "distance_mm": point.distance_mm,
                        "energy": point.energy,
                    }
                    for point in self._latest_points
                ]
            return {
                "instance_id": self._instance_id,
                "revision": self._scan_revision,
                "received_at": self._scan_received_at,
                "points": points,
            }

    def get_events(self, after: int = 0, timeout: float = 0.0) -> dict:
        if type(after) is not int or after < 0:
            raise ValueError("after must be a non-negative integer")
        if not isinstance(timeout, (int, float)) or timeout < 0:
            raise ValueError("timeout must be non-negative")
        with self.cv:
            if timeout:
                self.cv.wait_for(
                    lambda: self._last_event_id > after or self._shutdown,
                    timeout=min(float(timeout), 1.0),
                )
            return {
                "instance_id": self._instance_id,
                "events": [dict(event) for event in self._events if event["id"] > after],
                "last_event_id": self._last_event_id,
                "gap": bool(self._events and after < self._events[0]["id"] - 1),
            }

    def close(self) -> None:
        with self._lifecycle_lock:
            with self.cv:
                if self._shutdown and self._serial is None and self._worker is None:
                    return
                self._shutdown = True
            had_session = self._stop_active_session()
            if not had_session:
                self._try_stop("Radar stop during service shutdown")
            self._close_serial()

    def _manual_command(self, command: bytes, description: str) -> dict:
        with self._lifecycle_lock:
            with self.cv:
                self._require_connected_locked()
                if self._operation != "idle":
                    raise ValueError("Another radar operation is already active")
            self._write(command)
            self._event("tx", description, command=command.decode("ascii"))
            return self.status()

    def _begin_session(self, operation: str, *, continuous: bool) -> None:
        with self.cv:
            self._require_connected_locked()
            if self._operation != "idle":
                raise ValueError("Another radar operation is already active")
            self._cancel = threading.Event()
            self._operation = operation
            self._continuous = continuous
            if continuous:
                self._continuous_error = None
            self._radar_status = None
            self.cv.notify_all()

    def _continuous_worker(self) -> None:
        current = threading.current_thread()
        try:
            self._run_scan_session(continuous=True)
        except Exception as error:
            with self.cv:
                self._continuous_error = str(error)
            self._event("error", f"Continuous radar scan failed: {error}", error=str(error))
        finally:
            self._finish_session(current)

    def _run_scan_session(self, *, continuous: bool) -> None:
        parser = Lds50cParser()
        assembler = ScanAssembler()
        uses_millimetres = True
        received_first = False
        first_deadline = time.monotonic() + self._first_scan_timeout

        try:
            for command in STARTUP_COMMANDS:
                if self._cancel.is_set():
                    return
                try:
                    self._write(command)
                except Exception as error:
                    name = command.decode("ascii")
                    raise OSError(f"Radar preflight command {name} failed: {error}") from error
                self._event("tx", f"Radar command {command.decode('ascii')}")

            while not self._cancel.is_set():
                if not received_first and time.monotonic() >= first_deadline:
                    raise TimeoutError("Timed out while waiting for one complete radar revolution")

                with self.cv:
                    serial_port = self._serial
                    connected = self._connected
                if serial_port is None or not connected:
                    raise OSError("Radar serial port closed during scan")

                waiting = serial_port.in_waiting
                if not waiting:
                    self._cancel.wait(0.002)
                    continue
                data = serial_port.read(min(waiting, 8192))
                if not data:
                    self._cancel.wait(0.002)
                    continue

                for frame in parser.feed(data):
                    if isinstance(frame, AlarmFrame):
                        self._event("alarm", f"Radar alarm 0x{frame.code:04X}", code=frame.code)
                        continue
                    if isinstance(frame, StatusFrame):
                        if frame.uses_millimetres != uses_millimetres:
                            assembler.reset()
                        uses_millimetres = frame.uses_millimetres
                        with self.cv:
                            self._radar_status = {
                                "flags": frame.flags,
                                "uses_millimetres": frame.uses_millimetres,
                                "energy_enabled": frame.energy_enabled,
                                "trailing_removal_enabled": frame.trailing_removal_enabled,
                                "filter_enabled": frame.filter_enabled,
                            }
                            self.cv.notify_all()
                        continue
                    if not isinstance(frame, MeasurementFrame):
                        continue
                    completed = assembler.push(
                        normalize_measurement(frame, uses_millimetres)
                    )
                    if completed is None:
                        continue
                    received_first = True
                    self._publish_scan(completed)
                    if not continuous:
                        return

                self._set_parser_counters(parser)
        finally:
            self._set_parser_counters(parser)
            self._try_stop("Radar scan cleanup stop")

    def _publish_scan(self, points) -> None:
        with self.cv:
            self._scan_revision += 1
            self._scan_received_at = datetime.now(timezone.utc).isoformat()
            self._latest_points = tuple(points)
            revision = self._scan_revision
            self.cv.notify_all()
        self._event(
            "scan",
            f"Complete radar revolution {revision}: {len(points)} points",
            revision=revision,
            point_count=len(points),
        )

    def _set_parser_counters(self, parser: Lds50cParser) -> None:
        with self.cv:
            self._parser_counters = {
                "valid_frames": parser.valid_frames,
                "checksum_errors": parser.checksum_errors,
                "length_errors": parser.length_errors,
                "discarded_bytes": parser.discarded_bytes,
            }
            self.cv.notify_all()

    def _finish_session(self, worker) -> None:
        with self.cv:
            if self._worker is worker:
                self._worker = None
            self._operation = "idle"
            self._continuous = False
            self.cv.notify_all()

    def _stop_active_session(self) -> bool:
        with self.cv:
            if self._operation == "idle":
                return False
            self._cancel.set()
            worker = self._worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(self._first_scan_timeout + 1.0)
            if worker.is_alive():
                raise RuntimeError("Radar worker did not stop")
        with self.cv:
            self.cv.wait_for(lambda: self._operation == "idle", timeout=1.0)
            if self._operation != "idle":
                raise RuntimeError("Radar operation did not finish")
        return True

    def _write(self, data: bytes) -> None:
        with self.cv:
            serial_port = self._serial
            connected = self._connected
        if serial_port is None or not connected:
            raise OSError("Radar serial port is not open")
        written = serial_port.write(data)
        if written != len(data):
            raise OSError("Incomplete radar serial write")

    def _try_stop(self, description: str) -> None:
        with self.cv:
            if self._serial is None or not self._connected:
                return
        try:
            self._write(STOP_COMMAND)
            self._event("tx", description, command="LSTOPH")
        except Exception as error:
            self._event("error", f"Radar stop command failed: {error}", error=str(error))

    def _close_serial(self) -> None:
        with self.cv:
            serial_port = self._serial
            self._serial = None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception as error:
                self._event("error", f"Radar serial close failed: {error}", error=str(error))
        with self.cv:
            was_connected = self._connected
            self._connected = False
            self._port = ""
            self._baudrate = None
            self._operation = "idle"
            self._continuous = False
            self._worker = None
            if was_connected:
                self._event("connection", "Radar serial disconnected", connected=False)
            self.cv.notify_all()

    def _require_connected_locked(self) -> None:
        if not self._connected or self._serial is None:
            raise ValueError("Radar serial port is not open; connect explicitly first")

    def _event(self, kind: str, text: str, **fields) -> dict:
        with self.cv:
            self._last_event_id += 1
            event = {
                "id": self._last_event_id,
                "kind": kind,
                "text": text,
                "time": datetime.now(timezone.utc).isoformat(),
                **fields,
            }
            self._events.append(event)
            self.cv.notify_all()
            return event

    def _status_locked(self) -> dict:
        return {
            "instance_id": self._instance_id,
            "connected": self._connected,
            "port": self._port,
            "baudrate": self._baudrate,
            "operation": self._operation,
            "continuous": self._continuous,
            "continuous_error": self._continuous_error,
            "scan_revision": self._scan_revision,
            "scan_received_at": self._scan_received_at,
            "radar_status": dict(self._radar_status) if self._radar_status else None,
            "parser": dict(self._parser_counters),
            "last_event_id": self._last_event_id,
        }

    @staticmethod
    def _empty_parser_counters() -> dict:
        return {
            "valid_frames": 0,
            "checksum_errors": 0,
            "length_errors": 0,
            "discarded_bytes": 0,
        }
