import queue
import sys
import threading
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from radar_direct_bridge import RadarDirectBridge  # noqa: E402


STARTUP_COMMANDS = [b"LMDMMH", b"LOCONH", b"LFFF1H", b"LSSS1H", b"LSTARH"]


def measurement_frame(start_angle, count=10, *, distance=100, sector=40):
    payload = bytearray(b"\xCF\xFA")
    payload.extend(count.to_bytes(2, "little"))
    payload.extend(start_angle.to_bytes(2, "little"))
    payload.extend(sector.to_bytes(2, "little"))
    checksum = count + start_angle + sector
    for index in range(count):
        energy = index + 1
        current_distance = distance + index
        payload.append(energy)
        payload.extend(current_distance.to_bytes(2, "little"))
        checksum += energy + current_distance
    payload.extend((checksum & 0xFFFF).to_bytes(2, "little"))
    return bytes(payload)


def status_frame(flags):
    return b"ST" + bytes((flags, 0, 0, 0)) + b"ED"


def alarm_frame(code):
    return b"\xCE\xCE\xCE\xCE" + code.to_bytes(2, "little")


def complete_revolution(distance=100):
    return b"".join(
        measurement_frame(start, distance=distance)
        for start in (3500, 0, 1200, 2400, 3500, 0)
    )


def next_revolution(distance=200):
    return b"".join(
        measurement_frame(start, distance=distance)
        for start in (1200, 2400, 3500, 0)
    )


def wait_for(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.002)
    raise AssertionError("condition timed out")


class FakeRadarSerial:
    def __init__(self, **options):
        self.options = options
        self.writes = []
        self.closed = False
        self._rx = bytearray()
        self._lock = threading.Lock()
        self._read_error = None

    @property
    def in_waiting(self):
        with self._lock:
            return 1 if self._read_error is not None else len(self._rx)

    def feed(self, data):
        with self._lock:
            self._rx.extend(data)

    def fail_read(self, error):
        with self._lock:
            self._read_error = error

    def read(self, count):
        with self._lock:
            if self._read_error is not None:
                error, self._read_error = self._read_error, None
                raise error
            data = bytes(self._rx[:count])
            del self._rx[:count]
            return data

    def write(self, data):
        with self._lock:
            if self.closed:
                raise OSError("port is closed")
            self.writes.append(bytes(data))
            return len(data)

    def close(self):
        with self._lock:
            self.closed = True


class FakeRadarFactory:
    def __init__(self):
        self.ports = []

    def __call__(self, **options):
        port = FakeRadarSerial(**options)
        self.ports.append(port)
        return port


class RadarDirectBridgeTests(unittest.TestCase):
    def create_bridge(self, *, timeout=0.2):
        factory = FakeRadarFactory()
        bridge = RadarDirectBridge(
            serial_factory=factory,
            first_scan_timeout=timeout,
            event_limit=20,
        )
        self.addCleanup(bridge.close)
        return bridge, factory

    def test_instance_identity_is_stable_for_one_service_and_unique_after_restart(self):
        first, _ = self.create_bridge()
        second, _ = self.create_bridge()

        identity = first.status()["instance_id"]

        self.assertTrue(identity)
        self.assertEqual(first.latest_scan()["instance_id"], identity)
        self.assertEqual(first.get_events()["instance_id"], identity)
        self.assertEqual(first.status()["instance_id"], identity)
        self.assertNotEqual(second.status()["instance_id"], identity)

    def test_connect_validates_port_and_reference_baud_and_opens_8n1(self):
        bridge, factory = self.create_bridge()

        with self.assertRaises(ValueError):
            bridge.connect("/dev/ttyUSB0", 500000)
        with self.assertRaises(ValueError):
            bridge.connect("COM7", 115200)

        status = bridge.connect("com7", 921600)

        self.assertTrue(status["connected"])
        self.assertEqual(status["port"], "COM7")
        self.assertEqual(status["baudrate"], 921600)
        self.assertEqual(
            factory.ports[0].options,
            {
                "port": "COM7",
                "baudrate": 921600,
                "bytesize": 8,
                "parity": "N",
                "stopbits": 1,
                "timeout": 0,
                "write_timeout": 1,
                "xonxoff": False,
                "rtscts": False,
                "dsrdtr": False,
            },
        )

    def test_single_scan_sends_reference_preflight_freezes_one_circle_and_stops(self):
        bridge, factory = self.create_bridge()
        bridge.connect("COM7", 921600)
        port = factory.ports[0]
        port.feed(complete_revolution())

        result = bridge.capture_single()

        self.assertEqual(port.writes, STARTUP_COMMANDS + [b"LSTOPH"])
        self.assertEqual(result["scan_revision"], 1)
        scan = bridge.latest_scan(0)
        self.assertEqual(scan["revision"], 1)
        self.assertEqual(len(scan["points"]), 40)
        self.assertEqual(bridge.latest_scan(1)["points"], None)
        self.assertEqual(bridge.status()["operation"], "idle")

    def test_continuous_mode_publishes_only_complete_circles_and_preserves_last_on_failure(self):
        bridge, factory = self.create_bridge(timeout=0.5)
        bridge.connect("COM8", 500000)
        port = factory.ports[0]

        bridge.start_continuous()
        port.feed(complete_revolution(distance=100))
        wait_for(lambda: bridge.status()["scan_revision"] == 1)
        port.feed(next_revolution(distance=200))
        wait_for(lambda: bridge.status()["scan_revision"] == 2)
        retained = bridge.latest_scan(0)
        port.fail_read(OSError("simulated cable fault"))
        wait_for(lambda: bridge.status()["operation"] == "idle")

        status = bridge.status()
        self.assertFalse(status["continuous"])
        self.assertIn("simulated cable fault", status["continuous_error"])
        self.assertEqual(status["scan_revision"], 2)
        self.assertEqual(bridge.latest_scan(0), retained)
        self.assertEqual(port.writes, STARTUP_COMMANDS + [b"LSTOPH"])

    def test_unit_change_resets_only_the_incomplete_revolution(self):
        bridge, factory = self.create_bridge(timeout=0.5)
        bridge.connect("COM9", 384000)
        port = factory.ports[0]
        bridge.start_continuous()

        port.feed(b"".join(measurement_frame(start) for start in (3500, 0, 1200)))
        port.feed(status_frame(0x00) + alarm_frame(0x1234))
        wait_for(lambda: bridge.status()["radar_status"] is not None)
        self.assertEqual(bridge.status()["scan_revision"], 0)

        port.feed(b"".join(measurement_frame(start) for start in (2400, 3500, 0)))
        time.sleep(0.02)
        self.assertEqual(bridge.status()["scan_revision"], 0)
        port.feed(next_revolution(distance=100))
        wait_for(lambda: bridge.status()["scan_revision"] == 1)

        scan = bridge.latest_scan(0)
        self.assertEqual(scan["points"][0]["distance_mm"], 1000)
        status = bridge.status()
        self.assertFalse(status["radar_status"]["uses_millimetres"])
        events = bridge.get_events(0)["events"]
        self.assertTrue(any("0x1234" in event["text"] for event in events))
        bridge.stop_continuous()

    def test_first_circle_timeout_stops_and_retains_previous_complete_scan(self):
        bridge, factory = self.create_bridge(timeout=0.04)
        bridge.connect("COM10", 768000)
        port = factory.ports[0]
        port.feed(complete_revolution())
        bridge.capture_single()

        bridge.start_continuous()
        wait_for(lambda: bridge.status()["operation"] == "idle")

        status = bridge.status()
        self.assertEqual(status["scan_revision"], 1)
        self.assertIn("complete radar revolution", status["continuous_error"])
        self.assertEqual(port.writes[-6:], STARTUP_COMMANDS + [b"LSTOPH"])

    def test_stop_disconnect_and_close_race_converges_and_retains_scan(self):
        bridge, factory = self.create_bridge(timeout=0.5)
        bridge.connect("COM11", 921600)
        port = factory.ports[0]
        bridge.start_continuous()
        port.feed(complete_revolution())
        wait_for(lambda: bridge.status()["scan_revision"] == 1)
        failures = queue.Queue()

        def invoke(action):
            try:
                action()
            except Exception as error:  # pragma: no cover - asserted empty below
                failures.put(error)

        threads = [
            threading.Thread(target=invoke, args=(bridge.stop_continuous,)),
            threading.Thread(target=invoke, args=(bridge.disconnect,)),
            threading.Thread(target=invoke, args=(bridge.close,)),
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(1)

        self.assertTrue(all(not thread.is_alive() for thread in threads))
        self.assertTrue(failures.empty(), list(failures.queue))
        self.assertTrue(port.closed)
        self.assertIn(b"LSTOPH", port.writes)
        self.assertEqual(bridge.latest_scan(0)["revision"], 1)
        self.assertFalse(bridge.status()["connected"])
        self.assertTrue(bridge._worker is None or not bridge._worker.is_alive())

    def test_operations_require_connection_and_remain_mutually_exclusive(self):
        bridge, factory = self.create_bridge(timeout=0.5)
        with self.assertRaises(ValueError):
            bridge.start_rotation()

        bridge.connect("COM12", 500000)
        bridge.start_rotation()
        bridge.stop_rotation()
        self.assertEqual(factory.ports[0].writes, [b"LSTARH", b"LSTOPH"])
        bridge.start_continuous()
        with self.assertRaises(ValueError):
            bridge.capture_single()
        with self.assertRaises(ValueError):
            bridge.start_continuous()
        bridge.stop_continuous()


if __name__ == "__main__":
    unittest.main()
