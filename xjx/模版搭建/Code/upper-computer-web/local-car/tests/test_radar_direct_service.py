import copy
import json
import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from serve import make_server  # noqa: E402


class FakeMainBridge:
    def __init__(self):
        self.auth_token = "shared-test-token"
        self.state = {
            "token": self.auth_token,
            "connected": False,
            "port": "",
            "baudrate": None,
            "last_event_id": 7,
        }

    def status(self):
        return dict(self.state)

    def connect(self, port, baudrate):
        self.state.update(connected=True, port=port, baudrate=baudrate)
        return self.status()

    def send(self, command, owner=None):
        return {"queued": True, "request_id": 19, "command": command, "owner": owner}

    def request_stop(self):
        return {"status": "requested", "token": 123}


class FakeRadarBridge:
    def __init__(self, fail_port=None):
        self.calls = []
        self.fail_port = fail_port

    def status(self):
        self.calls.append(("status",))
        return {
            "connected": False,
            "operation": "idle",
            "scan_revision": 4,
            "last_event_id": 9,
        }

    def latest_scan(self, after_revision=0):
        self.calls.append(("latest_scan", after_revision))
        return {
            "revision": 4,
            "received_at": "2026-10-04T00:00:00+00:00",
            "points": None if after_revision >= 4 else [{"angle_deg": 0, "distance_mm": 100, "energy": 1}],
        }

    def get_events(self, after=0, timeout=0):
        self.calls.append(("get_events", after, timeout))
        return {"events": [], "last_event_id": 9, "gap": False}

    def connect(self, port, baudrate):
        self.calls.append(("connect", port, baudrate))
        if port == self.fail_port:
            raise ValueError("Cannot open radar serial: access denied (port busy)")
        return {"action": "connect", "port": port, "baudrate": baudrate}

    def disconnect(self):
        self.calls.append(("disconnect",))
        return {"action": "disconnect"}

    def start_rotation(self):
        self.calls.append(("start_rotation",))
        return {"action": "start"}

    def stop_rotation(self):
        self.calls.append(("stop_rotation",))
        return {"action": "stop"}

    def capture_single(self):
        self.calls.append(("capture_single",))
        return {"action": "scan", "scan_revision": 5}

    def start_continuous(self):
        self.calls.append(("start_continuous",))
        return {"action": "continuous-start"}

    def stop_continuous(self):
        self.calls.append(("stop_continuous",))
        return {"action": "continuous-stop"}


class RunningServer:
    def __init__(self, main, radar):
        self.server = make_server(main, ROOT, 0, radar_bridge=radar)
        self.origin = f"http://127.0.0.1:{self.server.server_port}"
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(1)

    def request(self, path, body=None, *, authorize=True):
        headers = {}
        data = None
        if body is not None:
            data = json.dumps(body).encode("utf-8")
            headers["Content-Type"] = "application/json"
            if authorize:
                headers["Origin"] = self.origin
                headers["X-Console-Token"] = self.server.bridge.auth_token
        request = Request(self.origin + path, data=data, headers=headers)
        try:
            with urlopen(request) as response:
                return response.status, response.read()
        except HTTPError as error:
            try:
                return error.code, error.read()
            finally:
                error.close()


class RadarDirectServiceTests(unittest.TestCase):
    def setUp(self):
        self.main = FakeMainBridge()
        self.radar = FakeRadarBridge()
        self.running = RunningServer(self.main, self.radar)

    def tearDown(self):
        self.running.close()

    def assert_json(self, path, expected, body=None, *, authorize=True, status=200):
        actual_status, raw = self.running.request(path, body, authorize=authorize)
        self.assertEqual(actual_status, status)
        self.assertEqual(json.loads(raw), expected)

    def test_get_routes_are_read_only_and_validate_revision_cursors(self):
        with patch("serve.available_ports", return_value=[{"device": "COM7", "description": "USB UART"}]):
            self.assert_json(
                "/api/radar-direct/ports",
                {"ports": [{"device": "COM7", "description": "USB UART"}]},
            )
        self.assert_json(
            "/api/radar-direct/status",
            {"connected": False, "operation": "idle", "scan_revision": 4, "last_event_id": 9},
        )
        self.assert_json(
            "/api/radar-direct/scan?after=3",
            {
                "revision": 4,
                "received_at": "2026-10-04T00:00:00+00:00",
                "points": [{"angle_deg": 0, "distance_mm": 100, "energy": 1}],
            },
        )
        self.assert_json(
            "/api/radar-direct/events?after=8",
            {"events": [], "last_event_id": 9, "gap": False},
        )
        for query in ("after=bad", "after=-1"):
            status, raw = self.running.request(f"/api/radar-direct/scan?{query}")
            self.assertEqual(status, 400)
            self.assertIn("after", json.loads(raw)["error"])
        self.assertEqual(
            self.radar.calls,
            [
                ("status",),
                ("latest_scan", 3),
                ("get_events", 8, 0),
            ],
        )

    def test_every_radar_mutation_uses_existing_same_origin_token(self):
        requests = [
            ("/api/radar-direct/connect", {"port": "COM7", "baudrate": 921600}, "connect"),
            ("/api/radar-direct/disconnect", {}, "disconnect"),
            ("/api/radar-direct/start", {}, "start"),
            ("/api/radar-direct/stop", {}, "stop"),
            ("/api/radar-direct/scan", {}, "scan"),
            ("/api/radar-direct/continuous/start", {}, "continuous-start"),
            ("/api/radar-direct/continuous/stop", {}, "continuous-stop"),
        ]
        for path, body, _ in requests:
            status, _raw = self.running.request(path, body, authorize=False)
            self.assertEqual(status, 403, path)

        for path, body, action in requests:
            status, raw = self.running.request(path, body)
            self.assertEqual(status, 200, path)
            self.assertEqual(json.loads(raw)["action"], action)

    def test_unavailable_radar_service_returns_503_without_affecting_main_api(self):
        self.running.close()
        self.running = RunningServer(self.main, None)

        status, raw = self.running.request("/api/radar-direct/status")
        self.assertEqual(status, 503)
        self.assertIn("unavailable", json.loads(raw)["error"].lower())
        status, raw = self.running.request("/api/radar-direct/start", {})
        self.assertEqual(status, 503)
        self.assertIn("unavailable", json.loads(raw)["error"].lower())
        self.assertEqual(json.loads(self.running.request("/api/status")[1]), self.main.status())

    def test_busy_radar_port_does_not_disconnect_or_advance_main_bridge(self):
        self.main.connect("COM7", 115200)
        before = copy.deepcopy(self.main.status())
        self.radar.fail_port = "COM7"

        status, raw = self.running.request(
            "/api/radar-direct/connect",
            {"port": "COM7", "baudrate": 921600},
        )

        self.assertEqual(status, 400)
        self.assertIn("busy", json.loads(raw)["error"])
        self.assertEqual(self.main.status(), before)

    def test_existing_main_routes_are_byte_for_byte_unchanged(self):
        other = RunningServer(FakeMainBridge(), None)
        self.addCleanup(other.close)
        sequence = [
            ("/api/status", None),
            ("/api/connect", {"port": "COM23", "baudrate": 115200}),
            ("/api/send", {"command": "info", "owner": None}),
            ("/api/stop", {}),
        ]

        for path, body in sequence:
            self.assertEqual(
                self.running.request(path, body),
                other.request(path, body),
                path,
            )


if __name__ == "__main__":
    unittest.main()
