import importlib.util
import json
import io
from contextlib import redirect_stdout
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from urllib.request import Request, urlopen
from urllib.error import HTTPError

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from serial_bridge import SerialBridge, READ_COMMAND
from serve import make_server, print_events, console_command
import serve


def wait_for(predicate, timeout=2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError('condition timed out')


class FakeSerial:
    def __init__(self, **kwargs):
        self.options = kwargs
        self.calls = [('open', threading.get_ident())]
        self.writes = []
        self.rx = queue.Queue()
        self.closed = False

    @property
    def in_waiting(self):
        self.calls.append(('waiting', threading.get_ident()))
        return 1 if not self.rx.empty() else 0

    def read(self, count):
        self.calls.append(('read', threading.get_ident()))
        try:
            return self.rx.get_nowait()
        except queue.Empty:
            return b''

    def write(self, data):
        self.calls.append(('write', threading.get_ident()))
        self.writes.append((time.monotonic(), data))
        return len(data)

    def close(self):
        self.calls.append(('close', threading.get_ident()))
        self.closed = True


class BridgeTests(unittest.TestCase):
    def test_diagnostic_queries_are_read_only(self):
        self.assertIsNotNone(READ_COMMAND.fullmatch('console status'))
        self.assertIsNotNone(READ_COMMAND.fullmatch('chassis snapshot'))
        self.assertIsNone(READ_COMMAND.fullmatch('console status extra'))

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.ports = []
        def factory(**kwargs):
            port = FakeSerial(**kwargs)
            self.ports.append(port)
            return port
        self.bridge = SerialBridge(Path(self.temp.name), serial_factory=factory,
                                   stop_timeout=.75, command_interval=.08, stop_interval=.2)

    def tearDown(self):
        self.bridge.close()
        self.temp.cleanup()

    def connect(self):
        self.bridge.connect('COM23', 115200)
        return self.ports[-1]

    def reply(self, port, token, complete=1, stopped=1):
        port.rx.put(f'OK chassis stop id=7 token={token} requested_ms=20 tx_complete={complete} wheels_stopped={stopped} reason=done\r\n'.encode())

    def test_owner_thread_and_pacing(self):
        port = self.connect()
        self.bridge.send('info')
        self.bridge.send('imu status')
        wait_for(lambda: len(port.writes) == 2)
        self.assertGreaterEqual(port.writes[1][0] - port.writes[0][0], .07)
        self.assertEqual(port.writes[0][1], b'info\r\n')
        self.assertEqual(port.options['bytesize'], 8)
        self.assertEqual(port.options['parity'], 'N')
        self.assertEqual(port.options['stopbits'], 1)
        self.bridge.close()
        self.assertEqual(len({ident for _, ident in port.calls}), 1)
        self.assertNotEqual(port.calls[0][1], threading.get_ident())

    def test_stop_device_id_does_not_replace_event_cursor(self):
        port = self.connect()
        for _ in range(20):
            self.bridge._event('system', 'prior event')
        token = self.bridge.request_stop()['token']
        wait_for(lambda: any(str(token).encode() in data for _, data in port.writes))
        after = self.bridge.status()['last_event_id']
        self.reply(port, token)
        wait_for(lambda: self.bridge.status()['stop']['status'] == 'confirmed')
        events = self.bridge.get_events(after, 0)['events']
        stops = [e for e in events if e['kind'] == 'stop']
        self.assertEqual(len(stops), 1)
        self.assertEqual(stops[0]['stop_id'], 7)
        self.assertGreater(stops[0]['id'], after)
        after = self.bridge.status()['last_event_id']
        self.bridge.resume()
        events = self.bridge.get_events(after, 0)['events']
        self.assertEqual(events[0]['stop_id'], 7)
        self.assertGreater(events[0]['id'], after)
        self.assertFalse(events[0]['latched'])

    def test_stop_preempts_pending_and_repeated_request_is_same_stop(self):
        port = self.connect()
        self.bridge.send('info')
        wait_for(lambda: len(port.writes) == 1)
        self.bridge.send('chassis run 10 0 0 1000')
        stop = self.bridge.request_stop()
        self.assertEqual(stop['token'], self.bridge.request_stop()['token'])
        wait_for(lambda: len(port.writes) >= 3)
        self.assertEqual([data for _, data in port.writes[1:3]],
                         [b'\x03', f'chassis stop {stop["token"]}\r\n'.encode()])
        self.assertTrue(self.bridge.status()['stop_latched'])
        self.assertFalse(any(b'chassis run' in data for _, data in port.writes))
        with self.assertRaises(ValueError):
            self.bridge.send('chassis run 10 0 0 1000')
        self.reply(port, stop['token'])
        wait_for(lambda: self.bridge.status()['stop']['status'] == 'confirmed')
        self.bridge.resume()
        self.assertFalse(self.bridge.status()['stop_latched'])

    def test_only_complete_matching_stop_reply_confirms(self):
        port = self.connect()
        token = self.bridge.request_stop()['token']
        wait_for(lambda: len(port.writes) >= 2)
        port.rx.put(b'OK chassis stop\r\n')
        self.reply(port, token + 1)
        port.rx.put(f'OK chassis stop id=7 token={token} requested_ms=20 tx_complete=1 '.encode())
        time.sleep(.05)
        self.assertNotEqual(self.bridge.status()['stop']['status'], 'confirmed')
        port.rx.put(b'wheels_stopped=1 reason=done\r\n')
        wait_for(lambda: self.bridge.status()['stop']['status'] == 'confirmed')

    def test_timeout_has_three_attempts_and_no_false_confirmation(self):
        port = self.connect()
        token = self.bridge.request_stop()['token']
        wait_for(lambda: self.bridge.status()['stop']['status'] == 'timeout')
        self.assertEqual(sum(data == f'chassis stop {token}\r\n'.encode() for _, data in port.writes), 3)
        self.assertFalse(self.bridge.status()['stop']['wheels_stopped'])

    def test_accepted_stop_only_polls_and_late_reply_does_not_restart_traffic(self):
        port = self.connect()
        token = self.bridge.request_stop()['token']
        wait_for(lambda: len(port.writes) == 2)
        self.assertNotIn(b'chassis stop\r\n', [data for _, data in port.writes])
        self.reply(port, token, complete=0, stopped=0)
        wait_for(lambda: self.bridge.status()['stop']['status'] == 'accepted')
        wait_for(lambda: self.bridge.status()['stop']['status'] == 'timeout')
        self.assertEqual(sum(data == f'chassis stop {token}\r\n'.encode() for _, data in port.writes), 1)
        queries = [t for t, data in port.writes if data == b'chassis stop-status\r\n']
        self.assertTrue(queries)
        self.assertTrue(all(b-a >= .19 for a,b in zip(queries, queries[1:])))
        count = len(port.writes)
        self.reply(port, token, complete=1, stopped=0)
        wait_for(lambda: self.bridge.status()['stop']['tx_complete'])
        self.assertEqual(self.bridge.status()['stop']['status'], 'timeout')
        time.sleep(.25)
        self.assertEqual(len(port.writes), count)
        self.reply(port, token+1)
        time.sleep(.03)
        self.assertFalse(self.bridge.status()['stop']['wheels_stopped'])
        self.reply(port, token)
        wait_for(lambda: self.bridge.status()['stop']['status'] == 'confirmed')
        self.assertTrue(self.bridge.status()['stop_latched'])
        self.assertEqual(len(port.writes), count)

    def test_late_reply_after_resume_is_ignored(self):
        port = self.connect()
        token = self.bridge.request_stop()['token']
        wait_for(lambda: self.bridge.status()['stop']['status'] == 'timeout')
        self.bridge.resume()
        self.reply(port, token)
        time.sleep(.04)
        self.assertEqual(self.bridge.status()['stop']['status'], 'timeout')
        self.assertFalse(self.bridge.status()['stop_latched'])

    def test_disconnect_waits_and_reconnect_never_replays(self):
        port = self.connect()
        self.bridge.send('info')
        wait_for(lambda: len(port.writes) == 1)
        self.bridge.send('chassis run 10 0 0 1000')
        thread = threading.Thread(target=self.bridge.disconnect)
        thread.start()
        wait_for(lambda: self.bridge.status()['stop_latched'])
        self.reply(port, self.bridge.status()['stop']['token'])
        thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertTrue(port.closed)
        new = self.connect()
        time.sleep(.12)
        self.assertEqual(new.writes, [])
        self.assertTrue(self.bridge.status()['stop_latched'])

    def test_validation_and_config_persistence_do_not_send(self):
        port = self.connect()
        for value in ['', 'info\nchassis run 1 0 0 10', '你好', 'x' * 64, 'info\x03']:
            with self.assertRaises(ValueError):
                self.bridge.send(value)
        self.assertEqual(self.bridge.status()['config']['profile'], '')
        self.bridge.save_config({'profile':'receive','units_per_rev':65536,'directions_confirmed':True})
        self.assertEqual(port.writes, [])
        self.assertEqual(json.loads((Path(self.temp.name)/'local-car.json').read_text())['profile'], 'receive')

    def test_preparation_blocks_cmd_motion_and_other_page_mutations(self):
        port = self.connect()
        self.bridge.preparation(True, 'page-one')
        for command, owner in [('chassis run 10 0 0 1000', None),
                               ('chassis run 10 0 0 1000', 'page-one'),
                               ('chassis profile receive', 'page-two')]:
            with self.assertRaises(ValueError):
                self.bridge.send(command, owner=owner)
        self.bridge.send('imu status')
        self.bridge.send('chassis units 65536', owner='page-one')
        self.bridge.send('imu verify 5', owner='page-one')
        with self.assertRaises(ValueError):
            self.bridge.send('imu verify 10', owner='page-one')
        with self.assertRaises(ValueError):
            self.bridge.preparation(False, 'page-two')
        self.bridge.request_stop()
        self.assertFalse(self.bridge.status()['preparation']['active'])
        self.assertFalse(any(b'chassis run' in data for _, data in port.writes))

    def test_cancel_from_cmd_ends_preparation_without_motion(self):
        port = self.connect()
        self.bridge.preparation(True, 'page-one')
        self.bridge.send('imu cal cancel')
        wait_for(lambda: bool(port.writes))
        self.assertFalse(self.bridge.status()['preparation']['active'])
        self.assertEqual(port.writes[0][1], b'imu cal cancel\r\n')

    def test_cancelled_preparation_rejects_delayed_owner_commands(self):
        port = self.connect()
        self.bridge.preparation(True, 'page-one')
        self.bridge.preparation(False, 'page-one')
        for command in ('chassis origin 2250 150 90', 'chassis stream on 42', 'imu status'):
            with self.assertRaises(ValueError):
                self.bridge.send(command, owner='page-one')
        self.assertEqual(port.writes, [])

    def test_cmd_stop_stays_open_and_telemetry_is_summarized(self):
        port = self.connect()
        self.assertTrue(console_command(self.bridge, 'stop', 'COM23', 115200))
        token = self.bridge.status()['stop']['token']
        self.assertTrue(console_command(self.bridge, 'stop', 'COM23', 115200))
        self.assertEqual(self.bridge.status()['stop']['token'], token)
        self.assertFalse(console_command(self.bridge, 'quit', 'COM23', 115200))
        self.reply(port, token)
        wait_for(lambda: self.bridge.status()['stop']['status'] == 'confirmed')
        output = io.StringIO()
        finished = threading.Event()
        with redirect_stdout(output):
            thread = threading.Thread(target=print_events, args=(self.bridge, finished))
            thread.start()
            try:
                port.rx.put(b'@CHASSIS {"kind":"state","t_ms":120,"session":3,"detail":"VERY_LONG_JSON"}\r\n')
                wait_for(lambda: 'MAP state' in output.getvalue())
            finally:
                finished.set()
                thread.join(1)
        self.assertNotIn('VERY_LONG_JSON', output.getvalue())

    def test_ctrl_c_discards_partially_entered_command_even_if_resumed(self):
        port = self.connect()
        handlers = []
        count = 0
        def input_line(prompt):
            nonlocal count
            count += 1
            if count == 1:
                handlers[0](2, None)
                handlers[0](2, None)
                self.reply(port, self.bridge.status()['stop']['token'])
                wait_for(lambda: self.bridge.status()['stop']['status'] == 'confirmed')
                self.bridge.resume()  # Another webpage may explicitly resume while input is still open.
                return 'chassis run 10 0 0 1000'
            time.sleep(.15)  # Keep the next prompt open long enough for an erroneously queued command.
            return 'quit'
        def register_signal(signum, handler):
            handlers.append(handler)
            return lambda *_: None
        with patch.object(serve, 'SerialBridge', return_value=self.bridge), \
             patch.object(serve.signal, 'signal', side_effect=register_signal), \
             patch('builtins.input', side_effect=input_line), \
             patch.object(sys, 'argv', ['serve.py','--no-browser','--port','0']), \
             redirect_stdout(io.StringIO()):
            self.assertEqual(serve.main(), 0)
        self.assertFalse(any(b'chassis run' in data for _, data in port.writes))

    def test_busy_port_reports_error_without_connection_or_command_replay(self):
        self.bridge.factory = lambda **kwargs: (_ for _ in ()).throw(PermissionError('Access denied'))
        with self.assertRaisesRegex(ValueError, 'busy/access denied'):
            self.bridge.connect('COM23', 115200)
        self.assertFalse(self.bridge.status()['connected'])

    def test_event_gap_request_ids_and_raw_logs(self):
        port = self.connect()
        result = self.bridge.send('info')
        wait_for(lambda: len(port.writes) == 1)
        port.rx.put(b'hello\r\n')
        wait_for(lambda: any(e['kind'] == 'rx' for e in self.bridge.get_events(0, 0)['events']))
        events = self.bridge.get_events(0, 0)['events']
        self.assertTrue(any(e.get('request_id') == result['request_id'] and e['kind'] == 'tx' for e in events))
        self.assertEqual(next((Path(self.temp.name)/'logs'/'serial').glob('*-rx.bin')).read_bytes(), b'hello\r\n')
        self.bridge.events = __import__('collections').deque(self.bridge.events, maxlen=2)
        self.bridge.save_config({'profile':'none'})
        self.assertTrue(self.bridge.get_events(0, 0)['gap'])

    def test_ports_endpoint_lists_devices_without_opening_them(self):
        from types import SimpleNamespace
        server = make_server(self.bridge, ROOT, 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        origin = f'http://127.0.0.1:{server.server_port}'
        try:
            with patch('serial.tools.list_ports.comports', return_value=[
                SimpleNamespace(device='COM7', description='USB-SERIAL CH340')]):
                with urlopen(origin+'/api/ports') as response:
                    self.assertEqual(json.load(response), {'ports':[
                        {'device':'COM7', 'description':'USB-SERIAL CH340'}]})
            with patch('serial.tools.list_ports.comports', return_value=[]):
                with urlopen(origin+'/api/ports') as response:
                    self.assertEqual(json.load(response), {'ports':[]})
            self.assertEqual(self.ports, [])
        finally:
            server.shutdown()
            server.server_close()

    def test_cmd_connect_requires_explicit_port(self):
        with self.assertRaises(ValueError):
            console_command(self.bridge, 'connect', None, 115200)
        self.assertEqual(self.ports, [])
        console_command(self.bridge, 'connect COM7', None, 115200)
        self.assertEqual(self.ports[-1].options['port'], 'COM7')

    def test_http_rejects_origin_host_and_token_and_serves_static(self):
        server = make_server(self.bridge, ROOT, 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        origin = f'http://127.0.0.1:{server.server_port}'
        try:
            with urlopen(origin + '/api/status') as response:
                status = json.load(response)
            for headers in [{}, {'Origin':'https://evil.test','X-Console-Token':status['token']},
                            {'Origin':origin,'X-Console-Token':'wrong'}]:
                with self.assertRaises(HTTPError) as caught:
                    urlopen(Request(origin+'/api/connect',data=b'{"port":"COM23","baudrate":115200}',headers=headers))
                self.assertEqual(caught.exception.code, 403)
                caught.exception.close()
            for headers in [{'Host':'evil.test'}, {'Origin':'https://evil.test'}]:
                with self.assertRaises(HTTPError) as caught:
                    urlopen(Request(origin+'/api/status',headers=headers))
                caught.exception.close()
            headers = {'Origin':origin,'X-Console-Token':status['token'],'Content-Type':'application/json'}
            with urlopen(Request(origin+'/api/connect',data=b'{"port":"COM23","baudrate":115200}',headers=headers)) as response:
                self.assertTrue(json.load(response)['connected'])
            with urlopen(Request(origin+'/api/send',data=b'{"command":"info"}',headers=headers)) as response:
                self.assertGreater(json.load(response)['request_id'], 0)
            with urlopen(origin+'/index.html') as response:
                self.assertIn(b'<!', response.read()[:50])
        finally:
            server.shutdown()
            server.server_close()


if __name__ == '__main__':
    unittest.main()
