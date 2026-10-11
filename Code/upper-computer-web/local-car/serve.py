"""Loopback dashboard and CMD console sharing one pyserial owner."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import math
import shutil
import subprocess
import signal
import threading
import time
from urllib.parse import parse_qs, urlsplit
import webbrowser
from radar_direct_bridge import RadarDirectBridge
from serial_bridge import SerialBridge


_planner_lock = threading.Lock()
RADAR_DEFAULTS = dict(lidar_x_mm=2250, lidar_y_mm=150, zero_deg=180,
                     distance_min_mm=100, distance_max_mm=40000, angle_min_tenths=0,
                     angle_max_tenths=3600, threshold=1, energy_min=0, energy_max=255)


def plan_radar_offline(body):
    """Run the exact firmware C core locally. This never receives a serial bridge."""
    start_mode = body.get('start_mode', 'manual')
    if start_mode not in ('manual', 'auto', 'upper', 'lower'):
        raise ValueError('start_mode must be manual, auto, upper or lower')
    speed = body.get('speed_mm_s', 200)
    if type(speed) not in (int, float) or not math.isfinite(speed) or not 10 <= speed <= 5000:
        raise ValueError('Route speed must be a finite number from 10 to 5000 mm/s')
    mask = body.get('mask', 0)
    if type(mask) is not int or not 0 <= mask < (1 << 25):
        raise ValueError('mask must describe 25 cells')
    params = {**RADAR_DEFAULTS, **body.get('params', {})}
    if any(type(params[key]) not in (int, float) or not math.isfinite(params[key]) for key in RADAR_DEFAULTS):
        raise ValueError('Radar parameters must be finite numbers')
    for key in list(RADAR_DEFAULTS)[3:]:
        if params[key] != int(params[key]):
            raise ValueError('Radar filter parameters must be integers')
        params[key] = int(params[key])
    points = body.get('points', [])
    circles = body.get('circles')
    if circles is not None:
        if not isinstance(circles, list) or not 15 <= len(circles) <= 256:
            raise ValueError('稳定建图需要至少15个带完整圈边界的有效圈，最多256圈')
        if points:
            raise ValueError('circles与最后一圈诊断points不能同时作为建图输入')
        if any(not isinstance(circle, list) or len(circle) > 65535 for circle in circles):
            raise ValueError('每圈必须是完整测量列表')
        points = [point for circle in circles for point in circle]
    if not isinstance(points, list) or (circles is None and len(points) > 4096):
        raise ValueError('A cloud contains at most 4096 points')
    if points and body.get('cloud_complete') is False:
        raise ValueError('完整圈建图需要未截断的完整点云；当前点云仍可查看，也可按已有地图或手工障碍规划')
    for point in points:
        if not isinstance(point, list) or len(point) != 3 or any(type(v) is not int for v in point):
            raise ValueError('Points are integer [angle_tenths, distance_mm, energy]')
        if not 0 <= point[0] <= 3599 or not 0 <= point[1] <= 65535 or not 0 <= point[2] <= 255:
            raise ValueError('Point values exceed the radar protocol range')
    workspace = Path(__file__).resolve().parents[3]
    sources = [workspace/'Code/tools/radar_plan_cli.c', workspace/'Code/template/App/radar_map.c',
               workspace/'Code/template/App/radar_start.c',
               workspace/'Code/template/App/radar_geometry.c', workspace/'Code/template/App/radar_stability.c',
               workspace/'Code/template/App/chassis_route_profile.c']
    headers = [workspace/'Code/template/App/radar_map.h', workspace/'Code/template/App/radar_start.h',
               workspace/'Code/template/App/radar_geometry.h', workspace/'Code/template/App/radar_stability.h',
               workspace/'Code/template/App/chassis_route_profile.h',
               workspace/'Code/template/App/chassis_config.h', workspace/'Code/template/App/motor_id_config.h']
    executable = workspace/'.embeddedskills/tests/radar_plan_cli.exe'
    with _planner_lock:
        if not all(path.is_file() for path in sources + headers):
            raise OSError('Shared radar C planner source is missing')
        if not executable.is_file() or any(path.stat().st_mtime > executable.stat().st_mtime for path in sources + headers):
            compiler = shutil.which('gcc')
            if not compiler:
                raise OSError('Offline planning needs GCC; install the same GCC used by firmware host tests')
            executable.parent.mkdir(parents=True, exist_ok=True)
            built = subprocess.run([compiler, '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                                    '-I'+str(workspace/'Code/template/App'), *map(str, sources),
                                    '-lm', '-o', str(executable)], capture_output=True, text=True, timeout=30)
            if built.returncode:
                raise OSError('Shared C planner build failed: '+built.stderr[-2000:])
        if circles is None:
            request = f'RADAR1 {mask} {len(points)}\n'+' '.join(str(params[key]) for key in RADAR_DEFAULTS)+'\n'
            request += ''.join(' '.join(map(str, point))+'\n' for point in points)
        else:
            # Frozen installation travels with the circle recording. Browser
            # local settings never stand in for missing measurement evidence.
            geometry = body.get('geometry', {})
            situation = body.get('situation', 2)
            if situation not in (1, 2):
                raise ValueError('situation must be 1 or 2')
            matrix = geometry.get('matrix_q16', [65809, 680, -407, 66623] if situation == 2 else [65536, 0, 0, 65536])
            origin = geometry.get('origin_q16', [149407997, 20569846] if situation == 2 else [15073280, 15073280])
            zero = geometry.get('zero_tenths', 1800 if situation == 2 else 900)
            if not isinstance(matrix, list) or len(matrix) != 4 or not isinstance(origin, list) or len(origin) != 2:
                raise ValueError('geometry requires four matrix and two origin coefficients')
            if any(type(v) is not int or not -(1 << 31) <= v < (1 << 31) for v in matrix + origin):
                raise ValueError('Q16 coefficients must be int32')
            if type(zero) is not int or not 0 <= zero < 3600:
                raise ValueError('zero_tenths must be from 0 to 3599')
            if not body.get('params'):
                params.update(lidar_x_mm=2280 if situation == 2 else 230,
                              lidar_y_mm=314 if situation == 2 else 230, zero_deg=zero / 10)
            request = f'RADAR2 {mask} {len(circles)}\n'+' '.join(str(params[key]) for key in RADAR_DEFAULTS)+'\n'
            request += ' '.join(map(str, [situation, *matrix, *origin, zero]))+'\n'
            request += ''.join(str(len(circle))+'\n'+''.join(' '.join(map(str, point))+'\n' for point in circle) for circle in circles)
        result = subprocess.run([str(executable), '--speed', str(speed), '--start', start_mode], input=request,
                                capture_output=True, text=True, timeout=15)
        try:
            output = json.loads(result.stdout)
        except ValueError as error:
            raise OSError('C planner did not return JSON: '+result.stderr[-500:]) from error
        if result.returncode or 'error' in output:
            raise ValueError('C planner rejected input: '+str(output.get('error', result.returncode)))
        return output


def available_ports():
    """Enumerate only; this never opens a serial device."""
    try:
        from serial.tools import list_ports
    except ImportError as error:
        raise OSError('Install pyserial: python -m pip install pyserial') from error
    return [{'device':p.device, 'description':p.description or ''}
            for p in sorted(list_ports.comports(), key=lambda p:p.device)]


class ConsoleHandler(SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def end_headers(self):
        # Local dashboard assets change during development; always fetch fresh files.
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def _json(self, status, value):
        data = json.dumps(value, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def _local_request(self, mutation=False):
        origin = self.server.console_origin
        if self.headers.get('Host') != urlsplit(origin).netloc:
            self._json(403, {'error':'Only this loopback Host is allowed'})
            return False
        request_origin = self.headers.get('Origin')
        if request_origin is not None and request_origin != origin:
            self._json(403, {'error':'Cross-origin access is not allowed'})
            return False
        if self.headers.get('Sec-Fetch-Site') == 'cross-site':
            self._json(403, {'error':'Cross-site access is not allowed'})
            return False
        if mutation and (request_origin != origin or not secrets.compare_digest(
                self.headers.get('X-Console-Token', ''), self.server.bridge.auth_token)):
            self._json(403, {'error':'Same Origin and X-Console-Token are required'})
            return False
        return True

    def do_GET(self):
        if not self._local_request():
            return
        path = urlsplit(self.path)
        if path.path == '/api/status':
            self._json(200, self.server.bridge.status())
        elif path.path in ('/api/logs', '/api/logs/tracking', '/api/logs/export'):
            from log_archive import LogArchive
            archive = LogArchive(self.server.bridge.state_dir / 'logs' / 'serial')
            query = parse_qs(path.query)
            try:
                if path.path == '/api/logs':
                    self._json(200, {'logs': archive.list(),
                                     'current': Path(self.server.bridge.rx_file.name).name[:-7]})
                elif path.path == '/api/logs/tracking':
                    self._json(200, {'sessions': archive.tracking(query.get('name', [''])[0])})
                else:
                    name = query.get('name', [''])[0]
                    kind = query.get('kind', ['rx'])[0]
                    segment = int(query['segment'][0]) if 'segment' in query else None
                    with archive.export(name, kind, segment) as stream:
                        stream.seek(0, 2)
                        size = stream.tell()
                        stream.seek(0)
                        self.send_response(200)
                        self.send_header('Content-Type', 'text/plain; charset=utf-8')
                        self.send_header('Content-Length', str(size))
                        self.send_header('Content-Disposition', f'attachment; filename="serial-{name}-{kind}{"-"+str(segment) if segment is not None else ""}.txt"')
                        self.end_headers()
                        try:
                            while chunk := stream.read(65536):
                                self.wfile.write(chunk)
                        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                            pass
            except (ValueError, OSError) as error:
                self._json(400, {'error': str(error)})
        elif path.path == '/api/ports':
            try:
                self._json(200, {'ports':available_ports()})
            except OSError as error:
                self._json(503, {'error':str(error)})
        elif path.path == '/api/events':
            try:
                after = int(parse_qs(path.query).get('after', ['0'])[0])
                if after < 0:
                    raise ValueError('after must be nonnegative')
                self._json(200, self.server.bridge.get_events(after))
            except ValueError as error:
                self._json(400, {'error':str(error)})
        elif path.path in ('/api/radar-direct/ports', '/api/radar-direct/status',
                            '/api/radar-direct/scan', '/api/radar-direct/events'):
            radar_bridge = self.server.radar_bridge
            if radar_bridge is None:
                self._json(503, {'error':'Direct radar service is unavailable'})
                return
            try:
                if path.path == '/api/radar-direct/ports':
                    self._json(200, {'ports':available_ports()})
                elif path.path == '/api/radar-direct/status':
                    self._json(200, radar_bridge.status())
                else:
                    try:
                        after = int(parse_qs(path.query).get('after', ['0'])[0])
                    except (TypeError, ValueError) as error:
                        raise ValueError('after must be a nonnegative integer') from error
                    if after < 0:
                        raise ValueError('after must be a nonnegative integer')
                    if path.path == '/api/radar-direct/scan':
                        self._json(200, radar_bridge.latest_scan(after))
                    else:
                        self._json(200, radar_bridge.get_events(after))
            except ValueError as error:
                self._json(400, {'error':str(error)})
            except OSError as error:
                self._json(503, {'error':str(error)})
        elif path.path.startswith('/api/'):
            self._json(404, {'error':'Unknown API'})
        else:
            super().do_GET()

    def do_HEAD(self):
        if self._local_request():
            super().do_HEAD()

    def do_POST(self):
        if not self._local_request(mutation=True):
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            limit = 4 * 1024 * 1024 if urlsplit(self.path).path == '/api/radar/plan' else 8192
            if not 0 <= length <= limit:
                raise ValueError('Request body too large')
            self.connection.settimeout(3)
            body = json.loads(self.rfile.read(length) or b'{}')
            if not isinstance(body, dict):
                raise ValueError('JSON body must be an object')
            bridge = self.server.bridge
            path = urlsplit(self.path).path
            if path == '/api/radar/plan':
                result = plan_radar_offline(body)
            elif path in ('/api/radar-direct/connect', '/api/radar-direct/disconnect',
                           '/api/radar-direct/start', '/api/radar-direct/stop',
                           '/api/radar-direct/scan', '/api/radar-direct/continuous/start',
                           '/api/radar-direct/continuous/stop'):
                radar_bridge = self.server.radar_bridge
                if radar_bridge is None:
                    self._json(503, {'error':'Direct radar service is unavailable'})
                    return
                if path == '/api/radar-direct/connect':
                    result = radar_bridge.connect(body.get('port'), body.get('baudrate'))
                elif path == '/api/radar-direct/disconnect':
                    result = radar_bridge.disconnect()
                elif path == '/api/radar-direct/start':
                    result = radar_bridge.start_rotation()
                elif path == '/api/radar-direct/stop':
                    result = radar_bridge.stop_rotation()
                elif path == '/api/radar-direct/scan':
                    result = radar_bridge.capture_single()
                elif path == '/api/radar-direct/continuous/start':
                    result = radar_bridge.start_continuous()
                else:
                    result = radar_bridge.stop_continuous()
            elif path == '/api/connect':
                result = bridge.connect(body.get('port'), body.get('baudrate'))
            elif path == '/api/send':
                result = bridge.send(body.get('command'), owner=body.get('owner'),
                                     command_epoch=body.get('command_epoch'), require_epoch=True)
            elif path == '/api/stop':
                result = bridge.request_stop()
            elif path == '/api/disconnect':
                result = bridge.disconnect()
            elif path == '/api/config':
                result = bridge.save_config(body)
            elif path == '/api/resume':
                result = bridge.resume(stop_token=body.get('stop_token'), require_token=True)
            elif path == '/api/preparation':
                result = bridge.preparation(body.get('active'), body.get('owner'))
            else:
                self._json(404, {'error':'Unknown API'})
                return
            self._json(200, result)
        except (ValueError, TypeError, OSError, subprocess.TimeoutExpired) as error:
            self._json(400, {'error':str(error)})


def make_server(bridge, root, port=8765, radar_bridge=None):
    server = ThreadingHTTPServer(('127.0.0.1', port), partial(ConsoleHandler, directory=str(root)))
    server.daemon_threads = True
    server.bridge = bridge
    server.radar_bridge = radar_bridge
    server.console_origin = f'http://127.0.0.1:{server.server_port}'
    return server


def print_events(bridge, finished):
    after = 0
    last_telemetry = 0
    telemetry_count = 0
    while not finished.is_set():
        batch = bridge.get_events(after, .5)
        after = batch['last_event_id']
        for event in batch['events']:
            if event['kind'] == 'raw':
                continue
            text = event['text']
            # The raw RX/JSONL capture retains every row. Keep CMD diagnostics readable.
            payload = text.strip()
            while payload.startswith('arm>'):
                payload = payload[4:].lstrip()
            if event['kind'] == 'rx' and payload.startswith(('PIX ', 'TRACK live ', 'TRACK row ')):
                continue
            if event['kind'] == 'tx' and text.startswith('radar fetch ') or text == 'OK radar page_pending':
                continue
            if event['kind'] in ('rx', 'radar') and text.startswith('@RADAR '):
                try:
                    radar = json.loads(text[7:])
                    if radar.get('k') not in ('status', 'nav', 'probe'):
                        continue
                    if radar.get('k') != 'probe':
                        text = f'RADAR {radar.get("k")} {radar.get("state", "")} {radar.get("reason", "")} (snapshot data; raw RX capture available)'
                except ValueError:
                    text = 'RADAR malformed reply (full line in log)'
            elif event['kind'] == 'rx' and (text.startswith('{') or text.startswith('@CHASSIS ')):
                telemetry_count += 1
                now = time.monotonic()
                if now - last_telemetry < 1:
                    continue
                try:
                    frame = json.loads(text[8:].strip() if text.startswith('@CHASSIS') else text)
                    text = f'MAP {frame.get("kind", "frame")} t={frame.get("t_ms", "-")} session={frame.get("session", "-")} ({telemetry_count} frames; full JSON in log)'
                except (ValueError, AttributeError):
                    text = f'JSON RX ({len(text)} chars; full line in log)'
                last_telemetry = now
                telemetry_count = 0
            print(f'\n[{event["time"][11:23]} {event["kind"].upper()}] {text}', flush=True)


def console_command(bridge, line, port, baudrate):
    """Return false only for explicit quit; all hardware commands go via bridge."""
    line = line.strip()
    if not line:
        return True
    if line.lower() in ('quit', 'exit'):
        return False
    if line.lower() == 'ports':
        print(json.dumps(available_ports(), ensure_ascii=False), flush=True)
    elif line.split()[0].lower() == 'connect':
        parts = line.split()
        if len(parts) > 2:
            raise ValueError('Use connect COMx')
        target = parts[1] if len(parts) == 2 else port
        if not target:
            raise ValueError('Select a port: use ports, then connect COMx')
        bridge.connect(target, baudrate)
    elif line.lower() == 'disconnect':
        bridge.disconnect()
    elif line.lower() in ('stop', 'chassis stop', 'wheel stop'):
        bridge.request_stop()
    elif line.lower() == 'resume':
        bridge.resume()
    elif line.lower() == 'status':
        state = bridge.status()
        print(f'Connected={state["connected"]} {state["port"]} {state["baudrate"]} 8N1; stop={state["stop"]["status"]}; latched={state["stop_latched"]}', flush=True)
    else:
        bridge.send(line)
    return True


def main():
    parser = argparse.ArgumentParser(description='Local car shared serial console')
    parser.add_argument('--port', type=int, default=8765, help='Loopback HTTP port')
    parser.add_argument('--serial-port', default=None, help='Optional explicit CMD connect target; no default port')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--no-console', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    bridge = SerialBridge(root.parents[2] / '.embeddedskills')
    radar_bridge = RadarDirectBridge()
    try:
        server = make_server(bridge, root, args.port, radar_bridge=radar_bridge)
    except OSError as error:
        try:
            radar_bridge.close()
        finally:
            bridge.close()
        print(f'Cannot start local server: {error}\nUse the existing console or choose --port 8766.', flush=True)
        return 1
    url = server.console_origin + '/'
    print(f'Car dashboard: {url}\nCMD and webpage share one serial owner. Target: {args.serial_port or "not selected"} {args.baudrate} 8N1.', flush=True)
    print('No automatic connection. Commands: ports, connect COMx, status, stop, resume, disconnect, quit.\nCtrl+C requests CHASSIS STOP and keeps this window open. quit stops, waits up to 6 seconds, then closes.', flush=True)
    finished = threading.Event()
    http_thread = threading.Thread(target=server.serve_forever, name='car-http', daemon=True)
    output_thread = threading.Thread(target=print_events, args=(bridge, finished), daemon=True)
    http_thread.start()
    output_thread.start()
    if not args.no_browser:
        webbrowser.open(url)
    quitting = False
    input_generation = 0
    def stop_signal(signum, frame):
        nonlocal input_generation
        input_generation += 1
        if not quitting:
            bridge.request_stop()
        print('\nChassis stop requested; current input discarded. Press Enter for a fresh prompt; type quit to exit.', flush=True)
    old_sigint = signal.signal(signal.SIGINT, stop_signal)
    try:
        if args.no_console:
            while True:
                time.sleep(.2)
        else:
            while True:
                try:
                    generation = input_generation
                    line = input('car> ')
                    if generation != input_generation:
                        print('Interrupted input discarded; enter a new command.', flush=True)
                        continue
                    if not console_command(bridge, line, args.serial_port, args.baudrate):
                        break
                except KeyboardInterrupt:
                    bridge.request_stop()
                except EOFError:
                    break
                except (ValueError, OSError) as error:
                    print(f'Error: {error}', flush=True)
    finally:
        quitting = True
        try:
            radar_bridge.close()
        finally:
            try:
                bridge.close()
            finally:
                finished.set()
                server.shutdown()
                server.server_close()
                output_thread.join(1)
                signal.signal(signal.SIGINT, old_sigint)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
