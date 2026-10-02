"""One serial owner for the local browser and interactive console.

Only the worker opens, reads, writes, and closes pyserial. Requests and events
cross the condition-protected queue; stop never waits for an ordinary RX reply.
"""
from collections import deque
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import secrets
import threading
import time


STOP_REPLY = re.compile(
    r'OK chassis stop id=(\d+) token=(\d+) requested_ms=(\d+) '
    r'tx_complete=([01]) wheels_stopped=([01]) reason=([A-Za-z0-9_]+)')
READ_COMMAND = re.compile(
    r'(?:info|help|bus status|console status|(?:state|position|config) (?:base|z|x|all)|'
    r'(?:vision|camera|material|imu|qr|grab) status|qr read|grab get [a-z][a-z0-9_]*|'
    r'chassis (?:task|status|snapshot|feedback|stop-status|route status)|map status)')
PREPARATION_COMMAND = re.compile(
    r'(?:chassis (?:units 65536|feedback (?:0|on)|'
    r'origin 2250 150 90|stream (?:off|on [1-9]\d{0,9}))|imu verify 5)')
CANCEL_COMMAND = re.compile(r'(?:bus recover|chassis route cancel|imu cal cancel|stop (?:all|base|z|x)|(?:vision|material|camera|grab) stop)')
PRIORITY_STOP = re.compile(r'(?:chassis route cancel|imu cal cancel|stop (?:all|base|z|x)|(?:vision|material|camera|grab) stop)')
TERMINAL_STOP = {'confirmed', 'timeout', 'disconnected'}


def open_serial(**options):
    try:
        import serial
    except ImportError as error:
        raise RuntimeError('pyserial is missing. Install it with: python -m pip install pyserial') from error
    return serial.Serial(**options)


class SerialBridge:
    def __init__(self, state_dir, serial_factory=open_serial, *, stop_timeout=6.0,
                 command_interval=.12, event_limit=2000, stop_interval=2.0):
        self.state_dir = Path(state_dir)
        self.state_dir.mkdir(parents=True, exist_ok=True)
        self.config_path = self.state_dir / 'local-car.json'
        self.config = {'profile':'', 'units_per_rev':None, 'directions_confirmed':False}
        if self.config_path.exists():
            try:
                self.config = self._config_values(json.loads(self.config_path.read_text(encoding='utf-8')))
            except (OSError, ValueError, TypeError):
                pass
        self.cv = threading.Condition(threading.RLock())
        self.events = deque(maxlen=event_limit)
        self.last_event_id = 0
        self.request_id = 0
        self.commands = deque()
        self.priority_commands = deque()
        self.controls = deque()
        self.factory = serial_factory
        self.serial = None
        self.connected = False
        self.port = ''
        self.baudrate = None
        self.stop_latched = False
        self.stop = {'status':'idle', 'token':0, 'tx_complete':False, 'wheels_stopped':False}
        self.stop_timeout = stop_timeout
        self.stop_interval = stop_interval
        self.stop_reply_active = False
        self.command_interval = command_interval
        self.next_command_at = 0
        self.stop_attempts = 0
        self.stop_started_at = 0
        self.stop_next_at = 0
        self.disconnecting = False
        self.preparation_state = {'active':False, 'owner':None}
        self.preparation_deadline = 0
        self.shutdown_requested = False
        self.disconnect_waiters = []
        self.rx_buffer = bytearray()
        self.rx_dropping = False
        self.auth_token = secrets.token_urlsafe(32)
        log_dir = self.state_dir / 'logs' / 'serial'
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d-%H%M%S') + '-' + secrets.token_hex(3)
        self.event_file = (log_dir / (stamp+'.jsonl')).open('a', encoding='utf-8', buffering=1)
        self.rx_file = (log_dir / (stamp+'-rx.bin')).open('ab', buffering=0)
        self.tx_file = (log_dir / (stamp+'-tx.bin')).open('ab', buffering=0)
        self.worker = threading.Thread(target=self._run, name='car-serial-owner', daemon=True)
        self.worker.start()

    @staticmethod
    def _config_values(values):
        if not isinstance(values, dict) or set(values) - {'profile','units_per_rev','directions_confirmed'}:
            raise ValueError('Unknown configuration fields')
        result = {'profile':'', 'units_per_rev':None, 'directions_confirmed':False, **values}
        if result['profile'] not in ('', 'none', 'receive'):
            raise ValueError('profile must be receive, none, or empty')
        units = result['units_per_rev']
        if units is not None and (type(units) is not int or units != 65536):
            raise ValueError('units_per_rev must be null (unconfirmed) or confirmed 65536')
        if type(result['directions_confirmed']) is not bool:
            raise ValueError('directions_confirmed must be boolean')
        return result

    def save_config(self, values):
        with self.cv:
            if not isinstance(values, dict):
                raise ValueError('Configuration must be an object')
            config = self._config_values({**self.config, **values})
            temp = self.config_path.with_suffix('.tmp')
            temp.write_text(json.dumps(config, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
            temp.replace(self.config_path)
            self.config = config
            self._event('system', 'Local confirmed configuration saved; no driver settings sent')
            return dict(config)

    def _event(self, kind, text, **fields):
        with self.cv:
            self.last_event_id += 1
            event = {'id':self.last_event_id, 'kind':kind, 'text':text,
                     'time':datetime.now(timezone.utc).isoformat(), **fields}
            self.events.append(event)
            self.event_file.write(json.dumps(event, ensure_ascii=False)+'\n')
            self.cv.notify_all()
            return event

    def get_events(self, after=0, timeout=1):
        with self.cv:
            self.cv.wait_for(lambda: self.last_event_id > after or self.shutdown_requested, timeout=min(timeout, 1))
            return {'events':[dict(e) for e in self.events if e['id'] > after],
                    'last_event_id':self.last_event_id,
                    'gap':bool(self.events and after < self.events[0]['id'] - 1)}

    def status(self):
        with self.cv:
            return {'token':self.auth_token, 'connected':self.connected,
                    'port':self.port, 'baudrate':self.baudrate, 'disconnecting':self.disconnecting,
                    'stop_latched':self.stop_latched, 'stop':dict(self.stop),
                    'preparation':dict(self.preparation_state),
                    'config':dict(self.config), 'last_event_id':self.last_event_id}

    def _clear_preparation(self, reason):
        if self.preparation_state['active']:
            owner = self.preparation_state['owner']
            self.preparation_state = {'active':False, 'owner':None}
            self._cancel_commands('preparation ended: '+reason, include_priority=False)
            self._event('preparation', reason, active=False, owner=owner)

    def preparation(self, active, owner):
        if type(active) is not bool or not isinstance(owner, str) or not 1 <= len(owner) <= 128:
            raise ValueError('Preparation requires boolean active and nonempty owner (max 128 characters)')
        with self.cv:
            if self.preparation_state['active'] and self.preparation_state['owner'] != owner:
                raise ValueError('Another webpage owns the current preparation')
            if active:
                if not self.connected or self.disconnecting or self.stop_latched:
                    raise ValueError('Connect and explicitly resume before preparation')
                self._cancel_commands('cancelled by preparation', include_priority=False)
                self.preparation_state = {'active':True, 'owner':owner}
                self.preparation_deadline = time.monotonic() + 150
                self._event('preparation', 'Preparation started; motion commands blocked', active=True, owner=owner)
            else:
                self._clear_preparation('Preparation released')
            return dict(self.preparation_state)

    def connect(self, port, baudrate):
        if not isinstance(port, str) or not re.fullmatch(r'COM[1-9]\d*', port, re.I):
            raise ValueError('Specify a Windows COM port explicitly, for example COM7')
        if type(baudrate) is not int or not 1200 <= baudrate <= 2000000:
            raise ValueError('Specify a valid integer baudrate explicitly')
        done, result = threading.Event(), {}
        with self.cv:
            if self.disconnecting or self.shutdown_requested:
                raise ValueError('Disconnect/quit is in progress')
            self.controls.append(('connect', (port.upper(), baudrate), done, result))
            self.cv.notify_all()
        if not done.wait(4):
            raise ValueError('Serial open is still pending; check status before retrying')
        if 'error' in result:
            raise ValueError(result['error'])
        return self.status()

    def send(self, command, owner=None):
        if not isinstance(command, str) or not command or len(command) > 63 or any(ord(c) < 32 or ord(c) > 126 for c in command):
            raise ValueError('Use one ASCII command, 1-63 printable characters, without CR/LF')
        command = command.strip()
        if not command:
            raise ValueError('Empty command')
        if command in ('chassis stop', 'wheel stop'):
            return {'queued':False, 'stop':self.request_stop()}
        if command.startswith('chassis stop '):
            raise ValueError('Use /api/stop; stop tokens are owned by this server')
        with self.cv:
            if not self.connected or self.disconnecting:
                raise ValueError('Serial is disconnected or closing; connect explicitly first')
            if self.stop_latched and not READ_COMMAND.fullmatch(command) and not PRIORITY_STOP.fullmatch(command) and command != 'bus recover':
                raise ValueError('Stop is latched; use resume explicitly before ordinary commands')
            if owner is not None and (not self.preparation_state['active'] or
                                      owner != self.preparation_state['owner']):
                raise ValueError('Preparation lease has ended or belongs to another page')
            if self.preparation_state['active'] and CANCEL_COMMAND.fullmatch(command):
                self._clear_preparation('Preparation cancelled by '+command)
            if self.preparation_state['active'] and READ_COMMAND.fullmatch(command) and owner != self.preparation_state['owner']:
                raise ValueError('Another webpage owns preparation replies; cancel preparation before querying')
            if self.preparation_state['active'] and not READ_COMMAND.fullmatch(command):
                if owner != self.preparation_state['owner'] or not PREPARATION_COMMAND.fullmatch(command):
                    raise ValueError('Preparation is active; motion and unrelated mutations are blocked')
            if PRIORITY_STOP.fullmatch(command):
                self._cancel_commands('cancelled by '+command, include_priority=False)
            if len(self.commands) + len(self.priority_commands) >= 100:
                raise ValueError('Command queue is full')
            self.request_id += 1
            if PRIORITY_STOP.fullmatch(command):
                self.priority_commands.append((self.request_id, command))
            else:
                self.commands.append((self.request_id, command))
            self.cv.notify_all()
            return {'queued':True, 'request_id':self.request_id}

    def _cancel_commands(self, reason, include_priority=True):
        queues = (self.commands, self.priority_commands) if include_priority else (self.commands,)
        for commands in queues:
            while commands:
                request_id, command = commands.popleft()
                self._event('system', f'Cancelled: {command}', request_id=request_id, error=reason)

    def _stop_event(self):
        phase = self.stop['status']
        if phase == 'confirmed':
            text = 'STOP: wheel feedback confirms stopped'
        elif phase in ('timeout', 'disconnected'):
            text = f'STOP: {phase}; motor_TX_complete={int(self.stop.get("tx_complete", False))}; wheel stop not confirmed'
        elif self.stop.get('tx_complete'):
            text = 'STOP: motor stop TX complete; wheel stop not confirmed'
        elif phase == 'accepted':
            text = 'STOP: MCU accepted request; motor stop TX not complete'
        elif phase == 'sent':
            text = 'STOP: request sent; waiting for MCU reply'
        else:
            text = 'STOP: '+phase+'; wheel stop not confirmed'
        fields = dict(self.stop)
        fields['stop_id'] = fields.pop('id', 0)
        self._event('stop', text, latched=self.stop_latched, **fields)

    def request_stop(self):
        with self.cv:
            self._clear_preparation('Preparation cancelled by stop')
            self._cancel_commands('cancelled by stop')
            if self.stop_latched and self.stop['status'] not in TERMINAL_STOP | {'idle'}:
                return dict(self.stop)
            self.stop_latched = True
            self.stop = {'status':'requested' if self.connected else 'disconnected',
                         'token':secrets.randbelow(0xFFFFFFFF)+1,
                         'tx_complete':False, 'wheels_stopped':False}
            self.stop_reply_active = self.connected
            self.stop_attempts = 0
            self.stop_started_at = time.monotonic()
            self.stop_next_at = self.stop_started_at
            self._stop_event()
            self.cv.notify_all()
            return dict(self.stop)

    def resume(self):
        with self.cv:
            if self.disconnecting:
                raise ValueError('Disconnect is in progress')
            if self.stop_latched and self.stop['status'] not in TERMINAL_STOP:
                raise ValueError('Stop request is still pending; wait for its result')
            self.stop_latched = False
            self.stop_reply_active = False
            fields = dict(self.stop)
            fields['stop_id'] = fields.pop('id', 0)
            self._event('stop', 'Normal command entry resumed; no motion sent', latched=False, **fields)
            return self.status()

    def disconnect(self):
        done = threading.Event()
        with self.cv:
            if not self.disconnecting:
                self.disconnecting = True
                self._event('connection', '正在停车并断开', connected=self.connected, disconnecting=True)
            self._clear_preparation('Preparation cancelled by disconnect')
            self._cancel_commands('cancelled by disconnect')
            if self.connected and (not self.stop_latched or self.stop['status'] not in TERMINAL_STOP):
                self.request_stop()
            self.disconnect_waiters.append(done)
            self.cv.notify_all()
        if not done.wait(self.stop_timeout + 1):
            raise ValueError('Serial owner is still closing; check status')
        return self.status()

    def close(self):
        if not self.worker.is_alive():
            return
        self.disconnect()
        with self.cv:
            self.shutdown_requested = True
            self.cv.notify_all()
        self.worker.join(2)
        if self.worker.is_alive():
            raise RuntimeError('Serial owner did not exit')
        for stream in (self.event_file, self.rx_file, self.tx_file):
            stream.close()

    def _write(self, data, text, request_id=None):
        written = self.serial.write(data)
        self.tx_file.write(data[:written or 0])
        if written != len(data):
            raise IOError('Incomplete serial write; ordinary commands cancelled')
        fields = {'hex':data.hex()}
        if request_id is not None:
            fields['request_id'] = request_id
        self._event('tx', text, **fields)

    def _handle_rx(self, data):
        self.rx_file.write(data)
        # Raw chunks preserve all bytes and timing, including invalid/partial lines.
        self._event('raw', '', hex=data.hex())
        for byte in data:
            if byte == 10:
                if not self.rx_dropping:
                    line = self.rx_buffer.rstrip(b'\r').decode('utf-8', errors='replace')
                    self._event('rx', line)
                    match = STOP_REPLY.fullmatch(line)
                    if match and self.stop_latched and self.stop_reply_active and self.stop['status'] != 'confirmed':
                        ident, token, requested, complete, stopped, reason = match.groups()
                        if int(token) == self.stop['token']:
                            self.stop.update(id=int(ident), requested_ms=int(requested),
                                             tx_complete=complete == '1', wheels_stopped=stopped == '1', reason=reason)
                            # Late evidence updates the result without restarting timed-out traffic.
                            self.stop['status'] = ('confirmed' if complete == stopped == '1' else
                                                   'timeout' if self.stop['status'] == 'timeout' else 'accepted')
                            self._stop_event()
                self.rx_buffer.clear()
                self.rx_dropping = False
            elif not self.rx_dropping:
                self.rx_buffer.append(byte)
                if len(self.rx_buffer) > 16384:
                    self.rx_buffer.clear()
                    self.rx_dropping = True
                    self._event('system', 'Oversize serial line discarded')

    def _close_serial(self, error=None):
        self.stop_reply_active = False
        self._clear_preparation('Preparation cancelled by serial disconnect')
        self._cancel_commands(error or 'serial disconnected')
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception as close_error:
                self._event('system', f'Serial close error: {close_error}')
            self.serial = None
        self.connected = False
        self.rx_buffer.clear()
        self.rx_dropping = False
        if self.stop['status'] not in TERMINAL_STOP | {'idle'}:
            self.stop['status'] = 'disconnected'
            self._stop_event()
        self._event('connection', error or 'Serial disconnected', connected=False)

    def _run(self):
        while True:
            with self.cv:
                if self.shutdown_requested:
                    return
                if self.controls:
                    kind, args, done, result = self.controls.popleft()
                    try:
                        if self.connected:
                            raise ValueError('Serial is already connected; disconnect before changing it')
                        if self.disconnecting:
                            raise ValueError('Connect cancelled by disconnect')
                        port, baudrate = args
                        self.serial = self.factory(port=port, baudrate=baudrate, bytesize=8,
                                                   parity='N', stopbits=1, timeout=0,
                                                   write_timeout=.15, xonxoff=False,
                                                   rtscts=False, dsrdtr=False)
                        self.connected, self.port, self.baudrate = True, port, baudrate
                        self.next_command_at = 0
                        self._event('connection', f'Connected {port} {baudrate} 8N1', connected=True, port=port, baudrate=baudrate)
                    except Exception as error:
                        result['error'] = f'Cannot open serial: {error}. If busy/access denied, close the other serial program.'
                        self._event('connection', result['error'], connected=self.connected)
                    finally:
                        done.set()
                now = time.monotonic()
                if self.preparation_state['active'] and now >= self.preparation_deadline:
                    self._clear_preparation('Preparation expired after 150 seconds')
                if self.connected:
                    try:
                        pending_stop = self.stop_latched and self.stop['status'] not in TERMINAL_STOP | {'idle'}
                        if pending_stop and now - self.stop_started_at >= self.stop_timeout:
                            self.stop['status'] = 'timeout'
                            self._stop_event()
                            pending_stop = False
                        if pending_stop and now >= self.stop_next_at:
                            if self.stop_attempts == 0:
                                self._write(b'\x03', '<Ctrl+C 0x03>')
                            if self.stop_attempts < 3 and self.stop['status'] != 'accepted':
                                command = f'chassis stop {self.stop["token"]}'
                                self._write((command+'\r\n').encode('ascii'), command)
                                if self.stop['status'] == 'requested':
                                    self.stop['status'] = 'sent'
                                    self._stop_event()
                                self.stop_attempts += 1
                                self.stop_next_at = now + self.stop_interval
                            else:
                                self._write(b'chassis stop-status\r\n', 'chassis stop-status')
                                self.stop_next_at = now + self.stop_interval
                        elif not self.disconnecting and (self.priority_commands or (not pending_stop and self.commands)) and now >= self.next_command_at:
                            request_id, command = (self.priority_commands or self.commands).popleft()
                            try:
                                self._write((command+'\r\n').encode('ascii'), command, request_id)
                            except Exception as error:
                                self._event('system', str(error), request_id=request_id, error=str(error))
                                raise
                            self.next_command_at = time.monotonic() + self.command_interval
                        # Bounded nonblocking read follows the priority check on every iteration.
                        waiting = self.serial.in_waiting
                        if waiting:
                            self._handle_rx(self.serial.read(min(waiting, 8192)))
                    except Exception as error:
                        self._close_serial(f'Serial I/O failed: {error}')
                if self.disconnecting and (not self.connected or self.stop['status'] in TERMINAL_STOP):
                    if self.connected:
                        self._close_serial()
                    self.disconnecting = False
                    for done in self.disconnect_waiters:
                        done.set()
                    self.disconnect_waiters.clear()
                self.cv.wait(.005)
