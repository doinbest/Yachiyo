"""Bounded COM validation: stop/recover/query only; never starts motion."""
import argparse
import datetime
import re
import time
import serial

parser = argparse.ArgumentParser()
parser.add_argument('--port', required=True)
parser.add_argument('--log', required=True)
parser.add_argument('--trials', type=int, default=3)
args = parser.parse_args()
if not 1 <= args.trials <= 5:
    parser.error('trials must be 1..5')
with serial.Serial(args.port, 115200, bytesize=8, parity='N', stopbits=1,
                   timeout=.02) as port, open(args.log, 'w', encoding='utf-8') as log:
    pending = bytearray()
    def record(kind, text):
        log.write(f'{datetime.datetime.now().isoformat(timespec="milliseconds")} {kind} {text}\n')
        log.flush()
    def send(command):
        record('TX', command)
        port.write((command+'\r\n').encode('ascii'))
        port.flush()
    def read(seconds, marker):
        deadline = time.monotonic()+seconds
        rows = []
        while time.monotonic()<deadline:
            pending.extend(port.read(port.in_waiting or 1))
            while b'\n' in pending:
                raw, _, rest = pending.partition(b'\n')
                pending[:] = rest
                row = raw.decode('utf-8', errors='replace').rstrip('\r')
                record('RX', row)
                rows.append(row)
                if 'ERR ' in row:
                    raise RuntimeError(row)
                if (marker(row) if callable(marker) else marker in row):
                    return '\n'.join(rows)
        raise TimeoutError(f'COM reply missing: {marker}')
    def request(command, marker, seconds=4):
        send(command)
        return read(seconds, marker)
    request('grab status', 'OK grab state=')
    before = request('grab get x_ppm', 'OK grab config key=x_ppm')
    before_ppm = re.search(r'value=([\d.]+)', before)[1]
    for trial in range(args.trials):
        started = time.monotonic()
        send('bus recover')
        result = read(15, 'OK bus recover complete')
        for flag in ('arm_ready=1', 'chassis_ready=1', 'chassis_cache=cleared',
                     'wheels_stopped=1', 'tasks_resumed=0'):
            if flag not in result:
                raise RuntimeError(f'Recovery incomplete: {result}')
        elapsed = time.monotonic()-started
        task = request('chassis task', 'OK chassis task ')
        if 'locked=0 ' not in task or 'error=0 ' not in task:
            raise RuntimeError(task)
        request('chassis feedback 0', 'OK chassis request accepted')
        request('chassis feedback on', 'OK chassis request accepted')
        # Explicit stop monitor collects fresh four-wheel speed/state evidence.
        token = 6200+trial
        send(f'chassis stop {token}')
        stopped = read(8, lambda row: f'token={token} ' in row and
                       'tx_complete=1 wheels_stopped=1' in row)
        if 'reason=stopped' not in stopped:
            raise RuntimeError(stopped)
        wheels = request('chassis feedback', 'OK wheel=4 ')
        rows = re.findall(r'OK wheel=(\d) raw_rpm=(-?\d+) raw_units=(-?\d+) flags=0x([0-9A-Fa-f]+) valid=(\d),(\d),(\d)', wheels)
        if len(rows)!=4 or any(int(r[1])!=0 or r[4:7]!=('1','1','1') for r in rows):
            raise RuntimeError(f'Fresh stopped feedback missing: {wheels}')
        task = request('chassis task', 'OK chassis task ')
        if 'locked=0 ' not in task or 'error=0 ' not in task:
            raise RuntimeError(task)
        record('RESULT', f'trial={trial+1} recovery_s={elapsed:.3f} locked=0 all_rpm=0 stop_confirmed=1')
        print(f'Trial {trial+1}: recovery {elapsed:.3f}s; unlocked, all four actual RPM=0.', flush=True)
    after = request('grab get x_ppm', 'OK grab config key=x_ppm')
    if re.search(r'value=([\d.]+)', after)[1] != before_ppm:
        raise RuntimeError('Recovery changed x_ppm')
    request('grab status', 'OK grab state=')
    request('bus status', 'OK bus recovery ')
    request('chassis feedback off', 'OK chassis request accepted')
    print(f'{args.trials}/{args.trials} stop/zero-cache recovery checks PASS; no motion started.', flush=True)
