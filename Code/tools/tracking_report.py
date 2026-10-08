"""Summarize frozen TRACK rows or long live streams. No hardware commands.

Example: python Code/tools/tracking_report.py segment.log --kind physical --rpm 6 --direction cw
The 128-row ring is a bounded window; firmware status statistics cover all new samples.
"""
import argparse
import json
import math
from pathlib import Path
import re
import zlib


def percentile(values, fraction=.95):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, math.ceil(len(ordered)*fraction)-1)]


def error_stats(rows):
    if not rows:
        return None
    errors = [r['e'] for r in rows]
    return {
        'rms_axis_px': [math.sqrt(sum(e[k]**2 for e in errors)/len(errors)) for k in range(2)],
        'rms_xy_px': math.sqrt(sum(sum(v*v for v in e) for e in errors)/len(errors)),
        'p95_axis_abs_px': [percentile([abs(e[k]) for e in errors]) for k in range(2)],
        'peak_axis_px': [max(abs(e[k]) for e in errors) for k in range(2)],
    }


def fields(line):
    values = {}
    for key, raw in re.findall(r'\b([a-z_0-9]+)=([^\s=]+)', line):
        try:
            values[key] = [float(v) for v in raw.split(',')] if ',' in raw else float(raw)
        except ValueError:
            values[key] = raw
    return values


def summarize(text, source='auto'):
    # A single file can contain live data AND a later ring export; never mix them.
    live = source == 'live' or (source == 'auto' and 'TRACK live ' in text and 'OK grab track trace_begin' not in text)
    rows, seen = [], set()
    firmware_stats = None
    firmware_timing = None
    expected, corrupt, ended = None, 0, False
    session, period, first_index = None, None, 0
    for line in text.splitlines():
        if live and 'OK grab track log_begin' in line:
            header = fields(line)
            if header.get('id') != session or (header.get('next') == 0 and rows):
                rows, seen, expected, corrupt, ended = [], set(), None, 0, False
                session = header.get('id')
                firmware_timing = None
                first_index = int(header.get('next', 0))
            period = header.get('period_ms')
        if live and 'OK grab track log_end' in line:
            end = fields(line)
            if session == end.get('id') and isinstance(end.get('count'), (int, float)):
                expected, ended = int(end['count']), True
        if 'OK grab track trace_begin' in line:
            if live:
                continue
            rows, seen = [], set()  # Report the last exported segment, not two exports mixed together.
            count = fields(line).get('count')
            expected = int(count) if isinstance(count, (int, float)) and 0 <= count <= 128 else None
            corrupt, ended = 0, False
        if 'OK grab track trace_end' in line:
            if not live:
                ended = True
        if 'OK grab track stats ' in line:
            firmware_stats = fields(line)
        if 'OK grab track timing ' in line:
            firmware_timing = fields(line)
        prefix = 'TRACK live ' if live else 'TRACK row '
        if prefix not in line:
            continue
        raw = line[line.index(prefix):].strip()
        row = fields(raw)
        if live:
            if session is None:
                session = row.get('id')
            if row.get('id') != session:
                continue
        if 'data' in row:
            match = re.fullmatch(r'(.*) crc=([\da-fA-F]{8})', raw)
            data = row['data']
            if not match or zlib.crc32(match[1].encode('ascii', errors='replace')) != int(match[2], 16) or not isinstance(data, list) or len(data) != 19:
                corrupt += 1
                continue
            row.update(e=data[:2], target=data[2:4], actual=data[4:7], cmd=data[7:10],
                       x=data[10], forward=data[11], left=data[12], room=data[13],
                       batch=data[14], age=data[15], request_age=data[16], sent_age=data[17], flags=data[18])
        io_match = re.search(r'\bio=([^\s]+)', raw)
        if io_match:
            try:
                io = [int(v) for v in io_match[1].split(',')]
            except ValueError:
                corrupt += 1
                continue
            if len(io) not in (26, 30, 34):
                corrupt += 1
                continue
            row.update(sent_rpm=io[:4], wheel_raw=io[4:8], wheel_age=io[8:12],
                       x_sent_pulses=io[12], x_sent_rpm=io[13], x_sent_age=io[14], x_raw=io[15],
                       x_action=io[16], bus_stage=io[17], bus_error=io[18], locked=io[19],
                       io_valid=io[20], motion_seq=io[21], stop_seq=io[22],
                       x_request_pulses=io[23], x_request_rpm=io[24], x_request_age=io[25])
            if len(io) >= 30:
                row['feedback_rpm'] = io[26:30]
            if len(io) >= 34:
                row['speed_age'] = io[30:34]
        scalars = ('t', 'i', 'rx', 'seq', 'flags', 'x', 'forward', 'left', 'room')
        vectors = (('e', 2), ('target', 2), ('actual', 3), ('cmd', 3))
        if not all(isinstance(row.get(k), (int, float)) and math.isfinite(row[k]) for k in scalars) or not all(isinstance(row.get(k), list) and len(row[k]) == size and all(math.isfinite(v) for v in row[k]) for k, size in vectors):
            corrupt += 1
            continue
        identity = (row['t'], row['i'])
        if identity not in seen:
            rows.append(row)
            seen.add(identity)
    rows.sort(key=lambda r: r['t'])
    samples, identities = [], set()
    for row in rows:
        key = (row.get('seq'), row.get('rx'))
        if int(row.get('flags', 0)) & 1 and key not in identities:
            samples.append(row)
            identities.add(key)
    motion = [r for r in rows if int(r.get('flags', 0)) & 2]
    tracking_samples = [r for r in samples if int(r.get('flags', 0)) & 2]
    stable = [r for r in tracking_samples if int(r.get('flags', 0)) & 4]
    startup = None
    if motion:
        first_stable = stable[0]['t'] if stable else None
        transient = [r for r in tracking_samples if first_stable is None or r['t'] < first_stable]
        startup = {'first_stable_ms': first_stable-motion[0]['t'] if first_stable is not None else None,
                   'peak_axis_px': error_stats(transient)['peak_axis_px'] if transient else None}
    intervals = [b['t']-a['t'] for a, b in zip(rows, rows[1:])]
    receives = sorted({r['rx'] for r in samples if 'rx' in r})
    receive_intervals = [b-a for a, b in zip(receives, receives[1:])]
    # Deduplicate the same completed batch reported by several control updates.
    sends = sorted({r['t']-r['sent_age'] for r in motion if r.get('sent_age', 65535) < 65534})
    batch_intervals = [b-a for a, b in zip(sends, sends[1:])]
    displacement = None
    if rows:
        displacement = {key: rows[-1][key]-rows[0][key] for key in ('forward', 'left', 'x')}
    present = {int(r['i']) for r in rows}
    missing = [i for i in range(first_index, expected) if i not in present] if expected is not None else None
    io_rows = [r for r in rows if 'io_valid' in r]
    wheel_sent = [r for r in io_rows if r['io_valid'] & 16]
    x_sent = [r for r in io_rows if r['io_valid'] & 32]
    return {
        'records': len(rows), 'new_valid_samples': len(samples),
        'expected_records': expected, 'corrupt_rows': corrupt,
        'missing_records': missing,
        'trace_complete': expected is not None and ended and present == set(range(first_index, expected)),
        'source_type': 'live' if live else 'frozen', 'stream_id': session, 'stream_period_ms': period,
        'sent_wheel_rpm_peak': [max(abs(r['sent_rpm'][k]) for r in wheel_sent) for k in range(4)] if wheel_sent else None,
        'wheel_feedback_age_p95_ms': [percentile([r['wheel_age'][k] for r in io_rows if r['io_valid'] & (1 << k)]) for k in range(4)] if io_rows else None,
        'x_sent_rpm_peak': max((r['x_sent_rpm'] for r in x_sent), default=None),
        'x_sent_age_p95_ms': percentile([r['x_sent_age'] for r in x_sent]),
        'locked_records': sum(bool(r['locked']) for r in io_rows),
        'tracking_samples': len(tracking_samples), 'stable_samples': len(stable),
        'window_ms': rows[-1]['t']-rows[0]['t'] if rows else 0,
        'all_error': error_stats(samples), 'tracking_error': error_stats(tracking_samples),
        'stable_error': error_stats(stable), 'startup': startup,
        'receive_interval_p95_ms': percentile(receive_intervals),
        'receive_hz': 1000*len(receive_intervals)/(receives[-1]-receives[0]) if receive_intervals else None,
        'control_interval_p95_ms': firmware_timing.get('control_p95') if firmware_timing and firmware_timing.get('updates', 0) else (percentile(intervals) if not live else None),
        'actual_batch_p95_ms': firmware_timing.get('batch_p95') if firmware_timing and firmware_timing.get('batch_samples', 0) else None,
        'observed_batch_gap_p95_ms': percentile(batch_intervals),
        'timing_source': 'device_all_events' if firmware_timing else 'frozen_rows' if not live else 'unavailable_sparse_stream',
        'firmware_timing': firmware_timing,
        'timing_sample_sufficient': bool(firmware_timing and firmware_timing.get('updates',0)>=20 and firmware_timing.get('batch_samples',0)>=20),
        'visual_receive_to_control_p95_ms': firmware_timing.get('visual_delay_p95') if firmware_timing and firmware_timing.get('rx_samples',0) else None,
        'prediction_interval_p95_ms': firmware_timing.get('prediction_p95') if firmware_timing and firmware_timing.get('timer',0) else None,
        'x_feedback_age_p95_ms': percentile([r['age'] for r in rows if 'age' in r]),
        'actual_speed_peak_mm_s': [max(abs(r['actual'][k]) for r in rows) for k in range(3)] if rows else None,
        'command_speed_peak_mm_s': [max(abs(r['cmd'][k]) for r in rows) for k in range(3)] if rows else None,
        'displacement_mm': displacement, 'minimum_approach_room_mm': min(r['room'] for r in rows) if rows else None,
        'firmware_full_segment_stats': firmware_stats,
        'scope': 'live stream; missing rows are not recovered from the MCU ring' if live else 'frozen ring window; receive timing is not camera exposure latency',
    }


def decode_log(data):
    """Accept raw serial text, bridge JSONL RX events or legacy capture JSON."""
    text = data.decode('utf-16' if data.startswith((b'\xff\xfe', b'\xfe\xff')) else 'utf-8-sig', errors='replace')
    if not text.lstrip().startswith('{'):
        return text
    try:
        document = json.loads(text)
    except json.JSONDecodeError:
        document = None
    if isinstance(document, dict) and 'details' in document:
        return document.get('details', {}).get('rx', '')
    lines = []
    for line in text.splitlines():
        event = json.loads(line)
        if isinstance(event, dict) and event.get('kind') == 'rx':
            lines.append(event.get('text', ''))
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--kind', choices=('observation', 'simulation', 'physical'), default='observation')
    parser.add_argument('--rpm', type=float)
    parser.add_argument('--direction', default='unspecified')
    parser.add_argument('--source', choices=('auto', 'live', 'frozen'), default='auto')
    args = parser.parse_args()
    text = decode_log(args.log.read_bytes())
    result = summarize(text, source=args.source)
    result.update({'kind': args.kind, 'nominal_rpm': args.rpm, 'direction': args.direction, 'source': str(args.log)})
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
