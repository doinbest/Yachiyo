"""Read persisted serial captures; exports never issue hardware commands."""
import importlib.util
import json
from pathlib import Path
import re
import tempfile

_spec = importlib.util.spec_from_file_location(
    'tracking_report_archive', Path(__file__).resolve().parents[2] / 'tools' / 'tracking_report.py')
_report = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_report)
NAME = re.compile(r'\d{8}-\d{6}-[0-9a-f]{6}')
BEGIN = re.compile(rb'OK grab track log_begin id=(\d+) period_ms=(\d+) next=(\d+)\r?\n$')
END = re.compile(rb'OK grab track log_end id=(\d+) count=(\d+)\r?\n$')
PIX_BEGIN = re.compile(rb'OK camera log_begin id=(\d+) next=(\d+) [^\r\n]*\r?\n$')
PIX_END = re.compile(rb'OK camera log_end id=(\d+) count=(\d+) [^\r\n]*\r?\n$')


class LogArchive:
    def __init__(self, directory):
        self.directory = Path(directory)

    def _path(self, name, suffix):
        if not isinstance(name, str) or not NAME.fullmatch(name):
            raise ValueError('Invalid serial log name')
        path = self.directory / (name + suffix)
        if not path.is_file():
            raise ValueError('Serial log not found')
        return path

    def list(self):
        rows = []
        for path in self.directory.glob('*-rx.bin'):
            name = path.name[:-7]
            if NAME.fullmatch(name):
                rows.append({'name': name, 'rx_bytes': path.stat().st_size,
                             'console_available': (self.directory / (name + '.jsonl')).is_file()})
        return sorted(rows, key=lambda row: row['name'], reverse=True)

    def _spans(self, name):
        path = self._path(name, '-rx.bin')
        spans, current, offset, configs = [], None, 0, {}
        # Freeze the byte boundary, even when the serial owner keeps appending.
        with path.open('rb') as stream:
            size = path.stat().st_size
            while offset < size:
                line = stream.readline(size - offset)
                pixel_begin, pixel_end = PIX_BEGIN.search(line), PIX_END.search(line)
                begin, end = BEGIN.search(line), END.search(line)
                config = re.search(rb'OK grab track config key=([a-z_]+) value=([^\r\n ]+)(?: profile=(predict|visual|feedback))?\r?\n$', line)
                if config:
                    configs[(config[3] or b'predict', config[1])] = line
                if pixel_begin or (begin and not (current and current['kind']=='pixels')):
                    header=pixel_begin or begin
                    kind='pixels' if pixel_begin else 'live'
                    next_index=int(header[2] if pixel_begin else header[3])
                else:
                    header=None
                if header and (current is None or current['kind']!=kind or current['id']!=int(header[1]) or next_index==0):
                    if current:
                        current['end'] = offset
                    current = {'id': int(header[1]), 'kind':kind, 'start': offset, 'end': size, 'closed': False, 'begin_received': True,
                               'context': list(configs.values())}
                    spans.append(current)
                pixel = re.search(rb'PIX id=(\d+)\b', line)
                live = pixel or re.search(rb'TRACK live id=(\d+)\b', line)
                row_kind='pixels' if pixel else 'live'
                if live and not (current and current['kind']=='pixels' and not pixel) and (current is None or current['kind']!=row_kind or current['id'] != int(live[1])):
                    if current:
                        current['end'] = offset
                    current = {'id': int(live[1]), 'kind':row_kind, 'start': offset, 'end': size, 'closed': False,
                               'begin_received': False, 'context': list(configs.values())}
                    spans.append(current)
                tail=pixel_end if current and current['kind']=='pixels' else end
                if tail and current and current['id'] == int(tail[1]):
                    current['end'] = offset + len(line)
                    current['closed'] = True
                    current = None
                offset += len(line)
        return spans

    def tracking(self, name):
        spans = self._spans(name)
        rows = []
        with self._path(name, '-rx.bin').open('rb') as stream:
            for index, span in enumerate(spans):
                stream.seek(span['start'])
                text = stream.read(span['end'] - span['start']).decode('utf-8', errors='replace')
                if span['kind']=='pixels':
                    result=_report.summarize_pixels(text)
                    motor=_report.summarize(text,source='live')
                    result['motor_records']=motor['records']
                    result['motor_summary']={key:motor[key] for key in ('command_speed_peak_mm_s','actual_speed_peak_mm_s','sent_wheel_rpm_peak','firmware_full_segment_stats','firmware_timing')}
                    missing_count,missing_preview=result['missing_count'],result['missing_preview']
                else:
                    result = _report.summarize(text, source='live')
                    missing = result.pop('missing_records')
                    missing_count,missing_preview=(len(missing),missing[:20]) if missing is not None else (None,[])
                result.update(index=index, closed=span['closed'], begin_received=span['begin_received'],
                              trace_complete=result['trace_complete'] and span['begin_received'] and span['closed'],
                              missing_count=missing_count, missing_preview=missing_preview,
                              parameter_keys=len(span['context']))
                rows.append(result)
        return rows

    def export(self, name, kind, segment=None):
        """Return a frozen disk stream. RAM usage is independent of capture length."""
        if kind not in ('rx', 'console', 'tracking'):
            raise ValueError('Unknown export format')
        path = self._path(name, '.jsonl' if kind == 'console' else '-rx.bin')
        start, end, context = 0, path.stat().st_size, []
        if kind == 'tracking':
            spans = self._spans(name)
            if type(segment) is not int or not 0 <= segment < len(spans):
                raise ValueError('Tracking segment not found')
            span = spans[segment]
            start, end, context = span['start'], span['end'], span['context']
        output = tempfile.TemporaryFile()
        try:
            output.write(b'\xef\xbb\xbf')
            if context:
                output.write('# 记录开启前最近一次设备参数读回；原始追踪数据如下。\r\n'.encode('utf-8'))
                for line in context:
                    output.write(line)
            with path.open('rb') as stream:
                stream.seek(start)
                remaining = end - start
                while remaining:
                    chunk = stream.readline(remaining) if kind == 'console' else stream.read(min(65536, remaining))
                    if not chunk:
                        break
                    remaining -= len(chunk)
                    if kind == 'console':
                        try:
                            event = json.loads(chunk)
                        except (ValueError, UnicodeDecodeError):
                            continue  # A currently appended, partial JSON event is not committed.
                        if event.get('kind') == 'raw':
                            continue
                        line = f"[{event.get('time', '')}] {event.get('kind', 'system').upper()} {event.get('text', '')}\r\n"
                        output.write(line.encode('utf-8'))
                    else:
                        output.write(chunk)  # Preserve damaged lines and the partial final line too.
            output.seek(0)
            return output
        except Exception:
            output.close()
            raise
