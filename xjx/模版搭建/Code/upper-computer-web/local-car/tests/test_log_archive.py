import json
from pathlib import Path
import sys
import tempfile
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


class LogArchiveTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)
        self.name = '20261009-190539-ffa902'
        self.rx = self.directory / (self.name + '-rx.bin')
        self.events = self.directory / (self.name + '.jsonl')

    def tearDown(self):
        self.temp.cleanup()

    def archive(self):
        import log_archive
        return log_archive.LogArchive(self.directory)

    def test_profile_parameters_survive_export_without_overwriting_original(self):
        headers=('OK grab track config key=x_speed value=60 profile=predict\r\n'
                 'OK grab track config key=x_speed value=25 profile=visual\r\n'
                 'OK camera log_begin id=1 next=0 mode=per_frame\r\n'
                 'OK camera log_end id=1 count=0 queued=0 dropped=0\r\n')
        self.rx.write_bytes(headers.encode())
        with self.archive().export(self.name,'tracking',0) as stream:
            exported=stream.read().decode('utf-8-sig')
        self.assertIn('value=60 profile=predict',exported)
        self.assertIn('value=25 profile=visual',exported)

    def test_disk_console_export_survives_memory_limit_and_new_instance(self):
        self.rx.write_bytes(b'partial without newline')
        records = [dict(time='2026-10-09T11:00:00+00:00', kind='rx', text=f'row {i}') for i in range(700)]
        records.insert(1, dict(time='2026-10-09T11:00:00+00:00', kind='tx', text='grab track begin 3 50'))
        self.events.write_text(''.join(json.dumps(r)+'\n' for r in records), encoding='utf-8')
        archive = self.archive()
        self.assertEqual(archive.list()[0]['name'], self.name)
        with self.archive().export(self.name, 'console') as stream:
            text = stream.read().decode('utf-8-sig')
        self.assertIn('row 0\r\n', text)
        self.assertIn('row 699\r\n', text)
        self.assertIn('TX grab track begin 3 50', text)
        with archive.export(self.name, 'rx') as stream:
            self.assertEqual(stream.read().decode('utf-8-sig'), 'partial without newline')

    def test_tracking_spans_are_separate_include_tail_and_check_crc(self):
        def row(i):
            core = f'TRACK live id=1 i={i} t={i*100} seq={i+1} rx={i*100} data=1,2,0,0,0,0,0,0,0,0,20,0,0,50,100,10,0,20,3'
            return f'{core} crc={zlib.crc32(core.encode()):08X}\r\n'
        good = 'OK grab track log_begin id=1 period_ms=100 next=0\r\n' + ''.join(row(i) for i in range(600)) + 'OK grab track log_end id=1 count=600\r\n'
        bad = 'OK grab track log_begin id=1 period_ms=50 next=0\r\n' + row(0) + row(1).replace('data=1,2', 'data=9,2') + 'OK grab track log_end id=1 count=4\r\n'
        self.rx.write_bytes(('before\r\n'+good+'between\r\n'+bad).encode())
        archive = self.archive()
        sessions = archive.tracking(self.name)
        self.assertEqual(len(sessions), 2)
        self.assertTrue(sessions[0]['trace_complete'])
        self.assertEqual(sessions[0]['records'], 600)
        self.assertEqual(sessions[1]['missing_count'], 3)
        self.assertEqual(sessions[1]['corrupt_rows'], 1)
        with archive.export(self.name, 'tracking', 0) as stream:
            self.assertEqual(stream.read().decode('utf-8-sig'), good)
        with archive.export(self.name, 'tracking', 1) as stream:
            self.assertEqual(stream.read().decode('utf-8-sig'), bad)

    def test_open_segment_and_partial_tail_are_not_reported_complete(self):
        text = b'OK grab track log_begin id=3 period_ms=100 next=0\r\nTRACK live id=3 i=0 t=12'
        self.rx.write_bytes(text)
        session = self.archive().tracking(self.name)[0]
        self.assertFalse(session['trace_complete'])
        self.assertFalse(session['closed'])
        with self.archive().export(self.name, 'tracking', 0) as stream:
            self.assertEqual(stream.read().decode('utf-8-sig'), text.decode())

    def test_unknown_file_and_segment_are_errors(self):
        self.rx.write_bytes(b'')
        with self.assertRaises(ValueError):
            self.archive().export('../other', 'rx')
        with self.assertRaises(ValueError):
            self.archive().export(self.name, 'tracking', 0)

    def test_missing_begin_is_visible_and_parameter_snapshot_is_exported(self):
        core = 'TRACK live id=8 i=12 t=1200 seq=7 rx=1200 data=1,2,0,0,0,0,0,0,0,0,20,0,0,50,100,10,0,20,3'
        line = f'{core} crc={zlib.crc32(core.encode()):08X}\r\n'
        self.rx.write_bytes(('OK grab track config key=body_speed value=160\r\n'+line+'OK grab track log_end id=8 count=13\r\n').encode())
        sessions = self.archive().tracking(self.name)
        self.assertEqual(len(sessions), 1)
        self.assertFalse(sessions[0]['trace_complete'])
        self.assertFalse(sessions[0]['begin_received'])
        self.assertEqual(sessions[0]['missing_count'], 12)
        with self.archive().export(self.name, 'tracking', 0) as stream:
            self.assertIn('key=body_speed value=160', stream.read().decode('utf-8-sig'))

    def test_pixel_segment_contains_all_frames_and_nested_motor_diagnostics(self):
        core='PIX id=1 i=0 t=100 c=3 v=1 m=1 x=334 y=338'
        text=('OK camera log_begin id=1 next=0 mode=per_frame\r\n'
              'OK grab track log_begin id=3 period_ms=200 next=0\r\n'
              f'{core} crc={zlib.crc32(core.encode()):08X}\r\n'
              'OK grab track log_end id=3 count=0\r\n'
              'OK camera log_end id=1 count=1 queued=1 dropped=0\r\n')
        self.rx.write_bytes(text.encode())
        archive=self.archive();sessions=archive.tracking(self.name)
        self.assertEqual(len(sessions),1)
        self.assertEqual(sessions[0]['source_type'],'pixels')
        self.assertEqual(sessions[0]['records'],1)
        self.assertTrue(sessions[0]['trace_complete'])
        with archive.export(self.name,'tracking',0) as stream:
            self.assertEqual(stream.read().decode('utf-8-sig'),text)


if __name__ == '__main__':
    unittest.main()
