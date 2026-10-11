"""Regression for the frozen MCU trace report, not a hardware simulation."""
import importlib.util
from pathlib import Path
import unittest
import zlib

spec = importlib.util.spec_from_file_location('tracking_report', Path(__file__).parents[1] / 'tools/tracking_report.py')
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)


class TrackingReportTest(unittest.TestCase):
    def test_per_frame_pixels_include_invalid_and_keep_coalesced_frames(self):
        lines=['OK camera log_begin id=1 next=0 mode=per_frame']
        for i in range(180):
            core=f'PIX id=1 i={i} t={1000+i*17} c=3 v={int(i!=40)} m=1 x={334+i%7} y=338'
            lines.append(f'{core} crc={zlib.crc32(core.encode()):08X}')
        lines.append('OK camera log_end id=1 count=180 queued=180 dropped=0')
        result=report.summarize_pixels('\n'.join(lines))
        self.assertTrue(result['trace_complete'])
        self.assertEqual(result['records'],180)
        self.assertEqual(result['valid_frames'],179)
        self.assertEqual(result['invalid_frames'],1)
        self.assertAlmostEqual(result['receive_hz'],1000/17)
        repeated = lines[:50] + ['OK camera log_begin id=1 next=49 mode=per_frame'] + lines[50:]
        self.assertTrue(report.summarize_pixels('\n'.join(repeated))['trace_complete'])
        reconnected = ['OK camera log_begin id=1 next=1 mode=per_frame'] + lines[2:]
        self.assertFalse(report.summarize_pixels('\n'.join(reconnected))['trace_complete'])
        lines[21]=lines[21].replace('x=340','x=339')
        lines.append('')
        # Explicitly corrupt a checksum-covered coordinate, and remove a tail row.
        lines[2]=lines[2].replace('y=338','y=339')
        del lines[-3]
        result=report.summarize_pixels('\n'.join(lines))
        self.assertFalse(result['trace_complete'])
        self.assertGreaterEqual(result['missing_count'],2)
        self.assertGreaterEqual(result['corrupt_rows'],1)
    def test_sparse_summary_uses_full_device_timing(self):
        result=report.summarize('OK grab track log_begin id=1 period_ms=300 next=0\n'
            'OK grab track timing visual=30 rx_samples=30 timer=70 updates=99 control_p50=20 control_p95=20 control_max=27 '
            'prediction_p95=20 visual_delay_p95=1 visual_delay_max=3 computation_p95=0 computation_max=1 '
            'batch_samples=40 batch_p50=31 batch_p95=38 batch_max=45 clock_ms=1')
        self.assertEqual(result['actual_batch_p95_ms'],38)
        self.assertEqual(result['control_interval_p95_ms'],20)
        self.assertEqual(result['timing_source'],'device_all_events')
        self.assertEqual(result['visual_receive_to_control_p95_ms'],1)
        unknown=report.summarize('OK grab track timing visual=30 visual_delay_p95=0')
        self.assertIsNone(unknown['visual_receive_to_control_p95_ms'])
    def test_mcu_reset_reuses_id_but_does_not_merge_segments(self):
        lines=[]
        for time_ms in (5000,50):
            lines.append('OK grab track log_begin id=1 period_ms=50 next=0')
            row=f'TRACK live id=1 i=0 t={time_ms} seq=1 rx={time_ms-10} data=2,3,0,0,0,0,0,0,0,0,20,0,0,50,80,70,0,20,1'
            lines.extend([f'{row} crc={zlib.crc32(row.encode()):08X}','OK grab track log_end id=1 count=1'])
        result=report.summarize('\n'.join(lines),source='live')
        self.assertEqual(result['records'],1)
        self.assertTrue(result['trace_complete'])

    def test_raw_and_jsonl_captures_are_readable(self):
        import json
        row='TRACK live id=7 i=0 t=50 seq=1 rx=40 data=2,3,0,0,0,0,0,0,0,0,20,0,0,50,80,70,0,20,1'
        lines=['OK grab track log_begin id=7 period_ms=50 next=0',
               f'{row} crc={zlib.crc32(row.encode()):08X}', 'OK grab track log_end id=7 count=1']
        events=[{'kind':'tx','text':'grab track log start'}]+[{'kind':'rx','text':line} for line in lines]
        text=report.decode_log('\n'.join(json.dumps(e) for e in events).encode())
        self.assertTrue(report.summarize(text,source='live')['trace_complete'])
        self.assertEqual(report.decode_log('\r\n'.join(lines).encode()),'\r\n'.join(lines))
        self.assertEqual(report.decode_log(json.dumps({'details':{'rx':'legacy'}}).encode()),'legacy')

    def test_live_file_keeps_long_segment_and_reports_gaps(self):
        rows = ['OK grab track log_begin id=7 period_ms=50']
        for i in range(200):
            if i == 30:
                continue
            row = (f'TRACK live id=7 i={i} t={1000+i*50} seq={i+1} rx={990+i*50} '
                   'data=2,3,0,0,0,0,0,38,4,13,81,0,0,32,100,90,0,20,3 '
                   'io=10,11,12,13,100,101,102,103,20,30,40,50,8200,25,15,167936,3,3,0,0,255,40,0,8210,26,5')
            rows.append(f'{row} crc={zlib.crc32(row.encode()):08X}')
        rows.append('OK grab track log_end id=7 count=200')
        result = report.summarize('\n'.join(rows))
        self.assertEqual(result['records'], 199)
        self.assertEqual(result['window_ms'], 9950)
        self.assertEqual(result['missing_records'], [30])
        self.assertFalse(result['trace_complete'])
        self.assertEqual(result['scope'], 'live stream; missing rows are not recovered from the MCU ring')
        self.assertEqual(result['sent_wheel_rpm_peak'], [10, 11, 12, 13])
        self.assertEqual(result['wheel_feedback_age_p95_ms'], [20, 30, 40, 50])
        self.assertEqual(result['x_sent_rpm_peak'], 25)

    def test_live_sessions_do_not_mix_with_frozen_export(self):
        rows = ['OK grab track log_begin id=8 period_ms=50']
        for session, i in [(7, 80), (8, 0), (8, 1)]:
            row = (f'TRACK live id={session} i={i} t={100+i*50} seq={i+1} rx={90+i*50} '
                   'data=2,3,0,0,0,0,0,0,0,0,20,0,0,50,100,90,0,20,1')
            rows.append(f'{row} crc={zlib.crc32(row.encode()):08X}')
        rows += ['OK grab track log_end id=8 count=2', 'OK grab track trace_begin count=0', 'OK grab track trace_end']
        result = report.summarize('\n'.join(rows), source='live')
        self.assertEqual(result['records'], 2)
        self.assertTrue(result['trace_complete'])

    def test_deduplicate_measurements_and_keep_startup(self):
        rows = []
        for i, (seq, rx, e, flags, sent_age) in enumerate([
                (1, 80, '12,6', 3, 20), (1, 80, '12,6', 3, 70),
                (2, 180, '3,4', 7, 20), (3, 280, '0,0', 7, 20)]):
            rows.append(f'TRACK row i={i} t={100+i*100} seq={seq} rx={rx} e={e} flags={flags} '
                        f'actual=2,3,4 cmd=5,6,7 target=8,9 x={20+i} forward={i} left={i*2} '
                        f'room={50-i*2} age=90 batch=100 request_age=0 sent_age={sent_age}')
        result = report.summarize('\n'.join(rows + rows[-1:]))
        self.assertEqual(result['records'], 4)
        self.assertEqual(result['new_valid_samples'], 3)
        self.assertEqual(result['tracking_samples'], 3)
        self.assertEqual(result['stable_samples'], 2)
        self.assertEqual(result['startup']['first_stable_ms'], 200)
        self.assertEqual(result['startup']['peak_axis_px'], [12, 6])
        self.assertAlmostEqual(result['stable_error']['rms_xy_px'], (25/2)**.5)
        self.assertEqual(result['x_feedback_age_p95_ms'], 90)
        self.assertEqual(result['displacement_mm'], {'forward': 3, 'left': 6, 'x': 3})

    def test_observation_is_not_tracking_acceptance(self):
        result = report.summarize('TRACK row i=0 t=50 rx=40 seq=1 e=80,20 flags=1 '
                                  'actual=0,0,0 cmd=0,0,0 target=0,0 x=20 forward=0 left=0 '
                                  'room=50 age=75 batch=0 request_age=65535 sent_age=65535')
        self.assertEqual(result['tracking_samples'], 0)
        self.assertIsNone(result['stable_error'])
        self.assertIsNone(result['actual_batch_p95_ms'])
        self.assertIsNone(result['startup'])

    def test_missing_evidence_is_not_zero_error(self):
        result = report.summarize('OK grab track state=track_observe valid=0')
        self.assertEqual(result['records'], 0)
        self.assertIsNone(result['all_error'])
        self.assertIsNone(result['receive_interval_p95_ms'])

    def test_compact_crc_and_missing_corrupt_records(self):
        row = 'TRACK row i=0 t=50 seq=1 rx=40 data=2,3,0,0,0,0,0,60,4,8,20,0,0,50,80,70,0,20,3'
        crc = f'{zlib.crc32(row.encode()):08X}'
        result = report.summarize('OK grab track trace_begin count=2\n'+row+' crc='+crc+'\n'+row.replace('data=2,3','data=2,9')+' crc='+crc+'\nTRACK row broken\nOK grab track trace_end')
        self.assertEqual(result['records'], 1)
        self.assertEqual(result['missing_records'], [1])
        self.assertEqual(result['corrupt_rows'], 2)
        self.assertFalse(result['trace_complete'])
        self.assertEqual(result['all_error']['peak_axis_px'], [2, 3])


if __name__ == '__main__':
    unittest.main()
