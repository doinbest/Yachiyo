"""Verify exported evidence can be unpacked without mixing captures."""
import importlib.util
from pathlib import Path
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('capture_unpack', ROOT / 'tools/radar_capture_unpack.py')
unpack = importlib.util.module_from_spec(spec)
spec.loader.exec_module(unpack)


def fixture():
    data = b'PREFIX\xcf\xfa\x01\x00END'
    identity = dict(session=13, origin=0, scan=2, map=2)
    raw = dict(identity, id=1, hex=data.hex().upper(), total=len(data),
               crc32=f'{zlib.crc32(data):08X}', complete=True, download_complete=True,
               window=100, circle=[106, len(data)+100], cloud_valid=True,
               unit='mm', pose=[2170, 230, 180])
    cloud = dict(identity, complete=True, points=[[0, 1000, 40]])
    return dict(snapshots=dict(raw=raw, cloud=cloud))


class CaptureUnpackTest(unittest.TestCase):
    def test_window_prefix_is_removed_using_circle_byte_offsets(self):
        raw, circle, metadata = unpack.unpack_capture(fixture())
        self.assertEqual(raw, b'PREFIX\xcf\xfa\x01\x00END')
        self.assertEqual(circle, b'\xcf\xfa\x01\x00END')
        self.assertEqual(metadata['point_count'], 1)

    def test_bad_crc_and_mixed_cloud_are_rejected(self):
        value = fixture()
        value['snapshots']['raw']['hex'] = '00' + value['snapshots']['raw']['hex'][2:]
        with self.assertRaisesRegex(ValueError, 'CRC'):
            unpack.unpack_capture(value)
        value = fixture()
        value['snapshots']['cloud']['scan'] = 9
        with self.assertRaisesRegex(ValueError, '同圈'):
            unpack.unpack_capture(value)

    def test_partial_capture_preserves_bytes_without_claiming_a_circle(self):
        value = fixture()
        value['snapshots']['raw'].update(complete=False, circle=[0, 0], cloud_valid=False)
        del value['snapshots']['cloud']
        raw, circle, metadata = unpack.unpack_capture(value)
        self.assertTrue(raw)
        self.assertIsNone(circle)
        self.assertFalse(metadata['capture_complete'])

    def test_incomplete_download_and_outside_circle_are_rejected(self):
        value = fixture()
        value['snapshots']['raw']['download_complete'] = False
        with self.assertRaisesRegex(ValueError, '下载'):
            unpack.unpack_capture(value)
        value = fixture()
        value['snapshots']['raw']['circle'][0] = 99
        with self.assertRaisesRegex(ValueError, '范围'):
            unpack.unpack_capture(value)


if __name__ == '__main__':
    unittest.main()
