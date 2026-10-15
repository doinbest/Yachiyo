"""Run the latest vision detector and wire encoder without opening hardware."""
import math
import os
from pathlib import Path
import runpy
import struct
import unittest
from unittest.mock import patch

import cv2
import numpy as np

SOURCE = Path(os.environ.get('PILE_VISION_SOURCE',
    str(Path(__file__).resolve().parents[1] / 'orange_pi/combined_color_digit_ring.py')))
VISION = runpy.run_path(str(SOURCE), run_name='vision_test')


class Port:
    is_open = True

    def __init__(self):
        self.packets = []

    def write(self, packet):
        self.packets.append(packet)
        return len(packet)


class PileLineTest(unittest.TestCase):
    def setUp(self):
        self.detector = VISION['PileLinePoseDetector']()
        self.frame = np.full((480, 640, 3), 235, np.uint8)
        self.usb = VISION['USBController']('/dev/test-not-opened')
        self.port = Port()
        self.usb.serial = self.port

    def raw(self, angle):
        dy = math.tan(math.radians(angle)) * 639 / 2
        return dict(angle_deg=angle, line_points=((0, round(400-dy)),
            (639, round(400+dy))), span_ratio=1.)

    def process_angles(self, angles):
        values = [None if a is None else self.raw(a) for a in angles]
        outputs = []
        globals_ = self.detector.process.__globals__
        with patch.dict(globals_, _extract_pile_line=lambda *a, **k:
                        (values.pop(0), np.zeros((480, 640), np.uint8), 336)):
            for _ in angles:
                outputs.append(self.detector.process(self.frame)[2])
        return outputs

    def test_missed_frames_cannot_be_new_valid_wire_evidence(self):
        outputs = self.process_angles([0, 0, 0, None, None, 0, 0, 0])
        self.assertEqual([d['detected'] for d in outputs],
                         [False, False, True, False, False, False, False, True])
        self.assertIsNotNone(self.detector.last_stable_detection)
        for d in outputs[3:5]:
            self.usb.send_detection(0x0e, d)
            packet = self.port.packets[-1]
            self.assertEqual(packet[3], 0)
            self.assertEqual(packet[4:12], bytes(8))

    def test_stability_is_checked_after_initial_lock_and_recovers(self):
        outputs = self.process_angles([0, 0, 0, 8, -8, 8, 8, 8])
        self.assertEqual([d['detected'] for d in outputs],
                         [False, False, True, False, False, False, False, True])
        self.assertAlmostEqual(outputs[-1]['angle_error_deg'], 8.)

    def test_black_line_and_ring_share_reference_and_sign(self):
        for angle in (-5., 0., 5.):
            dy = math.tan(math.radians(angle)) * 100
            ring = VISION['build_three_ring_pose_detection'](
                ((100, 200-dy), (200, 200), (300, 200+dy)))
            line = self.detector._build_detection(self.raw(angle)['line_points'],
                angle, True, 'test', 1.)
            self.assertEqual(line['angle_error_cdeg'], ring['angle_error_cdeg'])
        # A changed shared reference must also affect the default black-line detector.
        tree_source = SOURCE.read_text(encoding='utf-8-sig').replace(
            'THREE_RING_REFERENCE_YAW_DEG = 0.0',
            'THREE_RING_REFERENCE_YAW_DEG = 2.0')
        namespace = {'__name__': 'reference_test'}
        exec(compile(tree_source, str(SOURCE), 'exec'), namespace)
        detector = namespace['PileLinePoseDetector']()
        result = detector._build_detection(((0, 400), (639, 400)), 0, True, 'test', 1)
        self.assertEqual(result['angle_error_cdeg'], -200)

    def test_distinct_pose_protocols_and_signed_payloads(self):
        for angle in (325, -325):
            detection = dict(detected=True, error_x=-12, error_y=8,
                             angle_error_cdeg=angle, stop_correction=True)
            for command, slots in ((0x0d, (-12, 8, angle, 1)),
                                   (0x0e, (0, 0, angle, 1))):
                self.assertTrue(self.usb.send_detection(command, detection))
                packet = self.port.packets[-1]
                self.assertEqual(packet[:4], bytes((255, 178, command, 1)))
                self.assertEqual(len(packet), 14)
                self.assertEqual(struct.unpack('>hhhh', packet[4:12]), slots)
                self.assertEqual(packet[12], sum(packet[1:12]) & 255)
                self.assertEqual(packet[-1], 254)

    def test_fragmented_requests_switch_source_without_resetting_same_source(self):
        for chunk in (b'noise\xff\xb2', b'\x0d\xff\xff\xb2\x0e\xff',
                      b'\xff\xb2\x0e\xff'):
            self.usb.rx_buffer.extend(chunk)
            self.usb._parse_rx_buffer()
        self.assertEqual(self.usb.get_command(), (0x0e, 2))

    def test_ground_black_bands_with_different_widths(self):
        for width in (3, 7, 11, 21):
            with self.subTest(width=width):
                frame = self.frame.copy()
                cv2.line(frame, (0, 400), (639, 400), (15, 15, 15), width)
                raw, _, _ = VISION['_extract_pile_line'](frame)
                self.assertIsNotNone(raw)
                self.assertAlmostEqual(raw['angle_deg'], 0, delta=.15)

    def test_no_black_band_and_large_shadow_are_rejected(self):
        frames = [self.frame.copy(), np.zeros_like(self.frame), self.frame.copy()]
        frames[2][400:] = 15
        for frame in frames:
            self.assertIsNone(VISION['_extract_pile_line'](frame)[0])

    def test_angle_uses_float_fit_before_drawing_rounds_endpoints(self):
        cv2.line(self.frame, (0, 400), (639, 400), (15, 15, 15), 7)
        angle = .025
        direction = np.array([[1.], [math.tan(math.radians(angle))],
                              [320.], [400.]], np.float32)
        with patch.object(cv2, 'fitLine', return_value=direction):
            raw = VISION['_extract_pile_line'](self.frame)[0]
        self.assertIsNotNone(raw)
        self.assertAlmostEqual(raw['angle_deg'], angle, places=5)


if __name__ == '__main__':
    unittest.main()
