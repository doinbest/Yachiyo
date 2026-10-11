"""Latest B2 vision request/reset and capture tests without camera/serial hardware."""
import ast
import argparse
from pathlib import Path
import threading
import time
import types
import unittest
from unittest.mock import patch


def load_classes():
    source = Path(__file__).resolve().parents[1] / 'orange_pi/vision.py'
    names = ('USBController', 'LatestFrameCamera', 'ColorCircleDetector', 'AdaptiveCenterFilter')
    nodes = [node for node in ast.parse(source.read_text(encoding='utf-8-sig')).body
             if isinstance(node, ast.ClassDef) and node.name in names]
    ns = dict(serial=types.SimpleNamespace(Serial=lambda **_: types.SimpleNamespace()),
              threading=threading, time=time, VALID_COLOR_IDS=set(range(1, 7)),
              VALID_COMMAND_IDS=set(range(1, 7)) | {10, 11, 12},
              COMMAND_NAMES={n: str(n) for n in (*range(1, 7), 10, 11, 12)},
              COLOR_NAMES={n: str(n) for n in range(1, 7)})
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), ns)
    return ns


class LatestVisionTest(unittest.TestCase):
    def setUp(self):
        self.classes = load_classes()

    def arguments(self, *values):
        source = Path(__file__).resolve().parents[1] / 'orange_pi/vision.py'
        nodes = [node for node in ast.parse(source.read_text(encoding='utf-8-sig')).body
                 if isinstance(node, ast.FunctionDef) and node.name == 'parse_arguments']
        ns = {'argparse': argparse}
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), ns)
        with patch('sys.argv', ['vision.py', '--usb-port', '/dev/test', *values]):
            return ns['parse_arguments']()

    def test_numbered_ring_and_material_have_separate_default_centers(self):
        args = self.arguments()
        self.assertEqual((args.center_x, args.center_y), (372, 345))
        self.assertEqual(self.classes['ColorCircleDetector'].__init__.__defaults__[:2], (372, 345))
        self.assertEqual((args.ring_center_x, args.ring_center_y), (378, 333))

    def test_material_center_override_does_not_change_ring_center(self):
        args = self.arguments('--center-x', '350', '--center-y', '340')
        self.assertEqual((args.center_x, args.center_y), (350, 340))
        self.assertEqual((args.ring_center_x, args.ring_center_y), (378, 333))

    def test_ring_center_override_does_not_change_material_center(self):
        args = self.arguments('--ring-center-x', '380', '--ring-center-y', '330')
        self.assertEqual((args.center_x, args.center_y), (372, 345))
        self.assertEqual((args.ring_center_x, args.ring_center_y), (380, 330))

    def camera(self):
        camera = self.classes['LatestFrameCamera'].__new__(self.classes['LatestFrameCamera'])
        camera.running = False
        camera.condition = threading.Condition()
        camera.frame = None
        camera.sequence = 40
        camera.reader_error = None
        return camera

    def test_same_color_b2_request_restarts_command(self):
        usb = self.classes['USBController']('/dev/test')
        usb.rx_buffer.extend(bytes.fromhex('ff b2 03 ff ff b2 03 ff'))
        usb._parse_rx_buffer()
        self.assertEqual(usb.get_command(), (3, 2))

    def test_same_color_reset_returns_detector_to_coarse(self):
        Detector = self.classes['ColorCircleDetector']
        detector = Detector.__new__(Detector)
        detector.choose_color = 3
        detector.mode = 'circle'
        detector.last_circle_center = (298, 386)
        detector.last_circle_radius = 30
        detector.circle_miss_count = 4
        detector.set_color(3)
        self.assertEqual(detector.choose_color, 3)
        self.assertEqual(detector.mode, 'coarse')
        self.assertIsNone(detector.last_circle_center)
        self.assertEqual(detector.circle_miss_count, 0)
        self.assertIsNone(detector.center_filter.value)

    def test_capture_loop_counts_successful_frames_only(self):
        camera = self.camera()
        camera.running = True
        frames = iter([(False, None), (True, [1, 2, 3]), (False, None)])

        def read():
            result = next(frames)
            if camera.sequence == 41:
                camera.running = False
            return result

        camera.capture = types.SimpleNamespace(read=read)
        camera._reader_loop()
        self.assertEqual(camera.read_latest(40), (True, [1, 2, 3], 41))
        self.assertEqual(camera.read_latest(41, timeout=0), (False, None, 41))

    def test_latest_frame_skips_backlog_and_is_copied(self):
        camera = self.camera()
        camera.frame = [9, 8, 7]
        camera.sequence = 45
        ok, frame, sequence = camera.read_latest(40)
        self.assertTrue(ok)
        self.assertEqual(sequence, 45)
        frame[0] = 0
        self.assertEqual(camera.frame, [9, 8, 7])
        self.assertEqual(camera.read_latest(45, timeout=0), (False, None, 45))


if __name__ == '__main__':
    unittest.main()
