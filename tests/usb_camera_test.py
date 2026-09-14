"""Exercise the actual Python controller without camera/serial hardware."""
import ast
import argparse
from pathlib import Path
import struct
import threading
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


class Port:
    def __init__(self, **kwargs):
        self.options = kwargs
        self.is_open = True
        self.packets = []
        self.partial = False

    def write(self, packet):
        self.packets.append(packet)
        return len(packet) - int(self.partial)

    def close(self):
        self.is_open = False


class UsbCameraTest(unittest.TestCase):
    def setUp(self):
        tree = ast.parse((ROOT / 'orange_pi/change.py').read_text(encoding='utf-8-sig'))
        classes = [n for n in tree.body if isinstance(n, ast.ClassDef)
                   and n.name in ('USBController', 'UART2Controller')]
        ns = dict(serial=types.SimpleNamespace(Serial=Port), struct=struct,
                  threading=threading, VALID_COLOR_IDS=set(range(1, 7)),
                  COLOR_NAMES={n: str(n) for n in range(1, 7)})
        exec(compile(ast.Module(body=classes, type_ignores=[]), '<controller>', 'exec'), ns)
        self.assertTrue('USBController' in ns, 'USB CDC controller has not been migrated')
        self.controller = ns['USBController'](port='/dev/serial/by-id/test-stm32')

    def test_fragmented_commands(self):
        for part in (b'noise\xff', b'\xb2', b'\x03\xff'):
            self.controller.rx_buffer.extend(part)
            self.controller._parse_rx_buffer()
        self.assertEqual(self.controller.get_command(), (3, 1))

    def test_coalesced_and_invalid_commands(self):
        self.controller.rx_buffer.extend(bytes.fromhex('ff b2 09 ff ff b2 01 ff ff b2 04 ff'))
        self.controller._parse_rx_buffer()
        self.assertEqual(self.controller.get_command(), (4, 2))

    def test_big_endian_signed_errors_and_checksum(self):
        self.controller.send_detection(3, dict(detected=True, center=(334, 338), error_x=-17, error_y=28))
        self.assertEqual(self.controller.serial.packets[-1], bytes.fromhex('ff b2 03 01 01 4e 01 52 ff ef 00 1c 62 fe'))

    def test_invalid_detection_has_zero_coordinates(self):
        self.controller.send_detection(1, dict(detected=False, center=(100, 200), error_x=10))
        self.assertEqual(self.controller.serial.packets[-1], bytes.fromhex('ff b2 01 00 00 00 00 00 00 00 00 00 b3 fe'))

    def test_partial_write_is_reported(self):
        self.controller.serial.partial = True
        with self.assertRaises(OSError):
            self.controller.send_detection(1, dict(detected=False))

    def test_explicit_device_and_legacy_cli_alias(self):
        tree = ast.parse((ROOT / 'orange_pi/change.py').read_text(encoding='utf-8-sig'))
        nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'parse_arguments']
        ns = dict(argparse=argparse)
        exec(compile(ast.Module(body=nodes, type_ignores=[]), '<arguments>', 'exec'), ns)
        for flag in ('--usb-port', '--uart-port'):
            with patch('sys.argv', ['change.py', flag, '/dev/serial/by-id/test']):
                self.assertEqual(ns['parse_arguments']().usb_port, '/dev/serial/by-id/test')


if __name__ == '__main__':
    unittest.main()
