import importlib.util
import json
from pathlib import Path
import unittest
import zlib

spec=importlib.util.spec_from_file_location('motor_probe',Path(__file__).parents[1]/'scripts/probe_esp32_motor.py')
probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)

class MotorWireTests(unittest.TestCase):
    def test_boot_prefix_only_allowed_before_identity(self):
        body=b'{"event":"info","boot":1,"seq":0}'
        raw=f'{zlib.crc32(body):08x} '.encode()+body
        self.assertEqual(probe.decode_line(b'\x00\xffROM'+raw,True),(json.loads(body),5))
        with self.assertRaises(ValueError):probe.decode_line(b'\x00\xffROM'+raw)

    def test_modified_valid_json_rejected(self):
        body=b'{"event":"status","seq":1}'
        raw=f'{zlib.crc32(body):08x} '.encode()+body.replace(b':1',b':2')
        with self.assertRaises(ValueError):probe.decode_line(raw)

    def test_command_crc_covers_boot_id_token_and_limits(self):
        body='PULSE 123 2 456 50 250'
        packet=probe.packet(body)
        self.assertEqual(packet[9:-1],body.encode())
        self.assertEqual(int(packet[:8],16),zlib.crc32(body.encode()))

if __name__=='__main__':unittest.main()
