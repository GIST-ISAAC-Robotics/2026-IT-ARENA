import importlib.util
import json
from pathlib import Path
import unittest
from unittest.mock import Mock
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

    def test_long_15_percent_requires_its_firmware_and_explicit_guard(self):
        for firmware in ('motor_probe_v1','motor_probe_guarded_5s_v1','unknown'):
            with self.subTest(firmware=firmware), self.assertRaises(ValueError):
                probe.validate_pulse(firmware,150,5000,True)
        probe.validate_pulse('motor_probe_start15_5s_v1',150,5000,True)
        with self.assertRaises(ValueError):
            probe.validate_pulse('motor_probe_start15_5s_v1',150,5000)

    def test_no_doubling_past_cap_or_longer_duration(self):
        for duty,ms in ((151,5000),(300,5000),(150,5001),(150,500),(0,5000),(150.0,5000)):
            with self.subTest(duty=duty,ms=ms), self.assertRaises(ValueError):
                probe.validate_pulse('motor_probe_start15_5s_v1',duty,ms,True)
        with self.assertRaises(ValueError):
            probe.validate_pulse('motor_probe_start15_5s_v1',150,500)
        probe.validate_pulse('motor_probe_start15_5s_v1',100,500)
        probe.validate_pulse('motor_probe_guarded_5s_v1',50,5000,True)

    def test_rewritten_state_file_does_not_refresh_mcu_frames(self):
        now=50_000_000_000
        fresh={'host_ns':now-20_000_000};old={'host_ns':now-27_000_000_000}
        self.assertTrue(probe.telemetry_fresh(fresh,fresh,now))
        self.assertFalse(probe.telemetry_fresh(old,fresh,now))
        self.assertFalse(probe.telemetry_fresh(fresh,old,now))
        self.assertFalse(probe.telemetry_fresh(None,fresh,now))
        self.assertFalse(probe.telemetry_fresh({'host_ns':now+1},fresh,now))

    def test_stale_telemetry_prevents_arm_and_pulse(self):
        b=probe.Bench.__new__(probe.Bench)
        b.firmware='motor_probe_start15_5s_v1'
        b.motor={'host_ns':0};b.encoder={'host_ns':0}
        b.collect=Mock();b.arm=Mock();b.request=Mock()
        with self.assertRaisesRegex(RuntimeError,'stale MCU telemetry'):
            b.pulse(150,5000,observe_guard=True)
        b.arm.assert_not_called();b.request.assert_not_called()

    def test_close_does_not_claim_stop_when_reply_missing(self):
        b=probe.Bench.__new__(probe.Bench)
        b.off=Mock(side_effect=RuntimeError('response timeout'))
        b.port=Mock();b.log=Mock();b.raw=Mock();b.commands=Mock();b.save=Mock()
        b.motor={'pwm_pin':0};b.errors={}
        with self.assertRaises(RuntimeError):b.close()
        b.port.close.assert_called_once()
        self.assertFalse(b.save.call_args.args[1]['stop_confirmed'])

if __name__=='__main__':unittest.main()
