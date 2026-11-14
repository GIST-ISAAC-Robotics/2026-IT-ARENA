import importlib.util
from pathlib import Path
import struct
import unittest

path=Path(__file__).resolve().parents[1]/'scripts/probe_bus_servo.py'
spec=importlib.util.spec_from_file_location('probe_bus_servo',path)
probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)


class ServoProtocolTests(unittest.TestCase):
    def test_official_ping_and_read_examples(self):
        self.assertEqual(probe.packet(1,1).hex(),'ffff010201fb')
        self.assertEqual(probe.packet(1,2,b'\x38\x02').hex(),'ffff0104023802be')

    def test_echo_cannot_be_mistaken_for_response(self):
        sent=probe.packet(1,2,b'\x38\x02')
        reply=bytes.fromhex('ffff0104001805dd')
        frames,errors=probe.responses(sent+reply,sent)
        self.assertEqual(errors,[])
        self.assertEqual(frames,[dict(id=1,status=0,data=b'\x18\x05')])

    def test_bad_checksum_is_not_data(self):
        frames,errors=probe.responses(bytes.fromhex('ffff0104001805dc'),b'')
        self.assertEqual(frames,[]);self.assertIn('checksum',errors)

    def test_feedback_units_and_signed_magnitude(self):
        data=struct.pack('<HHH',2048,0x8003,0x0402)+bytes((120,25,0,0,1))+struct.pack('<HH',2048,0x8004)
        result=probe.feedback(data)
        self.assertEqual(result['position_degrees'],180)
        self.assertEqual(result['speed_signed'],-3)
        self.assertEqual(result['load_signed'],-2)
        self.assertEqual(result['current_signed'],-4)
        self.assertEqual(result['voltage_v'],12)


if __name__=='__main__':unittest.main()
