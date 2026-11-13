import importlib.util
import json
from pathlib import Path
import unittest
import zlib

path=Path(__file__).resolve().parents[1]/'scripts/probe_esp32_encoder.py'
spec=importlib.util.spec_from_file_location('encoder_probe',path)
probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)


def wire(item):
    body=json.dumps(item,separators=(',',':')).encode()
    return f'{zlib.crc32(body):08x} '.encode()+body+b'\n'


class EncoderProbeTests(unittest.TestCase):
    def test_partial_frames_and_boot_prefix(self):
        d=probe.Decoder();p=wire(dict(event='edge',boot=1,seq=2,delta=1))
        self.assertEqual(d.feed(b'ROM text\n'+p[:12]),[])
        self.assertEqual(d.feed(p[12:])[0]['delta'],1)
        self.assertEqual(d.errors['nonframe_lines'],1)

    def test_changed_valid_json_fails_crc(self):
        d=probe.Decoder();p=wire(dict(event='edge',boot=1,seq=2,delta=1))
        self.assertEqual(d.feed(p.replace(b'"delta":1',b'"delta":2')),[])
        self.assertEqual(d.errors['crc_errors'],1)

    def test_quadrature_direction_matches_firmware_table(self):
        table=(0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0)
        positive=(0,2,3,1,0);negative=tuple(reversed(positive))
        self.assertEqual(sum(table[(a<<2)|b] for a,b in zip(positive,positive[1:])),4)
        self.assertEqual(sum(table[(a<<2)|b] for a,b in zip(negative,negative[1:])),-4)
        self.assertEqual(table[3],0)  # 00->11 is invalid, never a two-count step.


if __name__=='__main__':unittest.main()
