"""오프라인 profile/ELF/flash 계획 검증과 portable C++ 경계 회귀. 장치를 열지 않는다."""
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
FW = ROOT/'src/arena_vehicle_interface/firmware_uno_q'

def load(name):
    spec = importlib.util.spec_from_file_location('uno_q_'+name, FW/(name+'.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

class ProfileTests(unittest.TestCase):
    def test_defaults_and_fingerprint(self):
        build = load('build')
        for name in ('unconfigured','bench'):
            p = json.loads((FW/'profiles'/(name+'.json')).read_text())
            text = build.profile_header(p)
            self.assertIn('p.kp=0', text)
            self.assertIn(build.profile_fingerprint(p), text)
            self.assertEqual(build.profile_fingerprint(p),build.profile_fingerprint(dict(reversed(list(p.items())))))

    def test_installed_rejects_unknown_or_nonfinite_values(self):
        build=load('build')
        original=json.loads((FW/'profiles/bench.json').read_text())
        original['mode']='installed'
        with self.assertRaises(ValueError): build.profile_header(original)
        # Synthetic compile-input test only; deliberately never emitted as an installed profile artifact.
        synthetic=dict(original, assembly_id='TEST_ONLY', encoder_direction=1,motor_direction_confirmed=True,
                       maximum_duty=.15,kp=.6,ki=1.2,maximum_speed_mps=.5)
        self.assertIn('Mode::Installed',build.profile_header(synthetic))
        for key,value in [('kp',float('nan')),('ki',True),('maximum_duty',1.01),
                          ('maximum_speed_mps',13/3.6+.001),('encoder_direction',0),
                          ('feedback_period_us',50000),('motor_direction_confirmed',False)]:
            with self.subTest(key=key):
                with self.assertRaises(ValueError): build.profile_header(dict(synthetic,**{key:value}))

    def test_flash_default_never_starts_openocd(self):
        flash=load('flash_native')
        with mock.patch.object(flash.subprocess,'run',side_effect=AssertionError('device open prohibited')):
            self.assertEqual(flash.main(['--action','backup','--backup-dir','unused-offline-backup',
                                        '--board-config','absent-offline-config']),0)
        self.assertFalse(Path('unused-offline-backup').exists())

    def test_flash_identity_and_option_byte_boundary(self):
        flash=load('flash_native')
        tcl=flash.identity_tcl('0123456789abcdef01234567')
        self.assertIn('0x482',tcl);self.assertIn('2048',tcl)
        self.assertNotIn('unlock',tcl);self.assertNotIn('options_write',tcl)
        with self.assertRaises(ValueError):flash.identity_tcl('evil;write')
        with self.assertRaises(ValueError):flash.tcl_path(Path('bad}path'))

class PortableTests(unittest.TestCase):
    def test_cpp_boundaries_sanitized_and_size_optimized(self):
        compiler=shutil.which('g++')
        if compiler is None:self.skipTest('g++ required; run this file in the isolated WSL environment')
        output=ROOT/'build/uno_q_firmware_20261114/tests'
        output.mkdir(parents=True,exist_ok=True)
        for mode,flags in [('sanitizers',['-O1','-fsanitize=address,undefined','-fno-omit-frame-pointer']),
                           ('size_optimized',['-Os','-fno-exceptions','-fno-rtti'])]:
            with self.subTest(mode=mode):
                binary=output/('firmware_boundaries_'+mode)
                command=[compiler,'-std=c++17','-Wall','-Wextra','-Werror','-pedantic',*flags,
                         '-I'+str(ROOT/'src/arena_vehicle_interface/portable'),'-I'+str(FW/'include'),
                         str(ROOT/'tests/test_uno_q_firmware.cpp'),'-o',str(binary)]
                result=subprocess.run(command,capture_output=True,text=True,timeout=90)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                result=subprocess.run([str(binary)],capture_output=True,text=True,timeout=20)
                (output/(mode+'.log')).write_text(result.stdout+result.stderr)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                self.assertIn('boundaries PASS',result.stdout)

if __name__=='__main__':unittest.main()
