"""좁은 실장 범위를 선택하는 오프라인 CLI 회귀. 합성값이며 장치에 적용하지 않는다."""
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))
from arena_vehicle_interface.steering_calibration import (
    cad_servo_goal, complete_mount_profile, record_zero, save_new_profile,
)


def synthetic_profile(tmp_path, *, direction=1, narrow=True):
    profile = complete_mount_profile(
        record_zero(2000, assembly_id='synthetic-cli-not-installed',
                    straight_confirmed=True, evidence_note='SYNTHETIC CLI TEST ONLY'),
        direction=direction, minimum=1880 if narrow else 1000,
        maximum=2120 if narrow else 3000, confirmed=True)
    path = tmp_path/'zero.json'
    save_new_profile(path, profile)
    return path, profile


def invoke(path, angle_deg, *options):
    before = hashlib.sha256(path.read_bytes()).hexdigest()
    result = subprocess.run(
        [sys.executable, str(ROOT/'scripts/steering_zero_profile.py'), 'goal', str(path),
         '--steering-deg', str(angle_deg), *options],
        capture_output=True, text=True, timeout=15)
    assert hashlib.sha256(path.read_bytes()).hexdigest() == before
    assert result.stderr == ''
    data = json.loads(result.stdout)
    assert data['output_enabled'] is False
    return result.returncode, data


SELECTED = ('--command-limit-rad', '.15', '--wheelbase-m', '.128',
            '--track-m', '.1239', '--wheel-limit-rad', '.2')


class SteeringZeroProfileCliTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.tmp_path = Path(folder.name)

    def test_selected_narrow_contract_reaches_zero_and_both_endpoints(self):
        for direction in (-1, 1):
            folder = self.tmp_path/str(direction)
            folder.mkdir()
            path, profile = synthetic_profile(folder, direction=direction)
            for angle in (-.15, 0., .15):
                with self.subTest(direction=direction, angle=angle):
                    code, result = invoke(path, math.degrees(angle), *SELECTED)
                    self.assertEqual(code, 0, result)
                    expected = cad_servo_goal(profile, math.radians(math.degrees(angle)),
                        command_limit=.15, wheelbase=.128, track=.1239, wheel_limit=.2)
                    self.assertEqual(result, expected)
                    self.assertFalse(result['hardware_angle_measured'])
                    self.assertTrue(1880 <= result['position'] <= 2120)

    def test_unselected_cli_keeps_existing_full_range_requirement(self):
        path, _ = synthetic_profile(self.tmp_path)
        code, result = invoke(path, 0.)
        self.assertEqual(code, 2)
        self.assertIn('range crosses mounted or count limit', result['reason'])

    def test_selected_limit_rejects_command_outside_it(self):
        path, _ = synthetic_profile(self.tmp_path)
        code, result = invoke(path, math.degrees(.15001), *SELECTED)
        self.assertEqual(code, 2)
        self.assertIn('outside CAD central command range', result['reason'])

    def test_old_full_range_invocation_remains_compatible(self):
        path, profile = synthetic_profile(self.tmp_path, narrow=False)
        code, result = invoke(path, 5.)
        self.assertEqual(code, 0)
        self.assertEqual(result, cad_servo_goal(profile, math.radians(5.)))

    def test_invalid_selected_contract_still_rejected(self):
        path, _ = synthetic_profile(self.tmp_path)
        for option, value in (('--wheelbase-m', '0'), ('--track-m', '0'),
                              ('--wheel-limit-rad', 'nan'), ('--command-limit-rad', 'nan')):
            with self.subTest(option=option):
                code, result = invoke(path, 0., *SELECTED, option, value)
                self.assertEqual(code, 2)
                self.assertFalse(result['profile_valid'])


if __name__ == '__main__':
    unittest.main()
