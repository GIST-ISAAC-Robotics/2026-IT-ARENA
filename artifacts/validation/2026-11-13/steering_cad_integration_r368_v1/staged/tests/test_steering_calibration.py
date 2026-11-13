"""r368 초안: 순수 CPU/임시 파일만. 아직 실행하지 않은 회귀."""
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))
from arena_vehicle_interface.steering_calibration import (
    CadMapping, check_profile, covers, read_profile, record_zero, relative_goal, save_new_profile,
)


def ready_profile(zero=2000, direction=1):
    # 합성값. 운영 교정값이나 실제 방향/끝점으로 복사하지 않는다.
    value = record_zero(zero, assembly_id='synthetic', straight_confirmed=True, evidence_note='synthetic')
    value.update(status='mount_checks_recorded', counts_per_turn=4096,
                 count_direction_per_cad_positive=direction, safe_min_count=100, safe_max_count=4000,
                 direction_confirmed=True, endpoints_confirmed=True, position_mode_confirmed=True,
                 scale_confirmed=True)
    return value


class SteeringCalibrationTests(unittest.TestCase):
    def test_zero_only_can_persist_but_does_not_enable_goal(self):
        data = record_zero(17, assembly_id='synthetic', straight_confirmed=True, evidence_note='synthetic')
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'zero.json'
            save_new_profile(path, data)
            self.assertEqual(read_profile(path), data)
            with self.assertRaises(FileExistsError):
                save_new_profile(path, data)
        with self.assertRaisesRegex(ValueError, 'pending'):
            relative_goal(data, 0)

    def test_neutral_is_offset_and_direction_is_explicit(self):
        for direction in (-1, 1):
            profile = ready_profile(direction=direction)
            self.assertEqual(relative_goal(profile, 0)['position'], 2000)
            self.assertEqual(relative_goal(profile, 2*math.pi/4096)['position'], 2000+direction)

    def test_invalid_and_missing_fields_never_make_a_target(self):
        for key, bad in [('zero_count', True), ('zero_count', 4096),
                         ('count_direction_per_cad_positive', None), ('counts_per_turn', 4095),
                         ('safe_min_count', 2000), ('scale_confirmed', False),
                         ('hardware_angle_measured', True), ('count_wrap_policy', 'modulo')]:
            profile = ready_profile()
            profile[key] = bad
            with self.assertRaises(ValueError):
                relative_goal(profile, 0)
        for angle in (math.nan, math.inf, True):
            with self.assertRaises(ValueError):
                relative_goal(ready_profile(), angle)

    def test_neutral_near_wrap_is_saved_but_range_not_covered(self):
        for zero in (0, 5, 4090, 4095):
            pending = record_zero(zero, assembly_id='synthetic', straight_confirmed=True, evidence_note='synthetic')
            self.assertEqual(pending['zero_count'], zero)
            if zero in (0, 4095):
                checked = ready_profile(zero)
                checked.update(safe_min_count=0, safe_max_count=4095)
                with self.assertRaisesRegex(ValueError, 'without wrap'):
                    check_profile(checked, ready=True)
            else:
                checked = ready_profile(zero)
                checked.update(safe_min_count=0, safe_max_count=4095)
                self.assertFalse(covers(checked, -.3, .3))
                goal = relative_goal(checked, -.3 if zero == 5 else .3)
                self.assertTrue(goal['clamped'])
                self.assertEqual(goal['position'], 0 if zero == 5 else 4095)

    def test_assembly_change_invalidates_reuse(self):
        with self.assertRaisesRegex(ValueError, 'identity changed'):
            check_profile(ready_profile(), assembly_id='new-mount')

    def test_duplicate_keys_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'bad.json'
            path.write_text('{"schema":1,"schema":2}', encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'duplicate'):
                read_profile(path)

    def test_cli_records_without_device_import_or_io(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'profile.json'
            command = [sys.executable, str(ROOT/'scripts/steering_zero_profile.py'), 'record',
                       '--neutral-count', '4090', '--assembly-id', 'synthetic',
                       '--confirm-wheels-straight', '--evidence-note', 'synthetic only', '--output', str(path)]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(read_profile(path)['zero_count'], 4090)

    def test_synthetic_cad_round_trip_and_inner_bound(self):
        mapping = CadMapping([(-.4, .49, .38), (-.1, .12, .11), (0, 0, 0),
                              (.1, -.111, -.121), (.4, -.39, -.50)])
        for angle in (-.3, -.1, 0, .1, .3):
            goal = mapping.central(angle, wheelbase=.128, track=.1239, wheel_limit=.45)
            self.assertAlmostEqual(goal.equivalent_rad, angle, places=11)
            self.assertLessEqual(max(abs(goal.left_rad), abs(goal.right_rad)), .45)
        for angle in (-100., 100.):
            goal = mapping.central(angle, wheelbase=.128, track=.1239, wheel_limit=.45)
            self.assertTrue(goal.clamped)
            self.assertLessEqual(max(abs(goal.left_rad), abs(goal.right_rad)), .45)
        self.assertTrue(mapping.relative(1).clamped)

    def test_bad_cad_sign_nonfinite_fold_or_missing_zero(self):
        for rows in [[(-.1, .1, .1), (0, 0, 0), (.1, .1, .1)],
                     [(-.1, .1, .1), (0, 0, 0), (.1, math.nan, -.1)],
                     [(-.1, .1, .1), (.05, -.05, -.05), (.1, -.1, -.1)]]:
            with self.assertRaises(ValueError):
                CadMapping(rows)


if __name__ == '__main__':
    unittest.main()
