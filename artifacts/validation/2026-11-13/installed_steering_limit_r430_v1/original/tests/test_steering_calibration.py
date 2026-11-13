"""CAD 조향과 영점의 순수 CPU/임시 파일 검증. 실제 장치는 제어하지 않는다."""
import ast
import hashlib
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
    cad_servo_goal, complete_mount_profile,
    cpp_zero_profile,
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
    def test_cpp_export_requires_same_verified_mount_and_preserves_offsets(self):
        for direction in (-1, 1):
            profile = ready_profile(zero=2137, direction=direction)
            header, digest = cpp_zero_profile(profile, assembly_id='synthetic')
            self.assertIn('p.zero=2137;', header)
            self.assertIn(f'p.direction={direction};', header)
            self.assertIn(digest, header)
        with self.assertRaisesRegex(ValueError, 'identity changed'):
            cpp_zero_profile(profile, assembly_id='different-mount')
        with self.assertRaisesRegex(ValueError, 'pending'):
            cpp_zero_profile(record_zero(2000, assembly_id='synthetic', straight_confirmed=True,
                                        evidence_note='synthetic'), assembly_id='synthetic')
        with self.assertRaisesRegex(ValueError, 'range crosses'):
            cpp_zero_profile(ready_profile(zero=3900), assembly_id='synthetic')

    def test_cpp_export_cli_is_offline_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            profile = Path(folder)/'profile.json'; target = Path(folder)/'mounted.hpp'
            data = ready_profile(); data['evidence_note'] = '\n#error do not paste user text\n'
            save_new_profile(profile, data)
            command = [sys.executable, str(ROOT/'scripts/steering_zero_profile.py'), 'export-cpp',
                       str(profile), '--assembly-id', 'synthetic', '--output', str(target)]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('"firmware_uploaded": false', result.stdout)
            before = target.read_bytes()
            self.assertNotIn(b'#error', before)
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertEqual(target.read_bytes(), before)

    def test_full_cad_commands_quantize_inward_in_both_mount_directions(self):
        limit = CadMapping.default().command_limit(wheelbase=.128, track=.1239, wheel_limit=math.pi/6)
        for direction in (-1,1):
            profile = ready_profile(direction=direction)
            for i in range(-100,101):
                angle = limit*i/100
                goal = cad_servo_goal(profile, angle)
                self.assertLessEqual(abs(goal['central_realized_rad']-angle), .002)
                self.assertFalse(goal['output_enabled'])
                self.assertLessEqual(max(abs(goal['left_target_rad']),abs(goal['right_target_rad'])), math.pi/6)
            with self.assertRaises(ValueError): cad_servo_goal(profile, limit+.001)
        with self.assertRaisesRegex(ValueError, 'range crosses'):
            cad_servo_goal(ready_profile(zero=3900), 0)

    def test_mount_record_and_cli_goal_do_not_send_commands(self):
        pending = record_zero(2100, assembly_id='synthetic', straight_confirmed=True, evidence_note='synthetic')
        ready = complete_mount_profile(pending, direction=-1, minimum=1700, maximum=2500, confirmed=True)
        self.assertEqual(pending['status'], 'zero_recorded_pending_mount_checks')
        self.assertEqual(cad_servo_goal(ready, 0)['position'], 2100)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'profile.json'
            save_new_profile(path, ready)
            result = subprocess.run([sys.executable, str(ROOT/'scripts/steering_zero_profile.py'),
                'goal', str(path), '--steering-deg','10'], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('"output_enabled": false', result.stdout)

    def test_preserved_cad_rows_and_both_design_endpoints(self):
        from arena_vehicle_interface.steering_cad_table import SOURCE_SHA256
        source = ROOT/'artifacts/validation/2026-11-13/steering_cad_fusion_v1/steering_table.csv'
        self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), SOURCE_SHA256)
        mapping = CadMapping.default()
        self.assertEqual(mapping.rows, CadMapping.from_csv(source).rows)
        self.assertEqual(len(mapping.rows), 96)
        for requested, row in ((1., mapping.rows[0]), (-1., mapping.rows[-1])):
            goal = mapping.central(requested, wheelbase=.128, track=.1239, wheel_limit=math.pi/6)
            self.assertAlmostEqual(goal.servo_rad, row[0], places=11)
            self.assertAlmostEqual(goal.left_rad, row[1], places=11)
            self.assertAlmostEqual(goal.right_rad, row[2], places=11)
        for row in mapping.rows:
            goal = mapping.relative(row[0])
            inverse = mapping.central(goal.equivalent_rad, wheelbase=.128, track=.1239, wheel_limit=math.pi/6)
            self.assertAlmostEqual(inverse.servo_rad, row[0], places=11)

    def test_launch_limit_uses_cad_and_legacy_remains_identical(self):
        path = ROOT/'src/arena_bringup/launch/simulation.launch.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_equivalent_steering_limit')
        scope = dict(math=math)
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), scope)
        calc = scope['_equivalent_steering_limit']
        values = dict(wheelbase_m=.128, track_width_m=.1239, max_steering_angle_rad=.45)
        self.assertEqual(calc(values), math.atan(.128/(.128/math.tan(.45)+.1239/2)))
        values.update(simulation_steering_model='fusion_cad_2026_11_13', simulation_drive_mode='single_motor',
                      max_steering_angle_rad=math.pi/6)
        limit = calc(values)
        self.assertGreater(math.degrees(limit), 26.)
        self.assertLess(math.degrees(limit), 27.)
        for key, value in [('simulation_drive_mode','legacy_velocity'), ('track_width_m',.135),
                           ('simulation_steering_model','typo')]:
            with self.assertRaises(ValueError): calc({**values,key:value})

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
