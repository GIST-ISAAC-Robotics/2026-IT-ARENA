"""합성 설치 bundle의 상한 전달만 검사한다. 실제 장착값/장치/ROS 실행 없음."""
import contextlib
import copy
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'),
                str(ROOT/'src/arena_vehicle_interface')]
import yaml
import runtime_installed_profile as firmware
import runtime_installed_limits as subject
from arena_vehicle_interface.steering_calibration import complete_mount_profile, record_zero


BASE = {
    'local_pursuit': {'ros__parameters': {'max_steering_angle_rad': .4578436017036438,
        'max_speed_mps': 1.5, 'use_sim_time': False, 'g1_current_corner_budget': 'off',
        'motion_lidar_frame_id': 'c1_unmounted_preview', 'nested_example': [1, 'kept', False]}},
    'lidar_safety': {'ros__parameters': {'max_steering_angle_rad': .4578436017036438,
        'wheelbase_m': .128, 'race_control_mode': 'b3_candidate', 'sensor_wall_timeout_s': 30.}},
}


def synthetic_bundle(root, limit=.2, name='installed'):
    folder = root/name
    board = dict(schema='runtime_installed_board_v1', assembly_id='SYNTHETIC-ONLY',
        encoder_direction=-1, motor_direction_confirmed=True, forward_dir_high=True,
        maximum_duty=.4, kp=.6, ki=1.2, maximum_speed_mps=2., feedback_period_us=20000,
        servo=dict(id=1, goal_speed=120, goal_acceleration=10, position_period_us=100000,
            observation_max_age_us=200000, command_limit=limit, wheelbase=.128,
            track=.1239, wheel_limit=.5235987755982988))
    zero = complete_mount_profile(record_zero(2000, assembly_id=board['assembly_id'],
        straight_confirmed=True, evidence_note='SYNTHETIC TEST ONLY'), direction=1,
        minimum=1000, maximum=3000, confirmed=True)
    bp, zp = root/(name+'.json'), root/(name+'-zero.json')
    bp.write_text(json.dumps(board), encoding='utf-8')
    zp.write_text(json.dumps(zero), encoding='utf-8')
    firmware.prepare(bp, zp, board['assembly_id'], folder)
    return folder


class InstalledLimitsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.base = self.root/'base.yaml'
        self.base.write_text(yaml.safe_dump(BASE, sort_keys=False), encoding='utf-8')
        self.installed = synthetic_bundle(self.root)
        self.output = self.root/'selection'

    def prepare(self):
        return subject.prepare(self.installed, self.base, self.output)

    def test_reduced_limit_is_not_rounded_above_mcu_and_other_values_are_preserved(self):
        manifest = self.prepare()
        limit = manifest['command_limit_rad']
        self.assertEqual(limit, .19999998807907104)
        self.assertLess(limit, manifest['board_command_limit_rad'])
        self.assertEqual(struct.unpack('<f', struct.pack('<f', limit))[0], limit)
        expected = copy.deepcopy(BASE)
        for node in subject.NODES:
            expected[node]['ros__parameters'][subject.PARAMETER] = limit
        self.assertEqual(yaml.safe_load((self.output/'runtime_profile.yaml').read_bytes()), expected)
        host = json.loads((self.output/'runtime_host_limits.json').read_bytes())
        self.assertEqual(host, {'max_steering_rad': limit, 'arguments': ['--max-steering-rad', repr(limit)]})
        self.assertEqual(subject.verify(self.output, self.installed,
            runtime_params=self.output/'runtime_profile.yaml', host_steering_rad=limit), manifest)

    def test_existing_cad_limit_stays_exact(self):
        self.installed = synthetic_bundle(self.root, .4578436017036438, 'cad')
        manifest = self.prepare()
        self.assertEqual(manifest['command_limit_rad'], manifest['board_command_limit_rad'])
        self.assertEqual(yaml.safe_load((self.output/'runtime_profile.yaml').read_bytes()), BASE)

    def test_existing_lower_operational_limit_is_never_silently_increased(self):
        for node in subject.NODES:
            with self.subTest(node=node):
                smaller = copy.deepcopy(BASE)
                smaller[node]['ros__parameters'][subject.PARAMETER] = .1
                self.base.write_text(yaml.safe_dump(smaller), encoding='utf-8')
                with self.assertRaisesRegex(ValueError, 'would widen existing steering limit'):
                    self.prepare()
                self.assertFalse(self.output.exists())
                self.assertEqual(yaml.safe_load(self.base.read_bytes()), smaller)

    def test_original_export_bundle_and_base_are_unchanged_and_not_overwritten(self):
        before = {p.name: p.read_bytes() for p in self.installed.iterdir()}
        base_before = self.base.read_bytes()
        manifest_before = firmware.verify(self.installed)
        self.prepare()
        self.assertEqual(firmware.verify(self.installed), manifest_before)
        self.assertEqual({p.name: p.read_bytes() for p in self.installed.iterdir()}, before)
        self.assertEqual(self.base.read_bytes(), base_before)
        with self.assertRaises(FileExistsError):
            self.prepare()

    def test_both_named_node_limits_must_already_exist(self):
        for missing in ('local_pursuit', 'lidar_safety'):
            with self.subTest(missing=missing):
                broken = copy.deepcopy(BASE)
                del broken[missing]['ros__parameters'][subject.PARAMETER]
                self.base.write_text(yaml.safe_dump(broken), encoding='utf-8')
                with self.assertRaises(ValueError):
                    self.prepare()
                self.assertFalse(self.output.exists())

    def test_duplicate_yaml_limit_is_rejected(self):
        self.base.write_text('local_pursuit:\n  ros__parameters:\n    max_steering_angle_rad: .2\n'
                             '    max_steering_angle_rad: .4\n', encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'duplicate YAML key'):
            self.prepare()
        self.assertFalse(self.output.exists())

    def test_different_firmware_bundle_cannot_be_mixed(self):
        self.prepare()
        other = synthetic_bundle(self.root, .15, 'other')
        with self.assertRaisesRegex(ValueError, 'selected installed bundle'):
            subject.verify(self.output, other)

    def test_changed_generated_profile_is_rejected(self):
        self.prepare()
        (self.output/'runtime_profile.yaml').write_text(yaml.safe_dump(BASE), encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'file changed'):
            subject.verify(self.output, self.installed)

    def test_updated_hash_does_not_hide_unrelated_parameter_change(self):
        self.prepare()
        path = self.output/'runtime_profile.yaml'
        values = yaml.safe_load(path.read_bytes())
        values['local_pursuit']['ros__parameters']['max_speed_mps'] = 9.
        path.write_text(yaml.safe_dump(values), encoding='utf-8')
        manifest_path = self.output/'manifest.json'
        manifest = json.loads(manifest_path.read_bytes())
        manifest['files'][path.name] = subject.digest(path.read_bytes())
        manifest_path.write_text(json.dumps(manifest), encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'beyond the two'):
            subject.verify(self.output, self.installed)

    def test_selected_old_yaml_or_host_limit_is_rejected(self):
        self.prepare()
        with self.assertRaisesRegex(ValueError, 'selected ROS YAML'):
            subject.verify(self.output, self.installed, runtime_params=self.base)
        for bad in (.2, .4578436017036438, float('nan'), True):
            with self.subTest(host_limit=bad), self.assertRaisesRegex(ValueError, 'selected host steering'):
                subject.verify(self.output, self.installed, host_steering_rad=bad)

    def test_cli_generates_checks_and_returns_host_arguments_without_starting_nodes(self):
        def invoke(args):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                code = subject.main(args)
            return code, json.loads(output.getvalue())
        code, result = invoke(['prepare', '--installed-profile', str(self.installed),
            '--runtime-params', str(self.base), '--output', str(self.output)])
        self.assertEqual(code, 0)
        self.assertFalse(result['nodes_started'])
        self.assertFalse(result['firmware_modified'])
        common = ['--selection', str(self.output), '--installed-profile', str(self.installed)]
        code, result = invoke(['host-arguments', *common])
        self.assertEqual((code, result), (0, ['--max-steering-rad', '0.19999998807907104']))
        code, result = invoke(['verify', *common, '--runtime-params', str(self.output/'runtime_profile.yaml'),
                              '--host-steering-rad', '.19999998807907104'])
        self.assertEqual(code, 0)
        code, result = invoke(['verify', *common, '--host-steering-rad', '.4578436017036438'])
        self.assertEqual(code, 2)
        self.assertIn('selected host steering', result['error'])


if __name__ == '__main__':
    unittest.main()
