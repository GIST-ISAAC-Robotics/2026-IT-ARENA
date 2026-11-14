"""기존 계약 함수와 설치 생성기의 연결만 검사한다. 모든 수치는 임시 합성 fixture."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import prepare_hardware_policy as subject
sys.path.insert(0, str(subject.ROOT/'tests'))
from test_runtime_installed_limits import synthetic_bundle


class HardwarePolicyTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def fixture(self):
        base = {node: {'ros__parameters': {
            'max_steering_angle_rad': .4, 'max_speed_mps': .7,
            'motion_lidar_frame_id': 'SYNTHETIC_OLD_FRAME',
            'unrelated_example': [1, 'preserve', False]}}
            for node in subject.limits.NODES}
        base['local_pursuit']['ros__parameters'].update(
            g1_g2_moving_pass_trial=False, g1_g2_moving_pass_profile='slow_v1')
        inputs = subject.required_inputs()
        inputs.update(assembly_id='SYNTHETIC-ONLY', runtime_port='SYNTHETIC_NO_DEVICE',
            clock_policy='SYNTHETIC_NO_CLOCK_FILE', motor_feedback_config='SYNTHETIC_NO_FEEDBACK_FILE')
        inputs['shared_ros_parameters'] = {key: ('SYNTHETIC_'+key if key.endswith('frame_id') else .01)
                                          for key in subject.PHYSICAL}
        # 기존 등록 이름을 사용하되 이 테스트가 실제 frame/장착을 확인한 것은 아니다.
        inputs['shared_ros_parameters']['motion_lidar_frame_id'] = 'c1_unmounted_preview'
        bp, ip = self.root/'base.yaml', self.root/'inputs.json'
        bp.write_text(subject.yaml.safe_dump(base), encoding='utf-8')
        subject.save_json(ip, inputs)
        bundle = synthetic_bundle(self.root)
        return base, inputs, bp, ip, bundle

    def test_draft_keeps_real_values_null_and_is_not_launch_profile(self):
        output = self.root/'draft'
        subject.draft(output)
        inputs = json.loads((output/'required_inputs.json').read_bytes())
        self.assertTrue(all(value is None for value in inputs['shared_ros_parameters'].values()))
        self.assertTrue(all(inputs[key] is None for key in ('assembly_id', *subject.HOST)))
        overlay = subject.yaml.safe_load((output/'policy_overlay.yaml').read_bytes())
        subject.check_policy(overlay)
        for node in subject.limits.NODES:
            self.assertNotIn('max_steering_angle_rad', overlay[node]['ros__parameters'])
            self.assertTrue(set(subject.PHYSICAL).isdisjoint(overlay[node]['ros__parameters']))
        status = json.loads((output/'status.json').read_bytes())
        for key in ('executable_runtime_profile','installed','hardware_ready','device_io'):
            self.assertIs(status[key], False)

    def test_composition_preserves_base_and_uses_existing_limits_contract(self):
        base, inputs, bp, ip, bundle = self.fixture()
        original = {path: path.read_bytes() for path in [bp, ip, *bundle.iterdir()]}
        output = self.root/'selected'
        subject.prepare(bp, ip, bundle, output)
        selected = subject.yaml.safe_load((output/'selected/runtime_profile.yaml').read_bytes())
        subject.check_policy(selected)
        manifest = subject.limits.verify(output/'selected', bundle)
        for node in subject.limits.NODES:
            params = selected[node]['ros__parameters']
            self.assertEqual(params['max_steering_angle_rad'], manifest['command_limit_rad'])
            self.assertLess(params['max_steering_angle_rad'], .2)
            self.assertEqual(params['max_speed_mps'], .7)
            self.assertEqual(params['unrelated_example'], [1, 'preserve', False])
            for key, value in inputs['shared_ros_parameters'].items():
                self.assertEqual(params[key], value)
        self.assertIs(selected['local_pursuit']['ros__parameters']['g1_g2_moving_pass_trial'], False)
        self.assertEqual(selected['local_pursuit']['ros__parameters']['g1_g2_moving_pass_profile'], 'slow_v1')
        self.assertEqual({path: path.read_bytes() for path in original}, original)
        self.assertEqual((output/'runtime_base.original.yaml').read_bytes(), original[bp])
        self.assertEqual((output/'hardware_inputs.original.json').read_bytes(), original[ip])
        receipt = json.loads((output/'handover.json').read_bytes())
        self.assertEqual(receipt['supplied_mcu_maximum_speed_mps'], 2.)
        for key in ('measured_inputs_independently_verified','runtime_launch_validated','installed',
                    'hardware_ready','nodes_started','device_io','firmware_written','speed_limits_propagated'):
            self.assertIs(receipt[key], False)

    def test_pending_physical_or_assembly_mismatch_cannot_generate_selected_yaml(self):
        base, inputs, bp, ip, bundle = self.fixture()
        cases = []
        pending = copy.deepcopy(inputs); pending['shared_ros_parameters']['motion_gyro_bias_rad_s'] = None
        cases.append(pending)
        pending = copy.deepcopy(inputs); pending['runtime_port'] = None
        cases.append(pending)
        pending = copy.deepcopy(inputs); pending['assembly_id'] = 'OTHER-SYNTHETIC-ASSEMBLY'
        cases.append(pending)
        pending = copy.deepcopy(inputs); pending['shared_ros_parameters']['motion_lidar_frame_id'] = 'SYNTHETIC_UNREGISTERED_FRAME'
        cases.append(pending)
        for index, value in enumerate(cases):
            with self.subTest(index=index):
                subject.save_json(ip, value)
                output = self.root/('rejected'+str(index))
                with self.assertRaises(ValueError):
                    subject.prepare(bp, ip, bundle, output)
                self.assertFalse(output.exists())

    def test_forged_verified_time_or_guard_mismatch_is_not_accepted(self):
        for node, key, value in (
            ('local_pursuit','motion_scan_timing_verified',True),
            ('lidar_safety','g1_guard_observation_policy','off'),
            ('lidar_safety','g1_observation_input_scope','verified_native_simulation_v1'),
            ('local_pursuit','g1_numeric_execution','python_v1')):
            with self.subTest(node=node,key=key):
                profile = subject.policy_overlay()
                profile[node]['ros__parameters'][key] = value
                with self.assertRaises(ValueError):
                    subject.check_policy(profile)


if __name__ == '__main__':
    unittest.main()
