"""시험 어댑터가 요청한 회복·보호 옵션을 조용히 누락하지 않는지 확인한다."""
import json
from pathlib import Path
import tempfile
import unittest

import run


class LaunchContract(unittest.TestCase):
    def setUp(self):
        self.out = run.BASE / 'unexecuted_contract_probe'
        self.text, self.settings, self.arguments = run.launch_source(self.out)
        self.required = dict(speed_profile='g1_compare_1mps', lidar_rate_hz=10,
            lidar_acquisition='sequential', lidar_compensation='both', control_rate_hz=50,
            grid_slot=3, autonomy_mode='local_pursuit', actuation_mode='virtual_mcu',
            speed_bump=False, collision_detector='configured', g1_follow=True, r0_plus=False,
            lidar_test_profile='', motion_test_profile='', record_inputs=False,
            runtime_artifacts=str(self.out))
        keys = ['partial_observation_policy', 'g1_parallel_hold_trial', 'g1_pass_release_trial',
                'g1_guard_curve_trial', 'g1_traffic_recovery_trial', 'g1_traffic_path_recovery_trial',
                'g1_boundary_evidence_mode', 'g1_trace_path', 'g1_occupancy_timing_mode',
                'g1_corridor_fallback_enabled', 'g1_recent_observation_enabled',
                'g1_short_boundary_prefix_enabled']
        self.extra = [f'evaluation_actor_sdf:={self.out / "evaluation_actor.sdf"}']
        for key in keys:
            value = self.settings[key]
            text = str(value).lower() if isinstance(value, bool) else str(value)
            self.extra.append(f'{key}:={text}')
        self.required['extra_arguments'] = self.extra

    def test_exact_registered_launch(self):
        self.assertTrue(run.validate_launch_call(self.required, self.settings, self.arguments, self.out))
        self.assertIs(self.settings['g1_traffic_recovery_trial'], True)
        self.assertIs(self.settings['g1_traffic_path_recovery_trial'], True)
        self.assertEqual(self.settings['start_signal_mode'], 'official_4red_local_candidate')
        self.assertEqual(self.arguments['speed_bump'], 'false')
        self.assertIn('signal_driver_v2.py', self.text)
        compile(self.text, 'launch_contract.py', 'exec')

    def test_dropped_extra_is_rejected(self):
        for omitted in range(len(self.extra)):
            values = dict(self.required, extra_arguments=self.extra[:omitted] + self.extra[omitted + 1:])
            with self.subTest(omitted=omitted), self.assertRaises(ValueError):
                run.validate_launch_call(values, self.settings, self.arguments, self.out)

    def test_changed_semantics_are_rejected(self):
        for key, value in [('speed_bump', True), ('speed_profile', 'brisk'), ('grid_slot', 2),
                           ('lidar_test_profile', 'unexpected.json'), ('r0_plus', True)]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                run.validate_launch_call(dict(self.required, **{key: value}), self.settings, self.arguments, self.out)
        for key in ('g1_traffic_recovery_trial', 'g1_traffic_path_recovery_trial'):
            changed = dict(self.settings, **{key: False})
            with self.subTest(key=key), self.assertRaises(ValueError):
                run.validate_launch_call(self.required, changed, self.arguments, self.out)

    def test_duplicate_and_unknown_options_rejected(self):
        for extra in ([*self.extra, self.extra[-1]], [*self.extra, 'extra:=true']):
            with self.assertRaises(ValueError):
                run.validate_launch_call(dict(self.required, extra_arguments=extra), self.settings, self.arguments, self.out)

    def test_only_registered_signal_and_failure_paths_change(self):
        changes = run.effective_sources()
        for original, changed, filename in changes:
            compile(changed, filename, 'exec')
            if filename == 'g1_traffic_eval.py':
                self.assertEqual(original.replace("{'red', 'yellow', 'green'}", "{'red', 'green'}"), changed)
            else:
                self.assertIn("report['error'] = 'signal_driver_failure", changed)
                self.assertIn("report['error'] = 'TimeoutError: green not observed", changed)
                self.assertIn('mcu_stop_service', changed)
                self.assertIn("status['observed_signal_mode']", changed)
        with self.assertRaises(ValueError):
            run.replace_once('ambiguous ambiguous', 'ambiguous', 'replacement')

    def test_runtime_contract_uses_actual_header_and_all_receipts(self):
        old = run.ROOT / 'artifacts/validation/2026-10-28/traffic_path_recovery_connection_v1/run1_course'
        with (old / 'follow_decisions.jsonl').open(encoding='utf-8') as stream:
            first = next(stream)
        signal = (run.SIGNAL / 'dense3_run2/signal_sequence.jsonl').read_text(encoding='utf-8')
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            (out / 'follow_decisions.jsonl').write_text(first, encoding='utf-8')
            (out / 'signal_sequence.jsonl').write_text(signal, encoding='utf-8')
            status = dict(observed_vision_mode='process',
                observed_signal_mode='official_4red_local_candidate', observed_race_supervisor=False)
            status_path = out / 'autonomy_status.jsonl'
            status_path.write_text(json.dumps(status)+'\n', encoding='utf-8')
            metadata = dict(controller_parameters=self.settings)
            result = run.verify_runtime(out, metadata, dict(false_start=False))
            # 이 두 과거 실행의 조합은 계약 함수의 입력 검사일 뿐 통합 주행 성과가 아니다.
            self.assertTrue(result['passed'], result)
            for bad in ({**status, 'observed_signal_mode':'temporary_3light'},
                        {**status, 'observed_vision_mode':'inline'},
                        {**status, 'observed_race_supervisor':True}, {}):
                status_path.write_text(json.dumps(bad)+'\n', encoding='utf-8')
                self.assertFalse(run.verify_runtime(out, metadata, dict(false_start=False))['passed'])
            status_path.write_text('', encoding='utf-8')
            self.assertFalse(run.verify_runtime(out, metadata, dict(false_start=False))['passed'])
            status_path.write_text(json.dumps(status)+'\n', encoding='utf-8')
            content = json.loads(first)
            content['data']['follow_config']['traffic_recovery_trial'] = False
            (out / 'follow_decisions.jsonl').write_text(json.dumps(content), encoding='utf-8')
            self.assertFalse(run.verify_runtime(out, metadata, dict(false_start=False))['passed'])
            (out / 'follow_decisions.jsonl').write_text(first, encoding='utf-8')
            records = [json.loads(line) for line in signal.splitlines()]
            [row for row in records if 'stage' in row][-1]['receipts'].pop()
            (out / 'signal_sequence.jsonl').write_text('\n'.join(json.dumps(r) for r in records), encoding='utf-8')
            self.assertFalse(run.verify_runtime(out, metadata, dict(false_start=False))['passed'])


if __name__ == '__main__':
    unittest.main()
