import copy
import json
import unittest
import run_onroad as run


class Contract(unittest.TestCase):
    def tearDown(self):
        run.SIDE, run.GRID_SLOT = 'left', 3

    def test_both_actual_slots_and_all_selected_options(self):
        for side, slot in [('left', 3), ('right', 0)]:
            run.SIDE, run.GRID_SLOT = side, slot
            output = run.BASE / ('not_executed_' + side)
            text, settings, arguments = run.launch_source(output)
            self.assertEqual(arguments['grid_slot'], str(slot))
            self.assertEqual(settings['start_signal_mode'], 'official_4red_local_candidate')
            self.assertTrue(settings['g1_traffic_recovery_trial'])
            self.assertTrue(settings['g1_traffic_path_recovery_trial'])
            compile(text, side + '.launch.py', 'exec')
            effective = dict(speed_profile='g1_compare_1mps', lidar_rate_hz=10,
                lidar_acquisition='sequential', lidar_compensation='both', control_rate_hz=50,
                grid_slot=slot, autonomy_mode='local_pursuit', actuation_mode='virtual_mcu',
                speed_bump=False, collision_detector='configured', g1_follow=True,
                r0_plus=False, lidar_test_profile='', motion_test_profile='',
                record_inputs=False, runtime_artifacts=str(output))
            names = ('partial_observation_policy', 'g1_parallel_hold_trial', 'g1_pass_release_trial',
                'g1_guard_curve_trial', 'g1_traffic_recovery_trial', 'g1_traffic_path_recovery_trial',
                'g1_boundary_evidence_mode', 'g1_trace_path', 'g1_occupancy_timing_mode',
                'g1_corridor_fallback_enabled', 'g1_recent_observation_enabled', 'g1_short_boundary_prefix_enabled')
            effective['extra_arguments'] = [f'evaluation_actor_sdf:={output / "evaluation_actor.sdf"}']
            for key in names:
                value = settings[key]
                effective['extra_arguments'].append(f'{key}:={str(value).lower() if isinstance(value,bool) else value}')
            self.assertTrue(run.validate_launch_call(effective, settings, arguments, output))
            with self.assertRaises(ValueError):
                run.validate_launch_call(dict(effective, grid_slot=3-slot), settings, arguments, output)
            broken = copy.deepcopy(effective)
            broken['extra_arguments'].pop()
            with self.assertRaises(ValueError):
                run.validate_launch_call(broken, settings, arguments, output)

    def test_physics_schedules_and_historical_gates_unchanged(self):
        for side, parent in [('left','g1_traffic_v1.json'),('right','e3_m3_right_grid_v1.json')]:
            original = json.loads((run.ROOT/'config/tests'/parent).read_text(encoding='utf-8'))
            selected = json.loads((run.BASE/(side+'_config.json')).read_text(encoding='utf-8'))
            for key in ('acceptance','actor','cases','seeds','timeouts','vehicle_and_sensor_unchanged'):
                self.assertEqual(original[key], selected[key], (side,key))
            self.assertEqual(selected['launch']['grid_slot'], 3 if side == 'left' else 0)

    def test_adapter_uses_only_short_registered_case(self):
        source = (run.BASE/'run_onroad.py').read_text(encoding='utf-8')
        self.assertNotIn("'stop_restart_full_course'", source)
        self.assertIn("route=dict(case='slow_lead', duration_s=12", source)
        self.assertIn("'--config', str(CONFIG), '--case', 'slow_lead'", source)
        self.assertIn("metadata['side'] != SIDE", source)
        self.assertIn('GRID_SLOT = 3 if SIDE', source)
        self.assertIn('paths.update([CONFIG, OVERLAY', source)


if __name__ == '__main__':
    unittest.main()
