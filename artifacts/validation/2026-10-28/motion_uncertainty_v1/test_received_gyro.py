import json
from pathlib import Path
import tempfile
import unittest

from audit_received_gyro import audit


class ReceivedGyroAudit(unittest.TestCase):
    def make_data(self, folder, conflict=False):
        rows = []
        for i in range(21):
            t = i / 10
            rows.append(dict(braking_diagnostics=dict(sim_time_s=t,
                truth_yaw_rate_rad_s=0. if t <= 1. else .3,
                truth_longitudinal_mps=0., truth_lateral_mps=0.)))
        (folder / 'trajectory.jsonl').write_text('\n'.join(map(json.dumps, rows)), encoding='utf-8')
        rows = [dict(kind='header', data=dict(motion_config=dict(gyro_bias_rad_s=0.)))]
        for second in range(2):
            rows.append(dict(kind='call', data=dict(input=dict(history=dict(gyro=dict(
                times=[.2, .5, .8, 1.5],
                values=[.01 if not (conflict and second) else .9, .02, .03, 9.]))))))
        (folder / 'follow_decisions.jsonl').write_text('\n'.join(map(json.dumps, rows)), encoding='utf-8')

    def test_stationary_selection_deduplicates_and_excludes_moving_measurements(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            self.make_data(folder)
            result = audit(folder)
            self.assertEqual(result['unique_gyro_samples'], 4)
            self.assertEqual(result['stationary_gyro_samples'], 3)
            self.assertAlmostEqual(result['stationary_gyro_mean_rad_s'], .02)
            self.assertAlmostEqual(result['stationary_gyro_std_rad_s'], .01)
            self.assertTrue(result['input_unchanged'])

    def test_conflicting_repeated_acquisition_is_an_error(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            self.make_data(folder, conflict=True)
            with self.assertRaisesRegex(ValueError, 'same acquisition'):
                audit(folder)


if __name__ == '__main__':
    unittest.main()
