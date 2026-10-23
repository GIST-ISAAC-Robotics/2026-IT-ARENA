"""독립 기하·시간·인과성 검사. 정답 궤적은 기대값 생성에만 사용한다."""
import ast
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/arena_autonomy"))
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable, advance_local


def history(v=2., w=1., **kwargs):
    h = MotionHistory(MotionConfig(**kwargs))
    for t in np.arange(0, .301, .005):
        h.add_wheels(float(t), v / .025, v / .025)
        h.add_gyro(float(t), w)
    return h


class MotionTests(unittest.TestCase):
    def test_zero_yaw_limit(self):
        np.testing.assert_allclose(advance_local(2., 0., .1), (.2, 0., 0.))
        np.testing.assert_allclose(advance_local(2., 1e-10, .1), (.2, 0., 0.), atol=1e-10)

    def test_exact_arc_and_lever_arm(self):
        h = history()
        p, _ = h.poses([0., .2], .2)
        a = .2
        np.testing.assert_allclose(p[1], [2 * math.sin(a) + .0725 * math.cos(a),
                                        2 * (1 - math.cos(a)) + .0725 * math.sin(a), a], atol=1e-12)

    def test_order_and_wrapped_yaw_do_not_change_pose(self):
        h = history(w=20.)
        p, _ = h.poses([0., .3, .1], .3)
        self.assertAlmostEqual(p[1, 2], 6.)
        self.assertAlmostEqual(p[2, 2], 2.)

    def test_modes_translation_and_original_indices(self):
        h = history(v=2., w=0.)
        expected = {"none": [1.97, 1.97], "deskew": [1.77, 1.97],
                    "shift": [1.87, 1.87], "both": [1.67, 1.87]}
        for mode, x in expected.items():
            p, i, m = h.project([2., float('nan'), 2.], 0., 0., .05, 12.,
                                0., .05, .15, mode, True)
            np.testing.assert_allclose(p[:, 0], x)
            np.testing.assert_array_equal(i, [0, 2])
            self.assertAlmostEqual(m['oldest_observation_age_s'], .15)
            self.assertAlmostEqual(m['newest_observation_age_s'], .05)

    def test_rotating_static_world_points_recovered(self):
        # 센서 ray 교점에서 가상 정적 표적을 만들고 알려진 운동으로 역변환한다.
        h = history(v=2., w=1.2, lidar_y_m=.04, lidar_yaw_rad=.3)
        n = 11
        ray_times = np.linspace(0., .1, n)
        pose, _ = h.poses(np.r_[ray_times, .15], .15)
        angles = -.5 + np.arange(n) * .1 + .3
        local = np.column_stack((3 * np.cos(angles) - .03, 3 * np.sin(angles) + .04))
        world = np.empty_like(local)
        for k, (p, point) in enumerate(zip(pose, local)):
            c, s = math.cos(p[2]), math.sin(p[2])
            world[k] = [c * point[0] - s * point[1] + p[0], s * point[0] + c * point[1] + p[1]]
        target = pose[-1]
        d = world - target[:2]
        c, s = math.cos(target[2]), math.sin(target[2])
        expected = np.column_stack((c*d[:, 0]+s*d[:, 1], -s*d[:, 0]+c*d[:, 1]))
        result, _, _ = h.project([3.]*n, -.5, .1, .05, 12., 0., .01, .15, 'both', True)
        np.testing.assert_allclose(result, expected, atol=1e-12)

    def test_direct_equals_two_stage(self):
        h = history()
        first, _, _ = h.project([2.]*11, -.5, .1, .05, 12., 0., .01, .15, 'deskew', True)
        direct, _, _ = h.project([2.]*11, -.5, .1, .05, 12., 0., .01, .15, 'both', True)
        p, _ = h.poses([.1, .15], .15)
        c, s = math.cos(p[0, 2]-p[1, 2]), math.sin(p[0, 2]-p[1, 2])
        d = p[0, :2]-p[1, :2]
        ct, st = math.cos(p[1, 2]), math.sin(p[1, 2])
        shift = np.array([ct*d[0]+st*d[1], -st*d[0]+ct*d[1]])
        two = first @ np.array([[c, s], [-s, c]]) + shift
        np.testing.assert_allclose(direct, two, atol=1e-12)

    def test_stationary_all_modes_equal(self):
        h = history(v=0., w=0.)
        outputs = [h.project([1., 2., 3.], -.2, .2, .05, 12., 0., .05, .15, m, True)[0]
                   for m in ('none', 'deskew', 'shift', 'both')]
        for p in outputs:
            np.testing.assert_allclose(p, outputs[0])

    def test_future_samples_not_used(self):
        h = history(v=2., w=0.)
        h.add_wheels(.4, 10000., 10000.)
        p, m = h.poses([0., .1], .1)
        self.assertAlmostEqual(p[1, 0]-p[0, 0], .2)
        self.assertAlmostEqual(m['wheel_age_s'], 0.)
        with self.assertRaisesRegex(MotionUnavailable, 'future_query'):
            h.poses([.2], .1)

    def test_missing_stale_and_gap(self):
        h = MotionHistory()
        with self.assertRaisesRegex(MotionUnavailable, 'waiting'):
            h.poses([0., .1], .1)
        h.add_wheels(0., 0., 0.)
        h.add_gyro(0., 0.)
        with self.assertRaisesRegex(MotionUnavailable, 'stale'):
            h.poses([0., .1], .1)
        h.add_wheels(.1, 0., 0.)
        h.add_gyro(.1, 0.)
        with self.assertRaisesRegex(MotionUnavailable, 'gap'):
            h.poses([0., .1], .1)

    def test_short_extrapolation_logged(self):
        h = history()
        _, m = h.poses([.2, .32], .32)
        self.assertAlmostEqual(m['extrapolation_s'], .02)

    def test_reject_duplicate_out_of_order_nan_and_reset(self):
        h = history()
        self.assertFalse(h.add_gyro(.1, 1.))
        self.assertFalse(h.add_wheels(.31, float('nan'), 1.))
        h.reset()
        self.assertTrue(h.add_gyro(0., 1.))

    def test_contract_and_stale_scan(self):
        h = history()
        for now, increment, verified, reason in ((.1, .05, False, 'unverified'),
                                               (.1, .1, True, 'future'), (.5, .05, True, 'stale')):
            with self.assertRaisesRegex(MotionUnavailable, reason):
                h.project([2.]*3, 0., .1, .05, 12., 0., increment, now, 'both', verified)

    def test_bias_correction_and_scale(self):
        h = history(v=2., w=.1, gyro_bias_rad_s=.1, wheel_scale=1.05)
        p, _ = h.poses([0., .1], .1)
        self.assertAlmostEqual(p[1, 0]-p[0, 0], .21)
        self.assertAlmostEqual(p[1, 2], 0.)

    def test_input_dependency_is_sensor_only(self):
        p = ROOT / 'src/arena_autonomy/arena_autonomy/lidar_motion_ros.py'
        source = p.read_text()
        for forbidden in ('/odom', '/sim/', '.orientation', 'linear_acceleration'):
            self.assertNotIn(forbidden, source)
        ast.parse(source)

    def test_scan_validity_matches_existing_projection(self):
        from arena_autonomy.core import scan_points
        r = [float('inf'), float('nan'), .01, .05, 1., 12., 13., -1.]
        h = MotionHistory()
        a, count = scan_points(r, -3., .3, .05, 12., -.03)
        b, indices, _ = h.project(r, -3., .3, .05, 12., 0., 0., .1, 'none')
        np.testing.assert_array_equal(a, b)
        self.assertEqual(count, len(indices))

    def test_negative_angle_increment_keeps_time_order(self):
        h = history(v=0., w=0.)
        p, indices, _ = h.project([1., 1., 1.], .3, -.3, .05, 12., 0., .05, .15, 'both', True)
        np.testing.assert_allclose(p[:, 1], np.sin([.3, 0., -.3]), atol=1e-12)
        np.testing.assert_array_equal(indices, [0, 1, 2])

    def test_empty_valid_scan_returns_empty_cloud(self):
        h = history()
        p, indices, m = h.project([float('inf')]*3, 0., .1, .05, 12., 0., .05, .15, 'both', True)
        self.assertEqual(p.shape, (0, 2))
        self.assertEqual(len(indices), 0)
        self.assertEqual(m['valid_points'], 0)

    def test_history_pruning_and_recovery_after_gap(self):
        h = history()
        for t in np.arange(3., 3.21, .005):
            h.add_wheels(float(t), 0., 0.)
            h.add_gyro(float(t), 0.)
        with self.assertRaisesRegex(MotionUnavailable, 'history_missing'):
            h.poses([0., .1], 3.2)
        p, _ = h.poses([3.05, 3.15], 3.2)
        np.testing.assert_allclose(p[0], p[1])

    def test_current_shift_cannot_make_old_scan_fresh(self):
        h = history()
        with self.assertRaisesRegex(MotionUnavailable, 'scan_stale'):
            h.project([1., 2.], 0., .1, .05, 12., 0., .05, .451, 'both', True)

    def test_invalid_configuration_and_metadata(self):
        for kwargs in ({'wheel_radius_m':0.}, {'history_s':.1}, {'gyro_bias_rad_s':float('nan')},
                       {'max_extrapolation_s':-1.}):
            with self.assertRaises(ValueError):
                MotionConfig(**kwargs)
        h = history()
        with self.assertRaisesRegex(MotionUnavailable, 'timing_invalid'):
            h.project([1., 2.], 0., .1, .05, 12., 0., -.01, .1, 'deskew', True)


if __name__ == '__main__':
    unittest.main()
