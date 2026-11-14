"""장치/ROS 없이 임의 장착·바이어스·증거 부족·상태 전이를 검증한다."""
from dataclasses import replace
import json
import math
import unittest

import numpy as np

from arena_autonomy.imu_quick_calibration import QuickCalibrationConfig, QuickImuCalibration


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.c = QuickImuCalibration()
        self.t = 0.
        self.up = np.array([.36, -.48, .8])
        self.forward = np.cross([0., 1., 0.], self.up)
        self.forward /= np.linalg.norm(self.forward)
        self.bias = np.array([.025, -.018, .032])

    def feed(self, *, state='WAIT_GREEN', speed=0., command=0., gyro=None, accel=None,
             steering=None, sign_verified=True, dt=.02, encoder=True, acceleration=True):
        self.t += dt
        self.c.observe_state(self.t, state, self.t)
        self.c.observe_command(self.t, command, 0. if steering is None else steering, self.t)
        if encoder:
            self.c.observe_encoder(self.t, speed, self.t)
        if acceleration:
            self.c.observe_accel(self.t, self.up*9.80665 if accel is None else accel, self.t)
        if steering is not None:
            self.c.observe_steering(self.t, steering, self.t, sign_verified=sign_verified)
        return self.c.observe_imu(self.t, self.bias if gyro is None else gyro, self.t)

    def stable(self, count=90, **kwargs):
        for _ in range(count):
            self.feed(**kwargs)

    def test_arbitrary_mount_bias_and_gravity_then_yaw_without_forward(self):
        self.stable()
        np.testing.assert_allclose(self.c.bias, self.bias, atol=1e-12)
        np.testing.assert_allclose(self.c.up, self.up, atol=1e-12)
        self.assertIsNone(self.c.forward)
        self.assertEqual(self.c.version, 1)
        for yaw in (-1.3, 0., .7):
            self.assertAlmostEqual(float(np.dot(self.bias+self.up*yaw-self.c.bias, self.c.up)), yaw)
        self.assertFalse(self.c.snapshot()['physical_calibration_verified'])

    def test_many_arbitrary_rotations_including_upside_down(self):
        rng = np.random.default_rng(41)
        for vector in [np.array([0., 0., -1.]), *rng.normal(size=(12, 3))]:
            self.c, self.t = QuickImuCalibration(), 0.
            self.up = vector/np.linalg.norm(vector)
            self.stable()
            np.testing.assert_allclose(self.c.up, self.up, atol=1e-12)

    def test_moving_encoder_cannot_calibrate_in_wait(self):
        self.stable(speed=.1)
        self.assertIsNone(self.c.bias)
        self.assertEqual(self.c.reason, 'encoder_moving')

    def test_zero_encoder_but_command_nonzero_cannot_calibrate(self):
        self.stable(command=.2)
        self.assertIsNone(self.c.bias)

    def test_stale_encoder_does_not_count_as_stationary(self):
        self.feed()
        self.stable(encoder=False)
        self.assertIsNone(self.c.bias)
        self.assertEqual(self.c.reason, 'encoder_stale')

    def test_missing_or_stale_acceleration_cannot_calibrate(self):
        self.feed()
        self.stable(acceleration=False)
        self.assertIsNone(self.c.bias)
        self.assertEqual(self.c.reason, 'accel_stale')

    def test_stationary_command_and_encoder_do_not_hide_hand_rotation(self):
        self.stable(gyro=self.up*.5+self.bias)
        self.assertIsNone(self.c.bias)

    def test_near_g_shaking_restarts_window(self):
        for i in range(150):
            self.feed(accel=self.up*9.80665+self.forward*(.5 if i % 2 else -.5))
        self.assertIsNone(self.c.bias)
        self.assertGreater(self.c.reset_count, 0)
        self.stable()
        self.assertEqual(self.c.version, 1)

    def test_non_gravity_and_no_gravity_not_acceleration_bias(self):
        for value in ([0., 0., 0.], self.up*12.):
            self.c, self.t = QuickImuCalibration(), 0.
            self.stable(accel=value)
            self.assertIsNone(self.c.bias)

    def test_sample_gap_restarts_partial_candidate(self):
        self.stable(40)
        self.feed(dt=.2)
        self.stable(40)
        self.assertIsNone(self.c.bias)
        self.stable(40)
        self.assertEqual(self.c.version, 1)

    def test_leave_wait_immediately_freezes_and_discards_candidate(self):
        self.stable(50)
        self.stable(state='FOLLOW', speed=.3, command=.3)
        self.assertIsNone(self.c.bias)
        self.stable()
        bias = self.c.bias.copy()
        self.stable(state='FOLLOW', speed=.3, command=.3, gyro=self.bias+self.up*.4)
        np.testing.assert_equal(self.c.bias, bias)
        self.assertEqual(self.c.version, 1)

    def test_wait_reentry_recalibrates_only_after_new_stable_window(self):
        self.stable()
        self.feed(state='FOLLOW', speed=.4)
        self.bias += .002
        self.stable(40)
        self.assertEqual(self.c.version, 1)
        self.stable(45)
        self.assertEqual(self.c.version, 2)
        np.testing.assert_allclose(self.c.bias, self.bias)

    def test_explicit_input_boundary_drops_prior_state_not_active_mapping(self):
        self.stable()
        self.c.reset_inputs()
        self.assertIsNone(self.c.encoder)
        self.assertIsNone(self.c.state)
        self.assertEqual(self.c.version, 1)
        np.testing.assert_allclose(self.c.bias, self.bias)

    def test_constant_velocity_has_no_forward_excitation(self):
        self.stable()
        self.stable(state='FOLLOW', speed=.4, command=.4, steering=0.)
        self.assertIsNone(self.c.forward_candidate())
        self.assertIsNone(self.c.encoder_candidate())

    def straight(self, encoder_sign=1):
        for i in range(45):
            self.feed(state='FOLLOW', speed=encoder_sign*i*.01, command=.4, steering=0.,
                      accel=self.up*9.80665+self.forward*.5)

    def turns(self, encoder_sign=1, sign_verified=True, sides=(1, -1), mismatch=False):
        for side in sides:
            angle, velocity = side*.18, .4
            yaw = velocity*math.tan(angle)/self.c.config.wheelbase_m
            if mismatch and side < 0:
                yaw = -yaw
            for _ in range(22):
                self.feed(state='FOLLOW', speed=encoder_sign*velocity, command=.4,
                          steering=angle, sign_verified=sign_verified, gyro=self.bias+self.up*yaw)

    def test_positive_command_does_not_prove_forward_or_encoder_sign(self):
        self.stable()
        self.straight(encoder_sign=-1)
        np.testing.assert_allclose(self.c.forward_candidate(), -self.forward, atol=1e-12)
        self.stable()
        self.assertIsNone(self.c.forward)
        self.assertIsNone(self.c.encoder_candidate())

    def test_verified_actual_steering_identifies_reversed_encoder_at_next_stop(self):
        self.stable()
        self.straight(encoder_sign=-1)
        self.turns(encoder_sign=-1)
        self.assertEqual(self.c.encoder_candidate(), -1)
        self.assertEqual(self.c.encoder_sign, 1)  # 주행 중 바뀌면 안 된다.
        self.assertIsNone(self.c.forward)
        self.stable()
        self.assertEqual(self.c.encoder_sign, -1)
        np.testing.assert_allclose(self.c.forward, self.forward, atol=1e-12)
        self.assertTrue(self.c.snapshot()['encoder_sign_conflicts_configured'])
        self.assertFalse(self.c.snapshot()['motor_controller_polarity_updated'])

    def test_unverified_steering_or_one_side_is_not_sign_reference(self):
        for verified, sides in ((False, (1, -1)), (True, (1,))):
            self.c, self.t = QuickImuCalibration(), 0.
            self.stable()
            self.turns(-1, verified, sides)
            self.assertIsNone(self.c.encoder_candidate())

    def test_inconsistent_left_right_turn_sign_stays_pending(self):
        self.stable()
        self.turns(-1, mismatch=True)
        self.assertIsNone(self.c.encoder_candidate())
        self.stable()
        self.assertEqual(self.c.encoder_sign, 1)

    def test_verified_prior_encoder_profile_can_resolve_forward(self):
        self.c = QuickImuCalibration(QuickCalibrationConfig(encoder_sign_verified=True))
        self.stable()
        self.straight()
        self.stable()
        np.testing.assert_allclose(self.c.forward, self.forward, atol=1e-12)

    def test_tilt_after_move_invalidates_old_forward_evidence(self):
        self.stable()
        self.straight()
        self.turns()
        self.up = np.array([0., 0., 1.])
        self.stable()
        self.assertIsNone(self.c.forward)
        self.assertEqual(self.c.turn_samples, 0)

    def test_duplicate_invalid_and_stale_gyro_cannot_finish_window(self):
        self.stable(50)
        self.assertFalse(self.c.observe_imu(self.t, self.bias, self.t))
        self.assertEqual(len(self.c.window), 0)
        self.assertFalse(self.c.observe_imu(self.t+.01, [float('nan')]*3, self.t+.01))
        self.assertFalse(self.c.observe_imu(self.t-.2, self.bias, self.t))
        self.assertIsNone(self.c.bias)

    def test_snapshot_json_has_no_nonfinite_values(self):
        json.dumps(self.c.snapshot(), allow_nan=False)
        self.stable()
        json.dumps(self.c.snapshot(), allow_nan=False)

    def test_invalid_config(self):
        for name, value in [('min_samples', 1), ('initial_encoder_sign', 0),
                            ('stationary_duration_s', float('nan')), ('wheelbase_m', 0.),
                            ('steering_sign_verified', 'true')]:
            with self.assertRaises(ValueError):
                replace(QuickCalibrationConfig(), **{name: value})

    def raw_count(self, index, count=120, boot='boot-a', sequence=None):
        self.c.observe_encoder_count(self.t+.02, count, boot, index if sequence is None else sequence,
                                     index*20_000, self.t+.02)

    def test_uncalibrated_raw_counts_can_provide_stationarity_without_speed(self):
        for i in range(100):
            self.raw_count(i)
            self.feed(encoder=False)
        self.assertEqual(self.c.version, 1)
        self.assertIsNone(self.c.encoder)  # 속도0 측정을 위조하지 않았다.
        self.assertIsNone(self.c.encoder_candidate())

    def test_changing_raw_counts_never_substitute_for_stationarity(self):
        for i in range(100):
            self.raw_count(i, count=i)
            self.feed(encoder=False)
        self.assertEqual(self.c.version, 0)

    def test_replayed_raw_count_sequence_is_not_new_stationarity_evidence(self):
        for i in range(100):
            self.raw_count(i, sequence=0)
            self.feed(encoder=False)
        self.assertEqual(self.c.version, 0)

    def test_raw_encoder_boot_change_restarts_and_retired_boot_is_rejected(self):
        for i in range(50):
            self.raw_count(i)
            self.feed(encoder=False)
        for i in range(50, 80):
            self.raw_count(i, boot='boot-b')
            self.feed(encoder=False)
        self.assertEqual(self.c.version, 0)
        self.assertFalse(self.c.observe_encoder_count(self.t+.02, 120, 'boot-a', 81, 1_620_000, self.t+.02))

    def test_raw_motion_overrides_quantized_zero_velocity(self):
        for i in range(100):
            self.raw_count(i, count=i)
            self.feed()
        self.assertEqual(self.c.version, 0)

    def test_nominal_raw_count_motion_in_wait_learns_candidates_without_speed_profile(self):
        self.c = QuickImuCalibration(QuickCalibrationConfig(raw_encoder_candidates=True))
        count, sequence = 0, 0
        c = self.c.config
        metres_per_count = 2*math.pi*c.nominal_wheel_radius_m/(c.nominal_counts_per_motor_rev*c.nominal_gear_ratio)
        def raw(delta=0, *, acceleration=0., steering=0., yaw=0.):
            nonlocal count, sequence
            self.t += .05
            sequence += 1
            count += delta
            self.c.observe_state(self.t, 'WAIT_GREEN', self.t)
            self.c.observe_command(self.t, 0., 0., self.t)
            self.c.observe_encoder_count(self.t, count, 'boot-raw', sequence, round(self.t*1e6), self.t)
            self.c.observe_accel(self.t, self.up*9.80665+self.forward*acceleration, self.t)
            self.c.observe_steering(self.t, steering, self.t, sign_verified=True)
            self.c.observe_imu(self.t, self.bias+self.up*yaw, self.t)
        for _ in range(90):
            raw()
        self.assertEqual(self.c.version, 1)
        for delta in range(1, 40):
            raw(-delta, acceleration=metres_per_count/.05**2)
        for side in (1, -1):
            angle, speed = .18*side, 20*metres_per_count/.05
            for _ in range(15):
                raw(-20, steering=angle, yaw=speed*math.tan(angle)/c.wheelbase_m)
        self.assertEqual(self.c.encoder_candidate(), -1)
        self.assertEqual(self.c.dynamic_encoder_basis, 'raw_count_nominal')
        self.assertEqual(self.c.version, 1)  # WAIT라도 실제 움직이는 동안 bias를 갱신하지 않는다.
        for _ in range(90):
            raw()
        self.assertEqual(self.c.raw_encoder_sign, -1)
        self.assertEqual(self.c.encoder_sign, 1)  # 향후 부호변환된 speed topic을 이중 반전하지 않는다.
        np.testing.assert_allclose(self.c.forward, self.forward, atol=1e-12)
        self.assertIsNone(self.c.encoder)
        self.assertFalse(self.c.snapshot()['raw_candidate_speed_published'])

    def test_epoch_float_roundoff_does_not_reject_same_acquisition(self):
        self.t = 1_794_615_200.
        self.c.observe_state(self.t+2e-7, 'WAIT_GREEN', self.t)
        self.assertIsNotNone(self.c.state)
        self.assertTrue(self.c.observe_encoder(self.t+2e-7, 0., self.t))


if __name__ == '__main__':
    unittest.main()
