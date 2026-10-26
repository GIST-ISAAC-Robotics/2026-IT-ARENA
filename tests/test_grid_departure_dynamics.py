"""제어 성공이 아니라 M5 이상 운동 시험기의 시간·좌표·보호 경계를 검사한다."""
import math

import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig
from arena_vehicle_interface.drive_feedback import MotorEncoderSpec
from g0_geometry import TruthBox, Footprint
from probe_grid_departure_observation import motion_history, append_stationary_measurements, moving_ranges
from simulate_grid_departure import IdealPlant, acquired_ranges, IndependentGuard, event_key
from test_grid_departure_observation import ROOM, SENSOR, EGO
from test_lidar_safety import node_double


def test_same_time_receipt_precedes_control_without_float_order_accident():
    receive = event_key(.5+500*.0002+.0998+.002, 999, 'scan_delivery', None)
    control = event_key(.6018+5*.02, 1, 'control', None)
    assert receive[0] == control[0] == 701800000
    assert receive < control
    assert event_key(.701800001, 1, 'scan_delivery', None) > control


def test_plant_is_rear_axle_bicycle_and_time_cannot_reverse():
    p = IdealPlant(EGO, .135)
    p.speed, p.steering = .2, .1
    p.advance(.5)
    k = math.tan(.1)/.135
    turn = k*.1
    expected = (-.0675+math.sin(turn)/k+.0675*math.cos(turn),
                (1-math.cos(turn))/k+.0675*math.sin(turn), turn)
    np.testing.assert_allclose(p.center, expected, atol=1e-14)
    assert p.travel == pytest.approx(.1)
    with pytest.raises(ValueError): p.advance(.49)


def test_ray_uses_both_ego_and_peer_acquisition_positions():
    times = .5+np.arange(500)*.0002
    poses = np.column_stack((.2*(times-.5), np.zeros((500, 2))))
    values = acquired_ranges(poses, times, [TruthBox(.7, 0., .12)], [(.5, 0.)], ROOM, SENSOR, .05, 12.)
    assert values[250] == pytest.approx(.7+.5*.05-.1-.06-.2*.05)
    stationary, _ = moving_ranges(EGO, [TruthBox(.7, 0., .12)], [(.5, 0.)], ROOM, SENSOR, times, .05, 12.)
    assert values[250] != pytest.approx(stationary[250])


def test_fixed_ego_matches_previous_scan_generator_and_near_occlusion_stays_unknown():
    times = .5+np.arange(500)*.0002
    boxes = [TruthBox(.1, 0., .12, Footprint(.04, .15))]
    actual = acquired_ranges(np.zeros((500, 3)), times, boxes, [(0., 0.)], ROOM, SENSOR, .05, 12.)
    expected, _ = moving_ranges(EGO, boxes, [(0., 0.)], ROOM, SENSOR, times, .05, 12.)
    np.testing.assert_allclose(actual, expected, equal_nan=True)
    assert math.isnan(actual[250])


def test_single_motor_quantization_preserves_integrated_travel_to_half_count():
    spec = MotorEncoderSpec(28, 15., .0325)
    distance = .12345
    count = spec.counts_from_motor_angle(distance/.0325*15.)
    reconstructed = spec.mean_wheel_mps(spec.motor_rad_s_from_counts(count, 1.))
    assert isinstance(count, int)
    assert abs(reconstructed-distance) <= spec.mean_wheel_travel_m_per_count/2
    assert spec.counts_per_wheel_revolution == 420


@pytest.mark.parametrize('fault', [False, True])
def test_independent_protection_preserves_zero_and_rejects_unknown(fault):
    h = motion_history(.135, SENSOR)
    append_stationary_measurements(h, 10.1)
    node, _, _ = node_double()
    scan = node.scan
    scan.header.frame_id = 'laser_frame'
    scan.time_increment = .0002
    if fault: scan.ranges[250] = math.nan
    guard = IndependentGuard(h, FollowConfig(wheelbase_m=.135))
    safe, meta = guard.command(10.1, scan, .2, 0.)
    assert safe.drive.speed == (0. if fault else .2)
    if fault: assert meta['reason'] == 'front_unobserved'
    safe, _ = guard.command(10.1, scan, 0., 0.)
    assert safe.drive.speed == 0.
