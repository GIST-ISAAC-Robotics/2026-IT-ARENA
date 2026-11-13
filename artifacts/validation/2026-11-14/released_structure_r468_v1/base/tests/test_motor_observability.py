"""단일 모터축 관측 한계를 고정한다. 한계 재현 통과는 주행 안전 합격이 아니다."""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.local_follow import LocalFollow
from arena_autonomy.local_scene import LocalScene, SensorRay
from arena_vehicle_interface.actuation_contract import ActuationReceiver, CommandProducer, ContractConfig, State
from arena_vehicle_interface.drive_feedback import MODE_MOTOR, MOTOR_STATIONARY_SCOPE


spec = importlib.util.spec_from_file_location('motor_observability',
    Path(__file__).resolve().parents[1]/'scripts/analyze_motor_observability.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def receiver():
    result = ActuationReceiver(ContractConfig(feedback_mode=MODE_MOTOR,
        ticks_per_rev=28, gear_ratio=15., wheel_radius_m=.0325), boot_id='audit')
    for seq in range(26):
        t = seq*10_000
        result.update_motor_encoder(t, sequence=seq, capture_us=t, motor_counts=0)
        result.tick(t)
    return result, 250_000


def history(v=0., w=0.):
    result = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675,
                                       lidar_x_m=.06, history_s=5.))
    for t in np.arange(0., 1.001, .01):
        result.add_mean_wheel(float(t), v/.0325)
        result.add_gyro(float(t), w)
    return result


def corridor(now):
    xs = np.linspace(-.25, 2., 140)
    points = np.vstack([np.column_stack((xs, np.full_like(xs, y))) for y in (-.425, .425)])
    rays = []
    for i, point in enumerate(points):
        delta = point-np.array([.06, 0.])
        direction = delta/np.linalg.norm(delta)
        rays.append(SensorRay((.06, 0.), tuple(point), now-.1, 'return',
                              'laser_frame', 0, i, tuple(direction)))
    return LocalScene(now, now-.1, now-.1, 'laser_frame', 0, tuple(rays))


def test_motor_gate_is_mean_low_speed_not_both_wheels_or_ground_stop():
    r, now = receiver()
    assert r._stationary(now)
    assert r.status()['stationary_scope'] == MOTOR_STATIONARY_SCOPE
    assert r.left_mps is None and r.right_mps is None
    h = CommandProducer(client_id='test')
    assert r.receive(h.control(r.offer(now), 'ARM'), now)[0]
    assert r.state == State.ARMED
    # 입력에는 차체 속도가 없다. 같은 0카운트는 정지와 상쇄 회전을 구별하지 못한다.
    assert r.mean_wheel_mps == 0.


def test_zero_drive_does_not_use_positive_command_no_progress_fault():
    r, now = receiver()
    h = CommandProducer(client_id='test')
    assert r.receive(h.control(r.offer(now), 'ARM'), now)[0]
    for seq in range(26, 100):
        now = seq*10_000
        r.update_motor_encoder(now, sequence=seq, capture_us=now, motor_counts=0)
        r.tick(now)
        command = h.drive(r.offer(now), speed_mps=0., steering_rad=0., source_us=now, host_now_us=now)
        assert r.receive(command, now)[0]
    assert r.state == State.ARMED
    assert r._stationary(now)


def test_stationary_feedback_does_not_clear_explicit_stop():
    r, now = receiver()
    assert r.receive({'v': 1, 'kind': 'STOP'}, now)[0]
    for seq in range(26, 60):
        now = seq*10_000
        r.update_motor_encoder(now, sequence=seq, capture_us=now, motor_counts=0)
        r.tick(now)
    assert r._stationary(now) and r.state == State.ESTOP
    assert r.target_speed == 0.


@pytest.mark.parametrize('duration', [.1, .3])
def test_zero_mean_and_gyro_cannot_transport_unobserved_translation(duration):
    # 정답을 history에 넣지 않는다. 관측 0이면 추정 변환은 항등 변환으로 남는다.
    points = np.array([[1., 0.], [1., .425]])
    estimated = transport_points(points, history(), .4, .4+duration)
    np.testing.assert_allclose(estimated, points, atol=1e-12)
    expected_for_synthetic_sliding = points-np.array([.9*duration, 0.])
    np.testing.assert_allclose(estimated-expected_for_synthetic_sliding,
                               np.tile([.9*duration, 0.], (2, 1)), atol=1e-12)


def test_gyro_still_transports_rotation_when_mean_zero():
    transported = transport_points([[1., 0.]], history(w=.5), .4, .5)
    assert abs(transported[0, 1]) > .01


def test_stale_zero_is_rejected_but_fresh_zero_is_not_an_error():
    h = history()
    transport_points([[1., 0.]], h, .8, .9)
    with pytest.raises(MotionUnavailable, match='motion_stale'):
        transport_points([[1., 0.]], h, 1., 1.1)


@pytest.mark.parametrize('speed', [0., .2])
def test_g1_release_is_fresh_geometry_gate_not_standstill_gate(speed):
    controller, h = LocalFollow(), history(speed)
    results = [controller.command(corridor(t), h, speed, 'left') for t in (.4, .5, .6, .7)]
    assert results[0][2]['behavior'] == 'HOLD'
    assert results[-1][0] > 0. and results[-1][2]['behavior'] in ('CRUISE', 'FOLLOW')


def test_repeated_scene_cannot_satisfy_fresh_release_confirmations():
    controller, h = LocalFollow(), history()
    for _ in range(5):
        command = controller.command(corridor(.4), h, 0., 'left')
        assert command[0] == 0. and command[2]['behavior'] == 'HOLD'


def diagnostic(t, wheel=0., body=.9):
    return {'sim_time_s': t, 'wheel_surface_speed_mps': wheel,
            'truth_longitudinal_mps': body, 'truth_lateral_mps': 0.,
            'motor_speed_rad_s': 15*wheel/.0325,
            'left_speed_rad_s': wheel/.0325, 'right_speed_rad_s': wheel/.0325}


def test_window_integral_and_signed_cancellation():
    times = [k*.02 for k in range(21)]
    rows = audit.checked_dynamics([diagnostic(t) for t in times])
    assert audit.discrepancy(rows, times, .1, .4) == pytest.approx(.27)
    assert audit.discrepancy(rows, times, .107, .207) == pytest.approx(.09)
    rows = [diagnostic(t, wheel=.9) for t in times]
    assert audit.discrepancy(rows, times, .1, .4) == 0.


def test_missing_time_is_unknown_not_zero_or_extrapolated():
    rows = [diagnostic(t) for t in (0., .02, .1, .12)]
    times = [r['sim_time_s'] for r in rows]
    assert audit.discrepancy(rows, times, .01, .1) is None
    assert audit.discrepancy(rows, times, -.01, .02) is None
    assert audit.discrepancy(rows, times, .1, .14) is None
    assert audit.nearest(rows, times, .05) is None


@pytest.mark.parametrize('kind', ['duplicate', 'unordered', 'nan', 'missing', 'empty'])
def test_diagnostic_input_is_validated(kind):
    rows = [diagnostic(0.), diagnostic(.02)]
    if kind == 'duplicate': rows[1]['sim_time_s'] = 0.
    if kind == 'unordered': rows.reverse()
    if kind == 'nan': rows[1]['wheel_surface_speed_mps'] = math.nan
    if kind == 'missing': del rows[1]['truth_longitudinal_mps']
    if kind == 'empty': rows = []
    with pytest.raises(ValueError):
        audit.checked_dynamics(rows)


def test_stationary_audit_does_not_fill_unmatched_records():
    statuses = [{'now_us': t, 'stationary_ready': True, 'state': 'ARMED'} for t in (20_000, 70_000)]
    result = audit.stationary_audit([diagnostic(.02)], [.02], statuses)
    assert result['matched_records'] == 1 and result['unmatched_records'] == 1
    assert result['body_moving_records'] == 1


def test_g1_event_join_and_preceding_integral():
    times = [k*.02 for k in range(51)]
    rows = [diagnostic(t) for t in times]
    statuses = [{'sim_time_s': .2, 'reason': 'follow_boundary_unconfirmed', 'speed_command_mps': 0.},
                {'sim_time_s': .3, 'reason': 'local_follow', 'speed_command_mps': 1.},
                {'sim_time_s': .4, 'reason': 'follow_boundary_unconfirmed', 'speed_command_mps': 0.},
                {'sim_time_s': .6, 'reason': 'local_follow', 'speed_command_mps': .02}]
    result = audit.g1_events(statuses, rows, times)
    assert result['first_boundary_hold']['preceding_windows']['0.1'] == pytest.approx(.09)
    assert result['first_release_after_boundary_hold']['diagnostic']['truth_longitudinal_mps'] == .9
