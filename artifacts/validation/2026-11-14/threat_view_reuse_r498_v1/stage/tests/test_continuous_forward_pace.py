"""연속 전진율로 고정 우회 여유만 보완하는 선택 옵션(continuous_forward_pace)의 경계와 반례."""
from dataclasses import replace

import numpy as np
import pytest

from arena_autonomy.bypass_handoff import BlockingPersistence, HandoffExecutionConfig
from arena_autonomy.front_patch_motion import ObservedForwardPace
from test_bypass_handoff import context, follow_space
from test_low_motion_reserve import limited
import g1_handoff_eval


def paced(continuous=True):
    ctrl = follow_space()
    cfg = replace(ctrl.execution, moving_follow_reserve=True, closing_follow_reserve=True, low_motion_reserve=True,
                  moving_pace_floor=True, observed_forward_pace=True, continuous_forward_pace=continuous)
    ctrl.execution = cfg
    ctrl.monitor = BlockingPersistence(cfg)
    # 세 스캔 이상의 낮은 이동 누적, 현재 이동 근거 없음: 기존에는 고정 여유 분기다.
    ctrl.monitor.rows = [{'source': x} for x in (0., .1, .2)]
    return ctrl


def with_pace(ctrl, rate, *, current=True):
    _, _, scene = context(.6)
    ctrl.monitor.forward_pace.rows.append((scene.source_first_s - (0. if current else .1), .6, np.zeros(2)))
    ctrl.monitor.forward_pace.rate = rate
    return ctrl


def estimate(xs, ys=None, stamps=None):
    from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
    history = MotionHistory(MotionConfig(wheel_radius_m=.0325, history_s=3.))
    for t in np.arange(0., 2., .01):
        history.add_mean_wheel(float(t), 0.)
        history.add_gyro(float(t), 0.)
    est = ObservedForwardPace()
    stamps = stamps or [.4 + k*.1 for k in range(len(xs))]
    for k, (x, s) in enumerate(zip(xs, stamps)):
        est.observe(s, s + .1, [x, (ys[k] if ys else 0.)], history)
    return est.rate


def test_default_off_dependency_and_old_records():
    assert HandoffExecutionConfig().continuous_forward_pace is False
    assert HandoffExecutionConfig.from_mapping({'bypass': {}}).continuous_forward_pace is False
    with pytest.raises(ValueError, match='continuous forward pace'):
        replace(paced(False).execution, observed_forward_pace=False, continuous_forward_pace=True)
    with pytest.raises(ValueError, match='continuous forward pace'):
        replace(paced(False).execution, continuous_forward_pace=1)


def test_off_keeps_the_fixed_reserve_branch_even_with_fresh_pace():
    speed, detail = limited(with_pace(paced(False), .15), .02, 1.)
    assert detail['reason'] == 'observed_entry_distance_reserve' and speed == pytest.approx(.02/.8)


@pytest.mark.parametrize('request_mps,expected', [(1., .02/.8+.15), (.1, .1), (0., 0.)])
def test_fresh_forward_pace_only_relaxes_the_additional_reserve(request_mps, expected):
    speed, detail = limited(with_pace(paced(), .15), .02, request_mps)
    assert detail['reason'] == 'continuous_forward_pace' and speed == pytest.approx(expected)
    assert detail['fixed_reserve_cap_mps'] == pytest.approx(.02/.8) and detail['observed_forward_pace_mps'] == .15
    assert detail['free_space_assumed'] is False and detail['identity_or_stationarity_proven'] is False


@pytest.mark.parametrize('case', ['zero_rate', 'previous_scan', 'stale_value_without_rows'])
def test_stale_or_zero_pace_returns_to_fixed_reserve(case):
    ctrl = paced()
    if case == 'zero_rate':
        with_pace(ctrl, 0.)
    elif case == 'previous_scan':
        with_pace(ctrl, .15, current=False)
    else:
        ctrl.monitor.forward_pace.rate = .8  # 행 없이 남은 값은 쓰지 않는다.
    speed, detail = limited(ctrl, .02, 1.)
    assert detail['reason'] == 'observed_entry_distance_reserve' and speed == pytest.approx(.02/.8)


def test_steady_030_lead_gives_conservative_positive_pace():
    rate = estimate([1. + .03*k for k in range(5)])
    assert rate == pytest.approx(.30 - .06/.4)


@pytest.mark.parametrize('xs,limit', [
    ([1., 1.03, 1.06, 1.08, 1.085], .05),   # 감속: 마지막 간격 전진율 이하
    ([1., 1.03, 1.06, 1.06, 1.06], 0.),     # 정지: 마지막 간격 정지면 0
    ([1., 1., 1., 1., 1.18], 0.),           # 정지 조각의 한 번 튐
    ([1., 1.03, 1.06, 1.09, 1.40], 0.),     # 연관 범위를 넘는 교체
])
def test_deceleration_stop_jump_and_replacement_are_not_forward_guarantees(xs, limit):
    assert estimate(xs) <= limit + 1e-12


def test_side_entry_and_duplicate_sensor_time_give_no_pace():
    assert estimate([1. + .03*k for k in range(5)], ys=[.05*k for k in range(5)]) == 0.
    assert estimate([1. + .03*k for k in range(5)], stamps=[.4, .5, .6, .6, .7]) == 0.


def test_evaluation_wiring_requires_paced_observed_motion():
    launch = {'actuation_mode': 'virtual_mcu', 'autonomy_mode': 'local_pursuit', 'speed_bump': False,
              'lidar_rate_hz': 10., 'lidar_compensation': 'both', 'lidar_acquisition': 'sequential',
              'speed_profile': 'g1_open_1p5', 'control_rate_hz': 50.}
    args = g1_handoff_eval.launch_arguments('g1g2', 'paced', launch, True, observed_motion=True, continuous_pace=True)
    assert 'evaluation_handoff_continuous_pace:=true' in args
    assert 'evaluation_handoff_continuous_pace:=true' not in g1_handoff_eval.launch_arguments(
        'g1g2', 'paced', launch, True, observed_motion=True)
    for kw in (dict(observed_motion=False), dict(reserve='low_motion', observed_motion=True)):
        with pytest.raises(ValueError, match='continuous forward pace'):
            g1_handoff_eval.launch_arguments('g1g2', kw.pop('reserve', 'paced'), launch, True, continuous_pace=True, **kw)
    with pytest.raises(ValueError):
        g1_handoff_eval.launch_arguments('g1', None, launch, True, continuous_pace=True)
