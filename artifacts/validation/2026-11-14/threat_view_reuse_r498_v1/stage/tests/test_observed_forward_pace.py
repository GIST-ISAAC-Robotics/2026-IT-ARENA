"""빠른 전방 반환의 속도 제한과 정지·측방·센서 시간 반례."""
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from arena_autonomy.bypass_handoff import BlockingPersistence, HandoffExecutionConfig
from arena_autonomy.front_patch_motion import ObservedForwardPace
from test_bypass_handoff import context, follow_space
from test_low_motion_reserve import limited


def measure(speed=.85, *, ego=0., noise=None, side=0.):
    from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
    history = MotionHistory(MotionConfig(wheel_radius_m=.0325, history_s=3.))
    for t in np.arange(0., 2., .01):
        history.add_mean_wheel(float(t), ego/.0325)
        history.add_gyro(float(t), 0.)
    estimator = ObservedForwardPace()
    for k in range(5):
        stamp = .5+k*.1
        xy = [1.+(speed-ego)*k*.1+(noise[k] if noise else 0.), side*k*.1]
        estimator.observe(stamp-.1, stamp, xy, history)
    return estimator, history


@pytest.mark.parametrize('ego', [0., .6, 1.2])
def test_forward_pace_compensates_ego_and_subtracts_distance_uncertainty(ego):
    estimator, _ = measure(ego=ego)
    assert estimator.rate == pytest.approx(.85-.06/.4)


@pytest.mark.parametrize('speed,side', [(0.,0.), (-.8,0.), (.1,.8)])
def test_stationary_approaching_or_sideways_patch_is_not_forward_pace(speed,side):
    assert measure(speed,side=side)[0].rate == 0.


@pytest.mark.parametrize('noise', [[0.,.03,-.03,.03,-.03], [-.03,0.,0.,0.,.03]])
def test_distance_noise_does_not_exceed_true_forward_speed_in_fixed_examples(noise):
    assert 0. < measure(noise=noise)[0].rate <= .85+1e-9


def test_single_jump_and_stopped_latest_scan_remove_forward_credit():
    estimator, history = measure()
    estimator.observe(.9, 1., [1.+.85*.4, 0.], history)
    assert estimator.rate == pytest.approx(0.,abs=1e-12)
    assert measure(0.,noise=[0.,0.,0.,0.,.3])[0].rate == 0.


@pytest.mark.parametrize('fault', ['missing','stale','reverse','nonfinite','missing_motion'])
def test_bad_observation_cannot_retain_speed_credit(fault):
    estimator, history = measure()
    source, now, xy = .9, 1., [1.425,0.]
    if fault == 'missing': xy = None
    if fault == 'stale': source,now = 1.4,1.5
    if fault == 'reverse': source,now = .7,.8
    if fault == 'nonfinite': xy = [float('nan'),0.]
    if fault == 'missing_motion':
        with patch('arena_autonomy.front_patch_motion.transport_points',side_effect=ValueError('gap')):
            estimator.observe(source,now,xy,history)
        assert estimator.rate == 0.
        return
    estimator.observe(source,now,xy,history)
    assert estimator.rate == 0.


def test_option_default_and_dependency_preserve_old_records():
    assert not HandoffExecutionConfig.from_mapping({'bypass':{}}).observed_forward_pace
    with pytest.raises(ValueError,match='observed forward pace'):
        HandoffExecutionConfig(observed_forward_pace=True)


@pytest.mark.parametrize('requested,expected', [(1.4,.9), (.4,.4), (0.,0.)])
def test_only_additional_reserve_cap_changes_and_normal_hold_survives(requested,expected):
    ctrl = follow_space()
    cfg = replace(ctrl.execution,moving_follow_reserve=True,closing_follow_reserve=True,
                  low_motion_reserve=True,moving_pace_floor=True,observed_forward_pace=True)
    ctrl.execution = cfg
    ctrl.monitor = BlockingPersistence(cfg)
    ctrl.monitor.rows = [{'source':0.}]
    ctrl.monitor.motion_rate_mps = .85
    ctrl.monitor.forward_pace.rate = .7
    speed, detail = limited(ctrl,.16,requested)
    assert speed == pytest.approx(expected)
    assert detail['observed_forward_pace_mps'] == .7


def test_persistently_stationary_patch_cannot_spend_old_forward_credit():
    ctrl = follow_space()
    cfg = replace(ctrl.execution,moving_follow_reserve=True,closing_follow_reserve=True,
                  low_motion_reserve=True,moving_pace_floor=True,observed_forward_pace=True)
    ctrl.execution = cfg
    ctrl.monitor = BlockingPersistence(cfg)
    ctrl.monitor.rows = [{'source':x} for x in (0.,.1,.2)]
    ctrl.monitor.forward_pace.rate = .8  # 오래된 값이 남아도 현재 이동 근거 없으면 사용 안 함.
    assert limited(ctrl,-.1,1.2)[0] == 0.


def test_duplicate_scan_does_not_count_as_another_motion_sample():
    cfg = replace(follow_space().execution,moving_follow_reserve=True,closing_follow_reserve=True,
                  low_motion_reserve=True,moving_pace_floor=True,observed_forward_pace=True)
    monitor = BlockingPersistence(cfg)
    _,history,scene = context()
    monitor.update(scene,history)
    for _ in range(5):
        assert not monitor.update(scene,history)['fresh']
    assert len(monitor.forward_pace.rows) <= 1 and monitor.forward_pace.rate == 0.
