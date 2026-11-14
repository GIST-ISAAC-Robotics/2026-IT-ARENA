"""복귀 관문의 연속 관측·세대 단절 반례. 주행 성공 판정이 아니다."""
from dataclasses import replace

import numpy as np
import pytest

import arena_autonomy.m3_execution_trial as mx
from test_lateral_intent import GRASS, car
from test_local_follow import history
from test_m3_execution_trial import padding
from test_m3_execution_trial import ctl
from test_m3_observed_follow import timed_scene

D0 = -.09


def call(trial, t, extra=(), dropout=None, epoch=0):
    scene, path = timed_scene(t, D0, extra=list(extra), dropout=dropout)
    if epoch:
        scene = replace(scene, source_epoch=epoch,
                        rays=tuple(replace(r, source_epoch=epoch) for r in scene.rays))
    return trial.step(scene, [], path, .35, padding(scene), history=history())


def test_empty_record_does_not_turn_next_unknown_scan_into_release():
    trial = mx.ExecutionLateralTrial(GRASS, mx.ExecutionConfig(release_to_baseline=True))
    first = call(trial, 1., [car(.30, .13, D0)])
    assert first['threats']['left']['present']
    cleared = call(trial, 1.1)
    assert not cleared.get('released_to_baseline')
    unknown = call(trial, 1.2, dropout=lambda deg: np.ones_like(deg, dtype=bool))
    assert not unknown.get('released_to_baseline'), unknown
    assert trial.active


@pytest.mark.parametrize('change', ['epoch', 'regression'])
def test_active_threat_history_loss_is_not_baseline_permission(change):
    trial = mx.ExecutionLateralTrial(GRASS, mx.ExecutionConfig(release_to_baseline=True))
    call(trial, 1., [car(.30, .13, D0)])
    assert trial.active
    out = call(trial, .9 if change == 'regression' else 1.1, epoch=int(change == 'epoch'))
    assert out['mode'] != 'BASELINE' and trial.active, out


def test_one_positive_clear_scan_after_many_unknowns_is_not_two_confirmations():
    trial = mx.ExecutionLateralTrial(GRASS, mx.ExecutionConfig(release_to_baseline=True))
    call(trial, 1., [car(.30, .13, D0)])
    for t in (1.1, 1.2, 1.3):
        assert not call(trial, t, dropout=lambda deg: np.ones_like(deg, dtype=bool)).get('released_to_baseline')
    out = call(trial, 1.4)
    assert not out.get('released_to_baseline'), out


def test_controller_epoch_transition_cannot_erase_an_active_threat():
    controller = ctl(mx.ExecutionConfig(release_to_baseline=True))
    h = history()
    for k in range(7):
        t = .4+.1*k
        scene, _ = timed_scene(t, D0, extra=[] if k < 3 else [car(.30, .13, D0)])
        controller.command(scene, h, 0., 'left')
    assert controller.lateral.active
    for k in range(6):
        t = 1.3+.1*k
        scene, _ = timed_scene(t, D0)
        scene = replace(scene, source_epoch=1,
                        rays=tuple(replace(r, source_epoch=1) for r in scene.rays))
        out = controller.command(scene, h, 0., 'left')
        assert out[0] == 0. and controller.lateral.active
    assert controller.lateral.release_history_invalidated
    controller.reset()   # 명시 세션 reset만 이 불확실성을 초기화한다.
    assert not controller.lateral.release_history_invalidated
