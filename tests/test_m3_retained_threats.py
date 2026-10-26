"""같은 측면 두 물체의 위협 기록 보존. 합성 상태 검증이며 물리/주행 증명 아님."""
import numpy as np
import pytest
import arena_autonomy.m3_execution_trial as mx
from test_lateral_intent import GRASS, car
from test_local_follow import history
from test_m3_execution_trial import padding
from test_m3_observed_follow import timed_scene


def sequence(kind, side=1.):
    d0 = -.09*side
    trial = mx.ExecutionLateralTrial(GRASS, mx.ExecutionConfig(release_to_baseline=True))
    rows = []
    for k in range(50):
        t = round(1.+.1*k, 3)
        ax, bx = max(-.65, .3-.4*(t-1.)), .30+.4*abs(t-3.5)
        a = car(ax, .08*side, d0, width=.10)
        b = car(bx, .24*side, d0, width=.10)
        objects = [a, b] if kind == 'both' else [a] if kind == 'a_only' else [b]
        scene, path = timed_scene(t, d0, extra=objects)
        motion = history(end=t+.01)
        motion.poses(np.array([t-.2, t]), t)
        rows.append((t, trial.step(scene, [], path, .35, padding(scene), history=motion)))
    return trial, rows


@pytest.mark.parametrize('side', [1., -1.])
def test_new_same_side_vehicle_cannot_erase_hidden_previous_vehicle(side):
    trial, rows = sequence('both', side)
    assert not any(row.get('released_to_baseline') for _, row in rows)
    assert trial.active


def test_first_vehicle_alone_stays_guarded():
    trial, rows = sequence('a_only')
    assert not any(row.get('released_to_baseline') for _, row in rows) and trial.active


def test_repeated_source_does_not_append_records_or_confirm():
    from dataclasses import replace
    trial = mx.ExecutionLateralTrial(GRASS, mx.ExecutionConfig(release_to_baseline=True))
    scene, path = timed_scene(1., -.09, extra=[car(.3, .13, -.09)])
    trial.step(scene, [], path, .35, padding(scene), history=history())
    count = sum(len(v) for v in trial.release_records.values())
    for t in (1.01, 1.02, 1.03):
        repeated = replace(scene, t_ref_s=t)
        out = trial.step(repeated, [], path, .35, padding(repeated), history=history())
        assert not out['release_gate']['sides']
        assert not out.get('released_to_baseline')
        assert sum(len(v) for v in trial.release_records.values()) == count


def test_capacity_latches_release_without_discarding_unknown_records(monkeypatch):
    monkeypatch.setattr(mx, 'RELEASE_MAX_RECORDS_PER_SIDE', 2)
    trial = mx.ExecutionLateralTrial(GRASS, mx.ExecutionConfig(release_to_baseline=True))
    for t in (1., 1.1, 1.2):
        scene, path = timed_scene(t, -.09, extra=[car(.3, .13, -.09)])
        trial.step(scene, [], path, .35, padding(scene), history=history())
    assert trial.release_memory_blocked == {'left': 'release_memory_capacity_exceeded'}
    assert len(trial.release_records['left']) == 2
    for t in (1.3, 1.4, 1.5):
        scene, path = timed_scene(t, -.09)
        out = trial.step(scene, [], path, .35, padding(scene), history=history())
        assert not out.get('released_to_baseline')
    trial.reset()
    assert not trial.release_records and not trial.release_memory_blocked


def test_memory_can_age_only_since_complete_positive_confirmation():
    # 원래 clear_forward에서 1 s를 넘기기 전 전체 관측이 확인되면 그 확인을 다음 fresh에서 재검사한다.
    trial = mx.ExecutionLateralTrial(GRASS, mx.ExecutionConfig(release_to_baseline=True))
    released = []
    for k in range(18):
        t = round(1.+.1*k, 3)
        scene, path = timed_scene(t, -.09, extra=[car(.3+.45*(t-1.), .13, -.09)])
        out = trial.step(scene, [], path, .35, padding(scene), history=history())
        if out.get('released_to_baseline'):
            released.append(t)
    assert released and not trial.active
