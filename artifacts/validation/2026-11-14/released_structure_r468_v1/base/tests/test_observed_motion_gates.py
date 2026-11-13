"""선택형 관측 이동 근거: 세계 고정 측방 반환과 전진 시작 조각의 구별."""
from dataclasses import replace

import numpy as np
import pytest

from arena_autonomy.bypass_handoff import (BlockingPersistence, BypassHandoffFollow, ENTRY_FORWARD_DRIFT_M,
                                           HandoffExecutionConfig, handoff_baseline)
from arena_autonomy.pass_release_trial import PassReleaseSelector
from arena_autonomy.static_side_return import StaticSideTracker


def track(speed, object_speed, scans=6, noise=()):
    tracker, ds, results = StaticSideTracker(), .5, []
    for i in range(scans):
        for _ in range(5):
            tracker.advance(speed, .02)
        ds += (object_speed-speed)*.1
        results.append(tracker.observe(i*.1, ds+(noise[i] if i < len(noise) else 0.)))
    return results


def test_return_receding_with_ego_travel_is_world_fixed():
    assert track(1.4, 0.)[-1] is True


def test_parallel_or_overtaking_car_is_not_world_fixed():
    assert not any(track(1.0, 1.0))
    assert not any(track(.8, 1.4))


def test_slow_ego_needs_real_travel_before_any_release():
    assert not any(track(.05, 0.))


def test_car_that_starts_moving_resets_the_evidence():
    tracker = StaticSideTracker()
    ds = .5
    for i in range(4):
        for _ in range(5):
            tracker.advance(1., .02)
        ds -= .1
        assert tracker.observe(i*.1, ds) is (i >= 2)
    for i in range(4, 7):
        for _ in range(5):
            tracker.advance(1., .02)
        ds += 0.  # 자차와 같은 속도로 움직이면 경로 좌표가 그대로다.
        assert tracker.observe(i*.1, ds) is False


def test_lost_or_stale_return_keeps_no_evidence():
    tracker = StaticSideTracker()
    tracker.advance(1., .2)
    tracker.observe(0., .5)
    tracker.observe(.1, None)
    assert tracker.rows == []
    tracker.observe(.2, .3)
    tracker.observe(.9, .2)
    assert len(tracker.rows) == 1


def test_selector_option_is_off_by_default_and_needs_pass_release():
    assert PassReleaseSelector(handoff_baseline()).static_side is None
    assert PassReleaseSelector(replace(handoff_baseline(), static_side_release_trial=True)).static_side is not None
    with pytest.raises(ValueError, match='static side release'):
        replace(handoff_baseline(), pass_release_trial=False, static_side_release_trial=True)


def test_world_fixed_return_clears_guard_state_only_when_proven():
    selector = PassReleaseSelector(replace(handoff_baseline(), static_side_release_trial=True))
    selector.sides['left'].update(seen=True, present=True)
    evidence = {'left': {}}
    assert selector._world_fixed('left', 0., {'frontmost_ds_m': .3}, evidence) is False
    assert selector.mode == 'GUARD'
    tracker = selector.static_side['left']
    tracker.rows = [(-.2, .5, -.2), (-.1, .4, -.1)]
    assert selector._world_fixed('left', 0., {'frontmost_ds_m': .3}, evidence) is True
    assert selector.mode == 'SOLO' and evidence['left']['world_fixed_return'] is True


def handoff(gate):
    return BypassHandoffFollow(handoff_baseline(), HandoffExecutionConfig(entry_motion_gate=gate), allow_simulation_trial=True)


def test_entry_gate_is_off_by_default_and_blocks_only_forward_drift():
    assert HandoffExecutionConfig().entry_motion_gate is False
    off, on = handoff(False), handoff(True)
    for follow in (off, on):
        follow.monitor.forward_drift_m = ENTRY_FORWARD_DRIFT_M+.008
    assert off._front_patch_advancing() is False
    assert on._front_patch_advancing() is True
    on.monitor.forward_drift_m = .005
    assert on._front_patch_advancing() is False
    on.monitor.forward_drift_m = -.03  # 다가오는 조각은 전진 근거가 아니다.
    assert on._front_patch_advancing() is False


def test_forward_drift_resets_with_the_monitor():
    monitor = BlockingPersistence()
    monitor.forward_drift_m = np.float64(.03)
    monitor.reset()
    assert monitor.forward_drift_m == 0.
