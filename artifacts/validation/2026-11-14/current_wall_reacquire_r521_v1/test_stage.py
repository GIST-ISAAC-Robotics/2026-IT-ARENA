"""현재 입력 계약과 점유 반례. 실차/폐루프/실행 허가 검증이 아니다."""
from pathlib import Path
import sys
import math
import json
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(A/'stage'), str(ROOT/'src/arena_autonomy'),
               str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'tests')]
# NumPy 2.5가 제거한 2D cross의 기존 determinant를 시험 프로세스에서만 보존한다.
_cross = np.cross
def compatible_cross(a, b, *args, **kwargs):
    aa, bb = np.asarray(a), np.asarray(b)
    if not args and not kwargs and aa.shape[-1:] == bb.shape[-1:] == (2,):
        return aa[..., 0]*bb[..., 1]-aa[..., 1]*bb[..., 0]
    return _cross(a, b, *args, **kwargs)
np.cross = compatible_cross

from current_wall_reacquire import CurrentWallReacquire, current_wall_candidate, connect_moving_pass
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.decision_process import make_controller
from arena_autonomy.bypass_entry_trial import straight_reference, entry_path
from test_local_follow import history

HEADER = json.loads((A.parent/'continuous_pass_preview_drive_r513_v1/expected_header.json').read_text())
CFG = FollowConfig(**HEADER['follow_config'])


def sensor(now=.7, shift=0., end=2., curved=False, obstacle=False, closed=False, actor=False):
    # 임의 형상은 ray casting 평가용이다. 제어기에는 반환/unknown만 전달한다.
    xs = np.linspace(-2., end, 160)
    center = shift + (.05*np.sin(6*xs) if curved else xs*0)
    segments = []
    for sign in (-1., 1.):
        points = np.c_[xs, center+sign*CFG.wall_offset_m]
        segments.extend(zip(points[:-1], points[1:]))
    if obstacle:
        segments.append((np.array([.18, -.22]), np.array([.18, .22])))
    if actor:
        actor_x = 1.2 if actor is True else float(actor)
        corners = np.array([[actor_x-.1, -.06], [actor_x+.1, -.06], [actor_x+.1, .06],
                            [actor_x-.1, .06], [actor_x-.1, -.06]])
        segments.extend(zip(corners[:-1], corners[1:]))
    if closed:
        for index in (0, -1):
            segments.append((np.array([xs[index], center[index]-CFG.wall_offset_m]),
                             np.array([xs[index], center[index]+CFG.wall_offset_m])))
    origin = np.array([.06, 0.]); ranges = []
    starts = np.array([s[0] for s in segments]); deltas = np.array([s[1]-s[0] for s in segments])
    for angle in -math.pi+np.arange(500)*math.tau/500:
        direction = np.array([math.cos(angle), math.sin(angle)])
        denom = compatible_cross(direction, deltas)
        with np.errstate(divide='ignore', invalid='ignore'):
            distance = compatible_cross(starts-origin, deltas)/denom
            proportion = compatible_cross(starts-origin, direction)/denom
        valid = (distance >= .05)&(distance <= 12)&(proportion >= 0)&(proportion <= 1)&(abs(denom)>1e-12)
        ranges.append(float(np.min(distance[valid])) if np.any(valid) else math.inf)
    first_raw = now-.1
    first_sec = int(first_raw)
    first_nanosec = round((first_raw-first_sec)*1e9)
    first = first_sec+first_nanosec*1e-9
    scene = scene_from_scan(ranges, angle_min=-math.pi, angle_increment=math.tau/500,
        range_min=.05, range_max=12., acquired_at_s=[first+i*.0002 for i in range(500)],
        t_ref_s=now, source_epoch=0, rear_half_angle_deg=0.)
    raw = SimpleNamespace(ranges=ranges, angle_min=-math.pi, angle_increment=math.tau/500,
        range_min=.05, range_max=12., time_increment=.0002,
        header=SimpleNamespace(frame_id='laser_frame',
            stamp=SimpleNamespace(sec=first_sec, nanosec=first_nanosec)))
    return scene, raw


def command(owner, scene):
    return owner._baseline_command(scene, history(), 0., 'left', 0., .1, allow_partial=True)


def test_shifted_current_wall_uses_same_points_without_old_anchor():
    normal, candidate = LocalFollow(CFG), CurrentWallReacquire(CFG)
    for now in (.4, .5, .6):
        scene, _ = sensor(now)
        command(normal, scene); command(candidate, scene)
    shifted, _ = sensor(1.1, shift=.16)
    before, after = command(normal, shifted), command(candidate, shifted)
    assert before[0] == 0 and before[2]['reason'] == 'follow_boundary_unconfirmed'
    assert 0 < after[0] <= .10
    assert after[2]['current_wall_reacquire']['anchor_error_m'] > CFG.boundary_anchor_error_m
    assert after[2]['occupied_points'] == len(shifted.occupied_endpoints_xy_m)
    assert after[2]['boundary_from_memory'] is False


def test_short_curved_current_fragment_positive_then_real_occupied_zero():
    free, _ = sensor(shift=.08, end=.50, curved=True)
    blocked, _ = sensor(shift=.08, end=.50, curved=True, obstacle=True)
    before = command(LocalFollow(CFG), free)
    after = command(CurrentWallReacquire(CFG), free)
    assert before[0] == 0
    assert 0 < after[0] <= .10
    assert after[2]['current_wall_reacquire']['current_boundary_ahead_m'] < CFG.boundary_min_ahead_m
    assert current_wall_candidate(blocked, 0., 'left', 0., .1, CFG) is None


def test_normal_positive_and_non_boundary_stop_unchanged():
    normal, candidate = LocalFollow(CFG), CurrentWallReacquire(CFG)
    for now in (.4, .5, .6, .7):
        scene, _ = sensor(now)
        before, after = command(normal, scene), command(candidate, scene)
        if before[0] > 0:
            assert before == after
    assert before[0] > .10
    blocked, _ = sensor(.8, obstacle=True)
    assert command(normal, blocked) == command(candidate, blocked)


def test_stale_or_front_lost_not_positive():
    scene, _ = sensor()
    assert current_wall_candidate(replace(scene, t_ref_s=1.1), 0., 'left', 0., .1, CFG) is None
    empty = replace(scene, rays=(), source_first_s=None, source_last_s=None)
    assert current_wall_candidate(empty, 0., 'left', 0., .1, CFG) is None


def test_pass_connector_positive_normal_exit_and_occupied_counterexample():
    owner, _ = make_controller(HEADER)
    adapter = owner.moving_pass
    adapter.active = True
    before = (0., 0., dict(reason='moving_pass_current_path_unavailable',
        handoff_state='MOVING_PASS', moving_pass=dict(entry=dict(reason='entry_guide_current_wall_disagrees'))))
    scene, raw = sensor(shift=.16)
    after = connect_moving_pass(adapter, before, scene, history(), 0., 'left', 0., .1, raw)
    assert 0 < after[0] <= .10 and after[2]['handoff_state'] == 'MOVING_PASS'
    assert after[2]['execution_delivery'] == 'existing_nonretained_plan_boundary'
    normal = (.7, .04, dict(reason='moving_pass_entry', handoff_state='MOVING_PASS'))
    assert connect_moving_pass(adapter, normal, scene, history(), 0., 'left', 0., .1, raw) is normal
    blocked, raw_blocked = sensor(shift=.16, obstacle=True)
    assert connect_moving_pass(adapter, before, blocked, history(), 0., 'left', 0., .1, raw_blocked) is before


def test_previous_steering_sweep_can_reject_candidate(monkeypatch):
    import current_wall_reacquire as module
    scene, _ = sensor(shift=.16)
    original = module.swept_limit
    calls = []
    def record(points, angle, *args, **kwargs):
        calls.append(angle)
        return 0. if angle == -.2 else original(points, angle, *args, **kwargs)
    monkeypatch.setattr(module, 'swept_limit', record)
    assert current_wall_candidate(scene, 0., 'left', -.2, .1, CFG) is None
    assert -.2 in calls and len(calls) == 2


def test_pass_old_wall_failure_can_handoff_new_current_follow_without_cap():
    owner, _ = make_controller(HEADER)
    adapter = owner.moving_pass
    adapter.active = True
    before = (0., 0., dict(reason='moving_pass_current_path_unavailable',
        handoff_state='MOVING_PASS', moving_pass=dict(entry=dict(reason='entry_guide_current_wall_disagrees'))))
    results = []
    # ray 일관성을 유지한 별도 정상경로 복귀 시험. 자차 폐루프 운동 증명은 아니다.
    for now, shift in ((.7, .16), (.8, .16), (.9, .08), (1., .08), (1.1, .08)):
        scene, raw = sensor(now, shift=shift, closed=True)
        result = connect_moving_pass(adapter, before, scene, history(), 0., 'left', 0., .1, raw)
        results.append(result)
        if not adapter.active:
            break
    assert results[0][0] == pytest.approx(.10) and results[1][0] == pytest.approx(.10)
    assert not adapter.active
    assert results[-1][0] > .10 and results[-1][2]['handoff_state'] == 'FOLLOW'
    assert results[-1][2]['moving_pass']['trial_cap_retained'] is False


def active_owner_with_old_wall(old_shift=0., actor_x=1.2):
    owner, _ = make_controller(HEADER)
    adapter = owner.moving_pass
    old, _ = sensor(.5, shift=old_shift, closed=True, actor=actor_x)
    _, reference = straight_reference(old, CFG)
    wall = reference['walls']['left']
    fit = (wall['slope'], wall['intercept_m'], *wall['observed_x_m'])
    guide = adapter.guide
    guide.path = entry_path(fit, .16, CFG, guide.entry, 1.2)
    guide.side = 'left'; guide.offset = .16; guide.transition = 1.2; guide.phase = 'entry'
    guide.reference_s = old.t_ref_s; guide.last_source = old.source_first_s
    guide.epoch = old.source_epoch; guide.wall_confirmations = 2
    x = np.asarray(wall['observed_x_m'])
    guide.wall_line = np.c_[x, wall['slope']*x+wall['intercept_m']]
    points = np.asarray(old.occupied_endpoints_xy_m)
    target = points[(points[:, 0] >= actor_x-.11)&(points[:, 0] <= actor_x+.11)&(abs(points[:, 1]) <= .061)]
    assert len(target) >= 3
    adapter.tracker.begin(old, target, reference['walls'])
    adapter.active = True
    return owner


def test_real_moving_pass_guide_failure_connects_to_same_current_helper():
    owner = active_owner_with_old_wall()
    scene, raw = sensor(.7, shift=.16, closed=True, actor=True)
    baseline = owner.moving_pass.command(scene, history(), 0., 'left', 0., .1, raw_scan=raw)
    assert baseline[0] == 0
    assert baseline[2]['reason'] == 'moving_pass_current_path_unavailable'
    after = connect_moving_pass(owner.moving_pass, baseline, scene, history(), 0., 'left', 0., .1, raw)
    assert after[0] == pytest.approx(.10)
    assert after[2]['reacquire_original_reason'] in ('entry_guide_current_outer_wall_unavailable',
                                                   'entry_guide_current_wall_disagrees')


def test_raw_scene_mismatch_does_not_authorize_reacquisition():
    owner = active_owner_with_old_wall()
    scene, raw = sensor(.7, shift=.16, closed=True, actor=True)
    baseline = owner.moving_pass.command(scene, history(), 0., 'left', 0., .1, raw_scan=raw)
    raw.header.stamp.nanosec += 10
    assert connect_moving_pass(owner.moving_pass, baseline, scene, history(), 0., 'left', 0., .1, raw) is baseline


def test_future_bypass_reserve_does_not_turn_current_positive_into_handoff_zero():
    owner = active_owner_with_old_wall(old_shift=-.20)
    for now, shift in ((.7,.16),(.8,.16),(.9,.08),(1.,.08),(1.1,.08)):
        scene, raw = sensor(now,shift=shift,closed=True,actor=True)
        adapter = owner.moving_pass
        baseline = adapter.command(scene,history(),0.,'left',0.,.1,raw_scan=raw)
        result = connect_moving_pass(adapter,baseline,scene,history(),0.,'left',0.,.1,raw)
        assert result[0] == pytest.approx(.10) and adapter.active
    deferred = result[2]['current_follow_handoff_deferred']
    assert deferred['normal_speed_mps'] > .10 and deferred['reserve']['cap_mps'] == 0
    # 실제 가까운 점유는 위 미래 예약과 다르다. 새 현재 점유 검사를 반드시 다시 한다.
    blocked, raw = sensor(1.2,shift=.08,closed=True,actor=True,obstacle=True)
    baseline = adapter.command(blocked,history(),0.,'left',0.,.1,raw_scan=raw)
    assert connect_moving_pass(adapter,baseline,blocked,history(),0.,'left',0.,.1,raw)[0] == 0
