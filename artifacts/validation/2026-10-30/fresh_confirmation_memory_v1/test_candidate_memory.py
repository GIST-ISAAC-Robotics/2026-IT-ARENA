"""fresh 확인 기억 후보의 국소 반례. 실제 proposal·swept_limit·광선별 팽창을 쓰고 장면만 손으로 만든다.

기준 배치: 직선 경로, 앞차 반환 x=1.4 m, 옆차 가까운 면 y=∓0.2152 m(Gazebo 오른쪽 참값). 옆 광선은 스캔 시작 0.02 s 뒤 취득.
fresh 판단(스캔 시작 +0.10 s)은 나이 0.08 s라 통과하고, 같은 스캔 재사용(+0.20 s, 나이 0.18 s — 10/30 Gazebo 오른쪽 차단 광선 나이)은 팽창 때문에 거리 0에서 막힌다.
"""
import math
import random
from types import SimpleNamespace

import pytest

import arena_autonomy.pass_release_trial as ps
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.measured_parallel_trial import MeasuredParallelTrial
import candidate_state as cs
from candidate_state import FreshMemoryParallelTrial

CHAIN = dict(partial_observation_policy='bounded_local_v4', boundary_evidence_mode='corner_distance_candidate',
             occupancy_timing_mode='per_ray_candidate', parallel_hold_trial=True, pass_release_trial=True,
             guard_curve_trial=True, traffic_recovery_trial=True, traffic_path_recovery_trial=True,
             corridor_fallback_enabled=True, rear_lane_keep_trial=True, rear_lane_keep_policy='observed_measured_v1')
CONFIG = FollowConfig(**CHAIN)
PATH = [[x/10, 0.] for x in range(11)]


def base(wall='left'):
    return (1., 0., {'reason': 'local_follow', 'path_points': PATH, 'swept_clearance_m': 1.4, 'preferred_wall': wall,
                     'partial_observation': dict(allowed=True, reason='fully_observed')})


def neighbor(y=-.2152, dx=0., offset=.02):
    return [(x+dx, y, offset) for x in (-.09, -.06, -.03, 0., .03, .06, .09)]


def scene(source, t_ref, points, epoch=0):
    rays = [SimpleNamespace(status='return', acquired_at_s=source+offset) for _, _, offset in points]
    return SimpleNamespace(source_first_s=source, t_ref_s=t_ref, source_epoch=epoch, rays=rays,
                           occupied_endpoints_xy_m=[[x, y] for x, y, _ in points])


def lead(offset=.05):
    return [(1.4, 0., offset)]


@pytest.fixture(autouse=True)
def no_region(monkeypatch):
    monkeypatch.setattr(ps, 'motion_region', lambda *a, **k: dict(distance_m=.6, queries=100, unobserved=0,
        masked_left=0, masked_right=0, masked_tail_extent_m=0., masked_front_samples=0))


def both():
    return MeasuredParallelTrial(CONFIG), FreshMemoryParallelTrial(CONFIG)


def step(trials, sc, speed=0., previous=0., wall='left'):
    return [t.apply(sc, base(wall), speed, previous, .02) for t in trials]


def stall_then_fresh(trials, side=-1., fresh_points=None, gap=.1):
    pts = neighbor(side*.2152)+lead()
    first = step(trials, scene(0., .10, pts))
    stale = step(trials, scene(0., .20, pts))
    nxt = step(trials, scene(gap, gap+.10, fresh_points if fresh_points is not None else pts))
    return first, stale, nxt


@pytest.mark.parametrize('side', [-1., 1.])
def test_age_only_block_keeps_one_fresh_confirmation_and_next_fresh_resumes(side):
    runtime, candidate = both()
    first, stale, nxt = stall_then_fresh((runtime, candidate), side)
    assert first[0][0] == first[1][0] == 0. and first[1][2]['reason'] == 'parallel_hold_release_wait'
    # 같은 스캔 재사용 틱: 두 정책 모두 0명령(적격성 false), 런타임만 확인 수를 지운다.
    assert stale[0][0] == stale[1][0] == 0.
    assert stale[0][2]['reason'] == stale[1][2]['reason'] == 'parallel_hold_unchanged_gap_hold'
    memory = stale[1][2]['fresh_confirmation_memory']
    assert memory['kept'] and memory['checks']['counterfactual_clears']
    # stall_then_fresh는 다음 fresh까지 실행한다: 런타임은 새로 1회, 후보는 기억 1회+새 1회.
    assert runtime.confirmations == 1 and candidate.confirmations == 2
    # 다음 fresh: 런타임은 다시 1회, 후보는 2회째로 재개.
    assert nxt[0][0] == 0. and nxt[1][0] > 0.


def test_duplicate_scan_never_increments_or_extends_memory():
    _, candidate = both()
    pts = neighbor()+lead()
    step([candidate], scene(0., .10, pts))
    for t in (.12, .14, .16, .20, .20, .21):
        out = step([candidate], scene(0., t, pts))[0]
        assert out[0] == 0. and candidate.confirmations <= 1
    assert candidate.confirmed['t_ref'] == .10
    # 확인 시각 +0.15 s 초과(0.27) 재사용은 기억 근거를 잃는다.
    out = step([candidate], scene(0., .27, pts))[0]
    assert out[0] == 0. and not out[2]['fresh_confirmation_memory']['kept'] and candidate.confirmations == 0


@pytest.mark.parametrize('fresh_points', [
    neighbor()+lead()+[(.35, 0., .05)],          # 새 전방 점유
    neighbor(-.165)+lead(),                      # 옆차 측방 접근(대각 접근의 다음 관측)
    neighbor(dx=.25)+lead()+[(.40, -.12, .05)],  # 옆차가 앞쪽으로 들어오며 경로 가장자리 점유
])
def test_next_fresh_rejection_clears_memory_immediately(fresh_points):
    runtime, candidate = both()
    _, stale, nxt = stall_then_fresh((runtime, candidate), fresh_points=fresh_points)
    assert stale[1][2]['fresh_confirmation_memory']['kept']
    assert nxt[1][0] == 0. and candidate.confirmations == 0 and candidate.confirmed is None
    assert nxt[0][0] == 0.


def test_moving_ego_geometry_change_is_not_age_only():
    _, candidate = both()
    step([candidate], scene(0., .10, neighbor()+lead()))
    moved = scene(0., .20, neighbor(dx=-.014)+[(1.386, 0., .05)])  # 0.35 m/s x 0.04 s 전진에 해당하는 점 이동
    out = step([candidate], moved)[0]
    memory = out[2]['fresh_confirmation_memory']
    assert out[0] == 0. and not memory['kept'] and not memory['checks']['same_points']
    assert candidate.confirmations == 0 and candidate.confirmed is None


@pytest.mark.parametrize('previous', [.006, -.02])
def test_steering_change_is_not_age_only(previous):
    _, candidate = both()
    step([candidate], scene(0., .10, neighbor()+lead()))
    out = candidate.apply(scene(0., .20, neighbor()+lead()), base(), 0., previous, .02)
    memory = out[2].get('fresh_confirmation_memory')
    assert out[0] == 0.
    assert memory is not None and not memory['kept'] and candidate.confirmations == 0


def test_wall_reference_change_clears():
    _, candidate = both()
    step([candidate], scene(0., .10, neighbor()+lead()))
    out = step([candidate], scene(0., .20, neighbor()+lead()), wall='right')[0]
    assert not out[2]['fresh_confirmation_memory']['kept'] and candidate.confirmations == 0


def test_different_raw_acquisition_times_are_not_the_same_scan():
    _, candidate = both()
    step([candidate], scene(0., .10, neighbor()+lead()))
    retimed = neighbor(offset=.021)+lead()
    out = step([candidate], scene(0., .20, retimed))[0]
    assert not out[2]['fresh_confirmation_memory']['checks']['same_raw_scan'] and candidate.confirmations == 0


def test_missing_scan_gap_clears_memory_before_next_fresh():
    runtime, candidate = both()
    _, stale, nxt = stall_then_fresh((runtime, candidate), gap=.30)
    assert stale[1][2]['fresh_confirmation_memory']['kept']
    assert nxt[1][0] == 0. and candidate.confirmations == 1   # 기억 없이 새로 1회


def test_non_age_rejection_on_same_scan_is_never_restored():
    _, candidate = both()
    pts = neighbor()+lead()
    step([candidate], scene(0., .10, pts))
    # 같은 스캔에서 조향이 달라진 거절(나이 전용 아님)이 나면 그 스캔의 확인 기록을 버린다.
    out = candidate.apply(scene(0., .20, pts), base(), 0., .03, .02)
    assert not out[2]['fresh_confirmation_memory']['kept'] and candidate.confirmed is None
    later = step([candidate], scene(0., .20, pts))[0]
    assert later[2]['fresh_confirmation_memory']['why'] == 'no_valid_fresh_confirmation_for_this_scan'
    assert later[0] == 0. and candidate.confirmations == 0


@pytest.mark.parametrize('event', ['epoch', 'reverse', 'reset'])
def test_epoch_reversal_and_reset_clear_like_runtime(event):
    _, candidate = both()
    pts = neighbor()+lead()
    step([candidate], scene(0., .10, pts))
    step([candidate], scene(0., .20, pts))
    assert candidate.confirmations == 1
    if event == 'epoch':
        out = step([candidate], scene(.1, .2, pts, epoch=1))[0]
    elif event == 'reverse':
        out = step([candidate], scene(0., .18, pts))[0]
    else:
        candidate.reset()
        out = step([candidate], scene(.1, .2, pts))[0]
    assert out[0] == 0. and candidate.confirmations == 1 and not candidate.remembered


def test_without_memory_events_candidate_matches_runtime_exactly():
    runtime, candidate = both()
    pts = neighbor(-.30)+lead()
    for k in range(12):
        source = .1*(k//5)
        a, b = step((runtime, candidate), scene(source, source+.10+.02*(k % 5), pts))
        assert a[:2] == b[:2] and a[2] == b[2]


def test_randomized_ineligible_always_zero_and_positive_only_from_eligible():
    rng = random.Random(7)
    _, candidate = both()
    for k in range(400):
        source = .1*(k//5)
        y = rng.choice([-.2152, -.2, -.17, -.3, .2152])
        extra = [(rng.uniform(.2, 1.2), rng.uniform(-.1, .1), .05)] if rng.random() < .15 else []
        pts = neighbor(y)+lead()+extra
        previous = rng.choice([0., 0., .002, .01])
        sc = scene(source, source+.10+.02*(k % 5), pts)
        out = candidate.apply(sc, base(), rng.choice([0., .1, .35]), previous, .02)
        trial = out[2]['parallel_hold_trial']
        if not trial['eligible']:
            assert out[0] == 0.
        if out[0] > 0:
            assert trial['eligible'] and out[0] == trial['target_speed_mps']
        if k % 37 == 0:
            candidate.reset()


def test_memory_tolerances_are_offline_candidate_values():
    assert (cs.GEOMETRY_POINT_TOLERANCE_M, cs.GEOMETRY_STEERING_TOLERANCE_RAD, cs.MEMORY_WINDOW_S) == (.005, .005, .15)
    assert CONFIG.confirmations == 2 and CONFIG.hold_clearance_m == .20 and CONFIG.resume_clearance_m == .28
