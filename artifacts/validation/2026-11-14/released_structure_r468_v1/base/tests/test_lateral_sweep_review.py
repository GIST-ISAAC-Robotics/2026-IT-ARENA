"""측방 대조가 현재 좌표·동일 시각·원래 결측을 지키는지 확인한다. 주행 시험 아님."""
from dataclasses import replace
import math
import numpy as np
import pytest

from arena_autonomy.lateral_intent import ObservationGrid, PathFrame, build_context, offset_candidate
from arena_autonomy.lateral_sweep_review import interpolate_pose, synchronized_sweep, review_sweep
from test_lateral_intent import GRASS, straight


def trajectory(poses, progress):
    return {'poses': np.asarray(poses, dtype=float), 'progress': np.asarray(progress, dtype=float)}


def test_same_time_difference_is_not_whole_reference_union():
    scene, _ = straight(0.)
    grid = ObservationGrid(scene, GRASS)
    # 이 모듈은 geometry/timing 진단이므로 두 경로의 특정 시각 자세를 직접 공급한다.
    candidate = trajectory([(0, 0, 0), (.5, 0, 0), (1, 0, 0)], [0, 1, 2])
    reference = trajectory([(0, 0, 0), (1, 0, 0), (2, 0, 0)], [0, 1, 2])
    out = synchronized_sweep(grid, candidate, reference, 1., GRASS, horizon_s=2.)
    x, y = grid.centers()
    prior_visited = (abs(x-.975) < .006) & (abs(y) < .0051)
    assert np.any(out['extra_masks'][-1] & prior_visited)
    union = np.logical_or.reduce(out['reference_masks'])
    assert np.all(union[prior_visited])  # 예전 전체 합집합 차감이라면 이 진입을 놓친다.


def test_forward_travel_is_kept_and_equal_paths_have_no_extra():
    scene, path = straight(-.09)
    frame = PathFrame(path)
    ctx = build_context(scene, frame, .35, GRASS)
    hold = offset_candidate(frame, ctx['state'], -.09, GRASS)
    out = synchronized_sweep(ctx['grid'], hold, hold, .35, GRASS)
    assert out['candidate_poses'][-1, 0] == pytest.approx(.175)
    assert not np.isfinite(out['first_extra']).any()
    assert not np.logical_or.reduce(out['extra_masks']).any()


def test_heading_interpolation_handles_pi_seam_and_rejects_extrapolation():
    poses = np.array([[0., 0., math.pi-.1], [1., 0., -math.pi+.1]])
    middle = interpolate_pose(poses, [0., 1.], [.5])
    assert middle[0, 2] == pytest.approx(math.pi)
    with pytest.raises(ValueError):
        interpolate_pose(poses, [0., 1.], [1.1])


def test_extra_observation_uses_actual_directional_support_without_relabeling_grid():
    scene, path = straight(-.09)
    frame = PathFrame(path)
    ctx = build_context(scene, frame, .35, GRASS)
    hold = offset_candidate(frame, ctx['state'], -.09, GRASS)
    center = offset_candidate(frame, ctx['state'], 0., GRASS)
    before = ctx['grid'].state.copy()
    result = review_sweep(scene, ctx['grid'], center, hold, ctx['hidden_seeds_xy'], .35, GRASS)
    assert result['extra_cells'] > 0 and result['extra_unknown'] == 0
    assert result['extra_hidden'] > 0 and not result['control_authorization']
    assert np.array_equal(before, ctx['grid'].state)
    masked, _ = straight(-.09, dropout=lambda a: (a >= 50) & (a <= 150))
    badctx = build_context(masked, frame, .35, GRASS)
    bad = review_sweep(masked, badctx['grid'], center, hold, badctx['hidden_seeds_xy'], .35, GRASS)
    assert bad['extra_unknown'] > 0


def test_hidden_reach_is_checked_at_later_occupancy_not_only_first_visit():
    scene, path = straight(0.)
    frame = PathFrame(path)
    ctx = build_context(scene, frame, .2, GRASS)
    hold = offset_candidate(frame, ctx['state'], 0., GRASS)
    # 같은 경로에서도 후방 상대가 미래에 따라붙는 위험은 존재하며, 추가 횡이동과 구분한다.
    result = review_sweep(scene, ctx['grid'], hold, hold, ctx['hidden_seeds_xy'], .2, GRASS)
    assert result['reference_hidden'] > 0 and result['candidate_hidden'] > 0
    assert result['extra_hidden'] == 0


def test_horizon_truncation_is_explicit_and_not_extrapolated():
    scene, path = straight(0.)
    frame = PathFrame(path)
    ctx = build_context(scene, frame, .35, GRASS)
    hold = offset_candidate(frame, ctx['state'], 0., GRASS)
    short = {'progress': hold['progress'][:2], 'poses': hold['poses'][:2]}
    result = synchronized_sweep(ctx['grid'], hold, short, .35, GRASS)
    assert result['truncated_horizon'] and result['horizon_s'] < .5
    assert result['candidate_poses'][-1, 0] <= .025+1e-9
