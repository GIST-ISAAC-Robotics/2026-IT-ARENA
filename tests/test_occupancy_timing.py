"""각 실제 취득 시각에 같은 운동/거리 상한을 적용한다. 점 삭제 없음."""
from types import SimpleNamespace
import numpy as np
import pytest

from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import swept_limit


def scene(now=1.2, times=(1., 1.08, 1.1), statuses=('return', 'masked', 'return')):
    return SimpleNamespace(t_ref_s=now, rays=[SimpleNamespace(acquired_at_s=t, status=s)
                                             for t, s in zip(times, statuses)])


def test_return_order_and_actual_age_preserved_without_new_points():
    assert per_ray_padding(scene(), .055, .5) == pytest.approx([.155, .105])
    assert len(per_ray_padding(scene(), .055, .5)) == 2
    assert FollowConfig().occupancy_timing_mode == 'oldest'
    with pytest.raises(ValueError): FollowConfig(occupancy_timing_mode='newest_for_all')


@pytest.mark.parametrize('fixed,bound,now,times', [(-1,.5,1.2,(1.,)), (.055,-1,1.2,(1.,)),
    (.055,.5,1.2,(1.3,)), (.055,.5,1.2,(float('nan'),)), (True,.5,1.2,(1.,)),
    (.055,.5,float('nan'),(1.,))])
def test_invalid_contract_or_future_point_rejected(fixed, bound, now, times):
    with pytest.raises(ValueError): per_ray_padding(scene(now, times, ('return',)), fixed, bound)


def test_equal_acquisition_times_exactly_match_old_scalar_sweep():
    points = np.array([[.6,.15],[.7,-.3],[1.,.4]])
    margins = per_ray_padding(scene(times=(1.,1.,1.), statuses=('return',)*3), .055, .5)
    for angle in (-.2,0.,.2):
        assert swept_limit(points, angle, margin=margins) == swept_limit(points, angle, margin=float(margins[0]))


def test_newer_unrelated_ray_never_refreshes_old_obstacle():
    points = np.array([[.30,.20],[2.,1.]])
    margins = per_ray_padding(scene(times=(1.,1.19), statuses=('return','return')), .055, .5)
    assert margins[0] == pytest.approx(.155)
    clearance = swept_limit(points, 0., margin=margins)
    assert clearance == swept_limit(points, 0., margin=float(margins[0]))
    assert clearance <= FollowConfig().hold_clearance_m
    assert swept_limit(points, 0., margin=float(margins[1])) > clearance


def test_each_rectangle_contains_same_bounded_motion_and_radial_error():
    rng = np.random.default_rng(22)
    ages = rng.uniform(0., .3, 1000)
    sample = scene(1., tuple(1.-ages), ('return',)*len(ages))
    margins = per_ray_padding(sample, .025+.03, .5)
    direction = rng.uniform(-np.pi,np.pi,len(ages))
    radial = rng.uniform(-.03,.03,len(ages))
    error = np.column_stack((np.cos(direction),np.sin(direction)))*radial[:,None]
    direction = rng.uniform(-np.pi,np.pi,len(ages))
    motion = np.column_stack((np.cos(direction),np.sin(direction)))*(.5*ages)[:,None]
    assert np.all(np.abs(error+motion) <= margins[:,None])
    assert np.all(margins <= .055+.5*max(ages)+1e-12)


def test_zero_motion_keeps_same_fixed_padding_for_every_return():
    assert per_ray_padding(scene(), .055, 0.) == pytest.approx([.055,.055])


@pytest.mark.parametrize('kwargs', [{'near':False}, {'far':False}, {'block':True}, {'unknown':True}])
def test_candidate_preserves_fragment_boundary_and_unknown_and_blocked_stops(kwargs):
    from test_fragmented_follow import observed, history
    from arena_autonomy.local_follow import LocalFollow
    follow = LocalFollow(FollowConfig(boundary_evidence_mode='fragmented_candidate',
                                     occupancy_timing_mode='per_ray_candidate'))
    for t in (.4,.5,.6,.7):
        assert follow.command(observed(t, **kwargs), history(), 0., 'left')[0] == 0


def test_candidate_keeps_all_occupied_points_and_reverse_stop():
    from test_fragmented_follow import observed, history
    from arena_autonomy.local_follow import LocalFollow
    follow = LocalFollow(FollowConfig(boundary_evidence_mode='fragmented_candidate',
                                     occupancy_timing_mode='per_ray_candidate'))
    for t in (.4,.5,.6,.7):
        result = follow.command(observed(t), history(), 0., 'left')
    assert result[0] > 0
    assert result[2]['unfiltered_occupancy']
    assert result[2]['occupancy_padding_points'] == result[2]['occupied_points'] == 270
    assert follow.command(observed(.8), history(), -.01, 'left')[0] == 0
