"""운용 정책 변화와 유지할 고장 처리를 구분한다. 자유 공간 증명 아님."""
import json
import math

import numpy as np
import pytest

from arena_autonomy.partial_observation import PartialObservationPolicy, short_observed_surface
from arena_autonomy.lidar_safety import LidarSafety
from test_local_follow import history, scan
from test_lidar_safety import node_double


def sample(first=.5, indices=(), value=math.inf):
    raw = scan(first, [8.]*500)
    for i in indices:
        raw.ranges[i] = value
    return raw


def warmed(points=(), forward=False):
    policy, h = PartialObservationPolicy(allow_narrow_forward=forward), history()
    assert policy.evaluate(sample(.4), points, h, .5, 1.)['allowed']
    return policy, h


@pytest.mark.parametrize('indices', [range(295,299), range(122,126), [374], [140,141,360,361]])
def test_small_bounded_side_gap_allows_capped_progress_without_filling(indices):
    p,h = warmed()
    raw = sample(indices=indices)
    original = list(raw.ranges)
    result = p.evaluate(raw, [], h, .6, 1.)
    assert result['allowed'] and result['speed_cap_mps'] == .5
    assert not result['unknown_filled'] and result['residual_hidden_object_risk']
    assert raw.ranges == original


@pytest.mark.parametrize('indices', [[250], range(250,253), range(295,300),
                                    range(500), list(range(130,134))+list(range(340,344))+[370]])
def test_central_wide_and_distributed_failure_remain_stops(indices):
    p,h = warmed()
    assert not p.evaluate(sample(indices=indices), [], h, .6, 1.)['allowed']


def test_close_return_at_gap_edge_blocks_even_though_other_edge_is_far():
    p,h = warmed()
    raw=sample(indices=[374])
    raw.ranges[373]=.15
    assert p.evaluate(raw, [], h, .6, 1.)['reason']=='near_gap_boundary'


@pytest.mark.parametrize('value',[.01,0.,-1.,-math.inf])
def test_under_range_return_is_not_a_permitted_side_dropout(value):
    p,h=warmed()
    assert p.evaluate(sample(indices=[375],value=value),[],h,.6,1.)['reason']=='invalid_near_return'


def test_known_obstacle_disappearance_does_not_expire_into_permission():
    p,h = warmed([[.06,.15]])
    raw=sample(indices=[375])
    assert p.evaluate(raw, [], h, .6, 1.)['reason']=='known_obstacle_latched'
    assert not p.evaluate(sample(.7,[375]), [], h, .8, 1.)['allowed']
    assert not p.evaluate(sample(1.,[375]), [], h, 1.1, 1.)['allowed']
    assert not p.evaluate(sample(1.1), [], h, 1.2, 1.)['allowed']
    assert not p.evaluate(sample(1.1), [], h, 1.22, 1.)['allowed']  # same scan != recovery
    assert p.evaluate(sample(1.2), [], h, 1.3, 1.)['allowed']


def test_side_wall_outside_stopping_sweep_is_not_a_disappeared_close_threat():
    p,h = warmed([[.06,.5]])
    assert p.evaluate(sample(indices=[375]), [], h, .6, 1.)['allowed']


def test_outside_trial_speed_stops_without_allocating_a_long_sweep():
    p,h=warmed()
    assert p.evaluate(sample(indices=[375]),[],h,.6,1e100)['reason']=='partial_speed_outside_trial'


def test_no_previous_observation_is_not_free_space_and_stale_history_latches():
    h=history()
    assert not PartialObservationPolicy().evaluate(sample(indices=[375]), [], h, .6,1.)['allowed']
    p,h=warmed()
    assert p.evaluate(sample(.9,[375]),[],h,1.,1.)['reason']=='partial_history_expired'
    assert not p.evaluate(sample(1.),[],h,1.1,1.)['allowed']


def candidate_node(forward=False):
    node,commands,statuses=node_double()
    node.partial_policy=PartialObservationPolicy(allow_narrow_forward=forward)
    node.command.drive.speed=1.
    node.motion.history=history()
    node.motion.project=lambda raw:(np.full((60,2),20.),60)
    node.scan=sample(.4)
    node.command.header.stamp.sec=0
    node.command.header.stamp.nanosec=500_000_000
    node.get_clock=lambda:type('Clock',(),{'now':lambda _:type('Now',(),{
        'nanoseconds':500_000_000,'to_msg':lambda _:node.command.header.stamp})()})()
    LidarSafety.control(node)
    node.scan=sample(.5,[375])
    node.command.header.stamp.nanosec=600_000_000
    node.get_clock=lambda:type('Clock',(),{'now':lambda _:type('Now',(),{
        'nanoseconds':600_000_000,'to_msg':lambda _:node.command.header.stamp})()})()
    return node,commands,statuses


def test_independent_gate_caps_request_not_scan_and_preserves_zero_command():
    node,commands,statuses=candidate_node()
    LidarSafety.control(node)
    assert commands[-1].drive.speed==.5
    report=json.loads(statuses[-1].data)
    assert report['front_observation']['reason']=='front_unobserved'
    assert report['partial_observation']['allowed']
    node.command.drive.speed=0.
    LidarSafety.control(node)
    assert commands[-1].drive.speed==0.


def test_reverse_feedback_stops_output_but_does_not_expire_live_observations():
    node,commands,statuses=candidate_node()
    # 0.4 s 역회전 값 동안에도 스캔과 엔코더/자이로 시각은 정상적으로 갱신된다.
    node.motion.history=history(v=-.01)
    for first in (.5,.6,.7,.8,.9):
        now=first+.1
        node.scan=sample(first,[375])
        node.command.header.stamp.sec=int(now)
        node.command.header.stamp.nanosec=round((now-int(now))*1e9)
        node.get_clock=lambda now=now:type('Clock',(),{'now':lambda _:type('Now',(),{
            'nanoseconds':round(now*1e9),'to_msg':lambda _:node.command.header.stamp})()})()
        LidarSafety.control(node)
        report=json.loads(statuses[-1].data)
        assert commands[-1].drive.speed==0.
        assert report['reason']=='invalid_or_reverse_feedback'
        assert report['partial_observation']['allowed']
        assert node.partial_policy.previous[1]==pytest.approx(now)
    node.motion.history=history(v=0.)
    LidarSafety.control(node)
    assert commands[-1].drive.speed==.5
    assert json.loads(statuses[-1].data)['partial_observation']['latch_reason'] is None


def test_latch_diagnostic_keeps_first_cause_until_confirmed_recovery():
    p,h=warmed([[.06,.15]])
    first=p.evaluate(sample(indices=[375]),[],h,.6,1.)
    assert first['latch_reason']=='previous_point_in_missing_sweep'
    assert first['latch_time_s']==.6
    later=p.evaluate(sample(.6,[375]),[],h,.7,1.)
    assert later['latch_reason']==first['latch_reason']
    assert later['latch_time_s']==.6


def test_still_visible_point_is_not_treated_as_disappeared():
    p,h=warmed([[.06,.15]])
    result=p.evaluate(sample(indices=[375]),[[.06,.15]],h,.6,1.)
    assert result['allowed'] and result['reason']=='bounded_side_missing'
    assert result['reobserved_previous_points']==1 and result['latch_evidence'] is None
    # 다음 새 스캔에서 그 점이 실제로 사라지면 여전히 정지한다.
    lost=p.evaluate(sample(.6,[375]),[],h,.7,1.)
    assert not lost['allowed'] and lost['latch_reason']=='previous_point_in_missing_sweep'


def test_same_scan_rechecks_old_unmatched_point_after_steering_changes():
    p,h=warmed([[.2,.4]])
    raw=sample(.5,[338])
    first=p.evaluate(raw,[],h,.6,1.,0.,0.)
    assert first['allowed']
    second=p.evaluate(raw,[],h,.62,1.,.37,0.)
    assert not second['allowed'] and second['reason']=='known_obstacle_latched'
    assert second['latch_evidence']['previous_source_first_s']==pytest.approx(.4)
    assert second['latch_evidence']['source_first_s']==pytest.approx(.5)


@pytest.mark.parametrize('surface',[False,True])
def test_current_close_obstacle_still_brakes_when_reobserved_match_is_removed_from_loss_check(surface):
    node,commands,statuses=candidate_node(forward=True)
    node.partial_policy.surface_translation_enabled=surface
    node.scan=sample(.5,[250])
    point=np.tile([.1,0.],(60,1))
    node.partial_policy.previous=(point,.5,.4)
    node.motion.project=lambda _: (point,60)
    LidarSafety.control(node)
    report=json.loads(statuses[-1].data)
    assert report['partial_observation']['allowed']
    assert report['reason']=='lidar_braking' and commands[-1].drive.speed < .2


@pytest.mark.parametrize('fault',['near_obstacle','stale_command','stale_scan','transport',
                                 'missing_motion','reverse_motion','invalid_scan','central_gap'])
def test_partial_policy_does_not_bypass_independent_faults(fault):
    node,commands,statuses=candidate_node()
    if fault=='near_obstacle':node.motion.project=lambda _:(np.tile([.1,0.],(60,1)),60)
    if fault=='stale_command':node.command.header.stamp.nanosec=0
    if fault=='stale_scan':node.scan=sample(.1,[375])
    if fault=='transport':node.scan_wall-=4.
    if fault=='missing_motion':node.motion.history=history(end=.1)
    if fault=='reverse_motion':node.motion.history=history(v=-.1)
    if fault=='invalid_scan':node.scan.angle_increment=0.
    if fault=='central_gap':node.scan.ranges[250]=math.nan
    LidarSafety.control(node)
    assert commands[-1].drive.speed==0.


@pytest.mark.parametrize('indices',[[250],[249,250],[283]])
@pytest.mark.parametrize('value',[math.inf,math.nan,13.])
def test_forward_candidate_keeps_narrow_unknown_but_limits_speed(indices,value):
    p,h=warmed(forward=True)
    raw=sample(indices=indices,value=value)
    original=np.asarray(raw.ranges).copy()
    result=p.evaluate(raw,[],h,.6,1.)
    assert result['allowed'] and result['reason']=='bounded_forward_missing'
    assert result['speed_cap_mps']==.2 and result['unknown_filled'] is False
    np.testing.assert_array_equal(raw.ranges,original)


@pytest.mark.parametrize('count',[3,4])
def test_forward_gap_has_stricter_width_than_side_gap(count):
    p,h=warmed(forward=True)
    result=p.evaluate(sample(indices=range(250,250+count)),[],h,.6,1.)
    assert result['reason']=='forward_gap_too_wide' and not result['allowed']
    assert p.previous[1]==.6  # 실제 수신한 관측은 남긴다.


@pytest.mark.parametrize('neighbor',[.22,.5,.999])
def test_forward_gap_needs_distant_real_neighbors(neighbor):
    p,h=warmed(forward=True)
    raw=sample(indices=[250]);raw.ranges[249]=neighbor
    result=p.evaluate(raw,[],h,.6,1.)
    assert result['reason']=='forward_gap_near_boundary' and not result['allowed']


def test_forward_hidden_known_object_remains_latched_after_it_disappears():
    p,h=warmed([[.4,0.]],forward=True)
    first=p.evaluate(sample(indices=[250]),[],h,.6,1.)
    assert first['reason']=='known_obstacle_latched'
    assert first['latch_reason']=='previous_point_in_missing_sweep'
    assert not p.evaluate(sample(.6,[250]),[],h,.7,1.)['allowed']
    assert not p.evaluate(sample(.7),[],h,.8,1.)['allowed']
    assert p.evaluate(sample(.8),[],h,.9,1.)['allowed']


def test_refused_forward_width_keeps_fresh_observations_for_later_narrow_gap():
    p,h=warmed(forward=True)
    for first in (.5,.6,.7,.8,.9):
        result=p.evaluate(sample(first,range(249,252)),[],h,first+.1,1.)
        assert result['reason']=='forward_gap_too_wide'
    assert p.evaluate(sample(1.,[250]),[],h,1.1,1.)['allowed']


def test_wide_blind_interval_does_not_recover_from_one_narrow_gap():
    p,h=warmed(forward=True)
    assert p.evaluate(sample(.8,range(500)),[],h,.9,1.)['reason']=='wide_or_many_missing'
    result=p.evaluate(sample(.9,[250]),[],h,1.,1.)
    assert result['reason']=='partial_history_expired' and not result['allowed']


@pytest.mark.parametrize('fault',['near_obstacle','stale_command','stale_scan','missing_motion','reverse_motion','invalid_scan'])
def test_forward_candidate_preserves_independent_fault_stops(fault):
    node,commands,statuses=candidate_node(forward=True)
    node.scan=sample(.5,[250])
    if fault=='near_obstacle':node.motion.project=lambda _:(np.tile([.1,0.],(60,1)),60)
    if fault=='stale_command':node.command.header.stamp.nanosec=0
    if fault=='stale_scan':node.scan=sample(.1,[250])
    if fault=='missing_motion':node.motion.history=history(end=.1)
    if fault=='reverse_motion':node.motion.history=history(v=-.1)
    if fault=='invalid_scan':node.scan.angle_increment=0.
    LidarSafety.control(node)
    assert commands[-1].drive.speed==0.


def test_forward_candidate_caps_independent_output_and_preserves_zero():
    node,commands,statuses=candidate_node(forward=True)
    node.scan=sample(.5,[250])
    LidarSafety.control(node)
    assert commands[-1].drive.speed==.2
    node.command.drive.speed=0.
    LidarSafety.control(node)
    assert commands[-1].drive.speed==0.


def test_surface_match_compares_adjacent_observed_samples_not_only_sample_locations():
    current=np.array([[.2,.4],[.245,.4],[.29,.4]])
    result=short_observed_surface(np.array([.222,.405]),current,[100,101,102])
    assert result is not None and result['normal_distance_m']==pytest.approx(.005)
    assert short_observed_surface(np.array([.222,.43]),current,[100,101,102]) is None
    assert short_observed_surface(np.array([.32,.4]),current,[100,101,102]) is None


@pytest.mark.parametrize('ids',[[100,102,103],[100,101,103],[100,100,101]])
def test_surface_match_never_bridges_unknown_or_duplicate_ray_indices(ids):
    assert short_observed_surface(np.array([.222,.4]),[[.2,.4],[.245,.4],[.29,.4]],ids) is None


@pytest.mark.parametrize('points',[
    [[.2,.4],[.28,.4],[.36,.4]],  # 긴 현
    [[.2,.4],[.245,.4],[.245,.445]],  # 코너/서로 다른 물체 가능
    [[.2,.4],[.245,.4],[.22,.4]],  # 반전
    [[.2,.4],[.245,.4]],  # 두 점만으로는 받지 않는다
])
def test_surface_match_requires_short_consistent_three_return_shape(points):
    assert short_observed_surface(np.array([.222,.4]),points,list(range(len(points)))) is None
