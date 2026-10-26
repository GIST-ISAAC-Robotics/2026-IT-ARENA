import math

import numpy as np
import pytest
from shapely.geometry import box

from traffic_merge import (MergeProfile, merge_geometry, merge_pose, scheduled_merge_pose,
                           steering_bound, validate_merge_route, merge_profile_for_case,
                           audit_merge_motion)


P = MergeProfile(.35, .8, 0.)


class Straight:
    def pose(self, s): return (s, 0., 0.)


def test_endpoints_are_flat_and_initial_pose_matches_original_slot():
    for distance, lateral in [(0., .13), (.35, .13), (1.15, 0.), (2., 0.)]:
        g = merge_geometry(distance, .13, P)
        assert g['rear_lateral_m'] == pytest.approx(lateral)
        assert g['slope'] == pytest.approx(0., abs=1e-12)
        assert g['curvature_m_inv'] == pytest.approx(0., abs=1e-12)
        assert merge_pose((2., 3., math.pi/2), distance, .13, P) == pytest.approx((2.-lateral, 3.+distance, math.pi/2))


def test_zero_longitudinal_speed_has_no_sideways_motion_or_rotation():
    schedule = [dict(from_s=0., to_s=2., speed_mps=.35), dict(from_s=2., to_s=5., speed_mps=0.)]
    poses = [scheduled_merge_pose(Straight(), 1., schedule, t, .13, P) for t in (2., 3., 4.9)]
    assert poses[0] == poses[1] == poses[2]
    assert poses[0][2] != 0  # 전이 중 멈춰도 자세가 돌아가지 않는다.


def test_rear_axle_tangent_matches_heading_and_continuous_steering_bound():
    for offset in (-.13, .13):
        bound = steering_bound(offset, P)
        assert bound < .37
        for distance in np.linspace(.351, 1.149, 61):
            x, y, yaw = merge_pose((0., 0., 0.), distance, offset, P)
            rear = (x-P.wheelbase_m/2*math.cos(yaw), y-P.wheelbase_m/2*math.sin(yaw))
            g = merge_geometry(distance, offset, P)
            assert rear == pytest.approx((distance-P.wheelbase_m/2, g['rear_lateral_m']))
            e = 1e-6
            derivative = (merge_geometry(distance+e, offset, P)['rear_lateral_m']-
                          merge_geometry(distance-e, offset, P)['rear_lateral_m'])/(2*e)
            assert math.tan(yaw) == pytest.approx(derivative, abs=1e-8)
            assert abs(g['steering_rad']) <= bound


@pytest.mark.parametrize('kwargs', [dict(transition_m=0.), dict(start_distance_m=-1.),
    dict(wheelbase_m=float('nan')), dict(max_steering_rad=True), dict(final_offset_m=.3)])
def test_invalid_profile_cannot_be_silently_clamped(kwargs):
    values = dict(start_distance_m=.35, transition_m=.8, final_offset_m=0.); values.update(kwargs)
    with pytest.raises(ValueError): MergeProfile(**values)


def route_fixture():
    config = {'actor': {'length_m': .2, 'width_m': .15}}
    case = {'route_kind': 'merge_straight', 'merge_profile': vars(P), 'actor_lateral_offset_m': .13,
            'duration_s': 4., 'schedule': [dict(from_s=0., to_s=4., speed_mps=.35)]}
    return config, case


@pytest.mark.parametrize('value', [None, '0.1', True, float('nan'), float('inf'), -.1])
def test_bad_distance_is_rejected(value):
    with pytest.raises(ValueError): merge_geometry(value, .13, P)


def evaluation_fixture():
    config, case = route_fixture()
    case.update(name='stop_restart', duration_s=6., schedule=[
        dict(from_s=0., to_s=4., speed_mps=.35), dict(from_s=4., to_s=6., speed_mps=0.)])
    config['merge_acceptance'] = dict(minimum_lateral_change_m=.11, final_offset_error_max_m=.005,
        moving_heading_error_max_rad=.05, static_heading_error_max_rad=.005)
    rows = [(float(t), *scheduled_merge_pose(Straight(), .3, case['schedule'], float(t), .13, P))
            for t in np.linspace(0., 6., 601)]
    return config, case, rows


def test_merge_must_be_explicit_and_is_primary_only():
    _, case, _ = evaluation_fixture()
    assert merge_profile_for_case(case) == P
    with pytest.raises(ValueError, match='explicit'):
        merge_profile_for_case({**case, 'route_kind': 'straight'})
    with pytest.raises(ValueError, match='one'):
        merge_profile_for_case({**case, 'additional_actors': [{}]})
    assert merge_profile_for_case({'name': 'stop_restart'}) is None


def test_actual_actor_audit_rejects_no_merge_missing_stream_bad_heading_and_wall():
    config, case, rows = evaluation_fixture()
    road = box(-1., -.225, 3., .225)
    def audit(poses, obstacles=()):
        return audit_merge_motion(config, case, Straight(), .3, 0., poses, road, obstacles)
    assert audit(rows)['valid']
    flat = [(t, x, .13, 0.) for t, x, y, yaw in rows]
    assert not audit(flat)['checks']['lateral_motion_observed']
    bad_yaw = [(t, x, y, yaw+.06) for t, x, y, yaw in rows]
    assert not audit(bad_yaw)['checks']['moving_heading_matches']
    assert not audit(rows[10:])['checks']['pose_coverage']
    assert not audit(rows, [box(.5, .1, .7, .2)])['checks']['actor_avoids_walls']


def test_primary_dispatch_preserves_existing_routes_and_tracks_merge():
    import validate_g1_traffic as runner
    config, case, _ = evaluation_fixture()
    config['actor']['lateral_offset_m'] = 0.
    assert runner.primary_pose_at(config, Straight(), .3, case, 2.) == scheduled_merge_pose(
        Straight(), .3, case['schedule'], 2., .13, P)
    old_case = {k: v for k, v in case.items() if k not in ('route_kind', 'merge_profile')}
    for t in (-1., 0., 2., 4., 6.):
        assert runner.primary_pose_at(config, Straight(), .3, old_case, t) == runner.actor_pose_at(
            Straight(), .3, case['schedule'], t, .13)


def test_primary_merge_config_is_explicit_and_matches_actual_starting_slot():
    import validate_g1_traffic as runner
    cfg = runner.load_config(runner.REPO/'config/tests/m2_merge_stop_restart_v1.json')
    line, scene = runner.load_centerline()
    case = cfg['cases'][0]; start = runner.actor_start_s(cfg, scene, 0, case)
    assert runner.primary_pose_at(cfg, line, start, case, 0.) == pytest.approx(
        runner.actor_pose_at(line, start, case['schedule'], 0., .13))


def test_route_checks_whole_rotated_body_and_rejects_wall_or_tight_steering():
    config, case = route_fixture(); straight = lambda *args: {'actor_start_s_m': .3, 'actor_end_s_m': 1.7}
    valid = validate_merge_route(config, Straight(), case, .3, box(-1., -.225, 3., .225), [], straight)
    assert valid['sample_count'] >= 280 and valid['minimum_sampled_road_edge_m'] > 0.
    with pytest.raises(ValueError, match='footprint'):
        validate_merge_route(config, Straight(), case, .3, box(-1., -.225, 3., .225), [box(.5, .1, .7, .2)], straight)
    case['merge_profile'] = {**vars(P), 'transition_m': .2}
    with pytest.raises(ValueError, match='steering'):
        validate_merge_route(config, Straight(), case, .3, box(-1., -.225, 3., .225), [], straight)
