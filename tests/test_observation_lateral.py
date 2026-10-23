from dataclasses import replace

import numpy as np
import pytest
from shapely.geometry import Point, Polygon

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.observation_lateral import protected_queries, polygon_distance, reachable_from_unknown
from arena_autonomy.observation_support import depth_support, sweep_support
from audit_observation_history import lateral_support
from test_corridor_visibility import scene
from test_linear_observation import unknown_at


@pytest.mark.parametrize('polygon', [
    [(0.,0.), (1.,0.), (1.,1.), (0.,1.)],
    [(0.,0.), (0.,1.), (1.,1.), (1.,0.)],
    [(.1,.2), (.9,.1), (.7,.8)],
])
def test_polygon_distance_matches_independent_geometry(polygon):
    q = np.random.default_rng(17).uniform(-.3, 1.3, (100,2))
    expected = [Polygon(polygon).distance(Point(p)) for p in q]
    np.testing.assert_allclose(polygon_distance(q, polygon), expected, atol=1e-12)


def test_degenerate_distance_does_not_mark_entire_plane_inside():
    np.testing.assert_allclose(polygon_distance([[0.,0.], [.5,1.]], [(0.,0.),(1.,0.)]), [0.,1.])


def test_protected_scope_is_sensor_relative_and_does_not_claim_rear():
    q = [[.1,0.], [.06,.1], [-.06,-.08]]
    assert protected_queries(q, (.06,0.,0.)).tolist() == [True,True,False]
    assert protected_queries(q, (.06,0.,np.pi)).tolist() == [False,True,True]


@pytest.mark.parametrize('observation', [scene(), unknown_at(90), unknown_at(60), replace(unknown_at(90),t_ref_s=.55)])
def test_vectorized_reachability_matches_independent_shapely_audit(observation):
    cfg = FollowConfig()
    q = np.random.default_rng(3).uniform([-.15,-.2], [.7,.3], (150,2))
    pad = np.array([.025+cfg.measurement_padding_m+cfg.object_motion_bound_mps*(observation.t_ref_s-r.acquired_at_s)
                    for r in observation.rays])
    depth = depth_support(observation,q,pad)
    _, oracle, _ = lateral_support(observation,q,cfg)
    np.testing.assert_array_equal(reachable_from_unknown(observation,q,cfg) & depth, oracle)


def test_scoped_candidate_keeps_rear_limit_explicit_and_front_unknown_hold():
    cfg = FollowConfig()
    result = sweep_support([scene()],0.,.2,0.,0.,cfg,protected_sensor_pose=(.06,0.,0.))
    assert result['supported']
    assert result['rear_dynamic_risk_unresolved'] is True
    assert result['unknown_filled'] is False and result['dynamic_cut_in_verified'] is False
    result = sweep_support([unknown_at(0),scene()],0.,.2,0.,0.,cfg,protected_sensor_pose=(.06,0.,0.))
    assert not result['supported']
