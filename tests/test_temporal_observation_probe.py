from dataclasses import replace

from arena_autonomy.local_follow import FollowConfig
from probe_temporal_observation import temporal_support, region_queries
from test_linear_observation import unknown_at
from test_corridor_visibility import scene


def test_previous_finite_evidence_is_distinct_from_filling_current_unknown():
    current=unknown_at(0)
    r=temporal_support([current,scene()],[[.4,0.]],FollowConfig())
    assert r['new_support_from_earlier']==1
    assert r['all_supported'] and not r['free_area_certified']
    assert current.rays[250].status=='unknown' and current.rays[250].hit_xy_m is None


def test_unknown_in_all_sources_and_close_returns_cannot_support():
    for observations in ([unknown_at(0),unknown_at(0)],[scene(.3),scene(.3)]):
        r=temporal_support(observations,[[.4,0.]],FollowConfig())
        assert not r['all_supported'] and r['supported']==0


def test_thin_side_unknown_appears_even_if_grid_misses_it():
    q,thin=region_queries(unknown_at(90),.175417,.43,.012566370614359173,FollowConfig())
    assert len(q)>10 and thin>=1
