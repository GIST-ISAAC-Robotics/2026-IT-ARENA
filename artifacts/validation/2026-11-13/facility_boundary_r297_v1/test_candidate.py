"""기존 실제 반환 경로 후보 연결의 장애물 유지·선택 경계. 합성 입력 검증."""
from dataclasses import replace
import numpy as np
import pytest
import candidate
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from test_local_follow import history,scene,settle

@pytest.fixture(autouse=True)
def installed():
    candidate.install()
    yield
    candidate.uninstall()

def controller():
    return LocalFollow(FollowConfig(solo_route_only=True,narrow_occluder_bridge=True,
        piecewise_wall_trial=True,standing_wall_reacquire=True,
        boundary_evidence_mode='corner_distance_candidate',
        occupancy_timing_mode='per_ray_candidate'))

def test_clear_then_front_obstacle_still_stops():
    c=controller();h=history();assert settle(c,h)[0]>0
    s=scene(.8,obstacle_x=.14);before=s.rays
    out=c.command(s,h,.1,'left')
    assert out[0]==0 and s.rays==before

def test_unknown_front_and_expired_input_still_stop():
    for kind in ('unknown','stale'):
        c=controller();h=history();assert settle(c,h)[0]>0
        s=scene(.8)
        if kind=='unknown':
            s=replace(s,rays=tuple(replace(r,status='unknown',hit_xy_m=None) if abs(np.arctan2(r.direction_xy[1],r.direction_xy[0]))<.4 else r for r in s.rays))
        else:s=scene(.8,source=.1)
        assert c.command(s,h,.1,'left')[0]==0

def test_option_off_does_not_invoke_extra_fragment_solver(monkeypatch):
    from arena_autonomy import fragmented_wall_hypothesis as module
    def unexpected(*a,**k):raise AssertionError('option OFF fragment query')
    monkeypatch.setattr(module,'propose',unexpected)
    for changes in ({'solo_route_only':False},{'narrow_occluder_bridge':False}):
        cfg=replace(controller().config,**changes);c=LocalFollow(cfg)
        assert settle(c,history())[0]>0
