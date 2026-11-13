"""긴 선호 경로의 공통 관측 끝까지만 사용하는 선택형 직각 후보 회귀."""
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from arena_autonomy import corner_corridor as cc
from arena_autonomy.continuous_prefix import refine
from arena_autonomy.corridor_observer import current_proposal
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import LocalScene, SensorRay

ROOT = Path(__file__).resolve().parents[1]


def config(enabled=True):
    return replace(FollowConfig(), wall_offset_m=.505,road_half_m=.305,piecewise_wall_trial=enabled,
        boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate',
        corridor_fallback_enabled=True,short_boundary_prefix_enabled=True)


def line(a,b,y=0.,n=36):
    return np.c_[np.linspace(a,b,n),np.full(n,y)]


@pytest.mark.parametrize('side',['left','right'])
def test_recorded_long_preferred_short_opposite_is_clipped(side):
    clouds=json.loads((ROOT/'artifacts/validation/2026-11-07/piecewise_corner_v1/corner_after.clouds.json').read_text())
    points=np.array(clouds[-1]['calls'][0]['points'])
    if side=='right':points[:,1]*=-1
    assert cc.short_prefix_evidence(points,side,config())['reason']=='not_a_short_boundary'
    candidate=refine(cc.short_prefix_evidence(points,side,config(),allow_long_preferred=True),config())
    assert candidate['supported'] and candidate['long_preferred_short_opposite']
    assert .25<candidate['shared_span_m']<1.0
    assert 0<candidate['base_observed_ahead_m']<.65
    assert not candidate['unknown_filled'] and not candidate['opposite_path_used_for_steering']
    assert np.max(np.array(candidate['path'])[:,0])<1.0


@pytest.mark.parametrize('kind',['good','far','disagreement','anchor','both_long','origin','gap'])
def test_common_prefix_boundaries(monkeypatch,kind):
    own,other=line(-.2,1.4),line(.14,.60)
    if kind=='far':other=line(.40,.95)
    if kind=='disagreement':other=line(.14,.60,.15)
    if kind=='origin':own,other=line(-.2,1.4,.20),line(.14,.60,.20)
    if kind=='gap':other=np.vstack((line(.12,.17),[[.20,.30],[.55,.30]],line(.60,.95)))
    def path(points,side,offset):
        p=own if side=='left' else other
        return p,dict(wall_distance_m=.75 if kind=='anchor' and side=='right' else .505,
            boundary_observed_ahead_m=1.4 if side=='left' or kind=='both_long' else .46,path_points=p.tolist())
    monkeypatch.setattr(cc,'local_path',path)
    out=refine(cc.short_prefix_evidence([[0.,0.]],'left',config(),allow_long_preferred=True),config())
    assert out['supported']==(kind=='good')
    if kind=='good':
        assert out['base_observed_ahead_m']==.46
        assert np.max(np.asarray(out['path'])[:,0])<.70


@pytest.mark.parametrize('kind',['clear','obstacle','unknown','fast'])
def test_connected_proposal_keeps_current_occupancy_and_short_budget(kind):
    clouds=json.loads((ROOT/'artifacts/validation/2026-11-07/piecewise_corner_v1/corner_after.clouds.json').read_text())
    points=np.array(clouds[-1]['calls'][0]['points'])
    if kind=='obstacle':points=np.vstack((points,[[.15,0.]]))
    rays=[]
    for i,p in enumerate(points):
        direction=p-[.06,0.];direction/=np.linalg.norm(direction)
        rays.append(SensorRay((.06,0.),tuple(p),.3,'return','laser_frame',0,i,tuple(direction)))
    if kind=='unknown':rays.append(SensorRay((.06,0.),None,.3,'unknown','laser_frame',0,len(rays),(1.,0.)))
    scene=LocalScene(.4,.3,.3,'laser_frame',0,tuple(rays))
    assert not current_proposal(scene,0.,'left',0.,.02,config(False))['point_occupancy_supported']
    out=current_proposal(scene,2. if kind=='fast' else 0.,'left',0.,.02,config())
    assert out['point_occupancy_supported']==(kind=='clear')
    if kind=='clear':
        assert 0<out['proposed_speed_mps']<=.35
        assert out['stop_budget_m']<=out['path_budget_m']<.40
        assert not out['memory_refresh_authorized'] and not out['free_area_certified']
