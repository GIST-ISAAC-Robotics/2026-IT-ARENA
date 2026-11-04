"""두 벽과 실제 전방 시야가 서로 대신될 수 없는지 보는 제한 반례."""
import math
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'tests')]
from arena_autonomy.local_follow import FollowConfig, pursue_path
from arena_autonomy.local_scene import scene_from_scan
from straight_budget import estimate
from test_ray_wall_continuity import cast, RIGHT, ROOM, WALL, ORIGIN

CFG=FollowConfig(solo_route_only=True,blend_open_speed_by_length=True,open_route_speed_mps=10/3.6,
                 standstill_gap_m=.05,headway_s=.5)
PATH=np.column_stack((np.linspace(-.08,1.75,36),np.zeros(36)))


def fixture(segments=None):
    points,ids=cast([WALL,RIGHT,*ROOM] if segments is None else segments)
    ranges=np.full(500,np.nan); ranges[ids]=np.linalg.norm(points-ORIGIN,axis=1)
    scene=scene_from_scan(ranges.tolist(),angle_min=-math.pi,angle_increment=math.tau/500,
                          range_min=.05,range_max=12.,acquired_at_s=(.5+np.arange(500)*.0002).tolist(),t_ref_s=.65)
    speed,ahead=1.7,PATH.tolist()
    cap,angle,remaining=pursue_path(PATH,speed,CFG,.15,speed_cap=CFG.open_route_speed_mps)
    detail=dict(reason='local_follow',open_route_speed_enabled=True,boundary_from_memory=False,
                partial_observation={'reason':'fully_observed'},path_points=ahead,path_source_s=.5,
                observed_search_horizon_m=2.977,observed_path_ahead_m=remaining,swept_clearance_m=2.977)
    return (cap,angle,detail),scene


def test_straight_corridor_lifts_only_speed():
    out,scene=fixture()
    speed,why=estimate(out,scene,0.,1.7,CFG)
    assert why=='straight_budget' and out[0]<speed<=CFG.open_route_speed_mps
    assert out[1]==0 and np.array_equal(out[2]['path_points'],PATH)


def test_missing_forward_ray_or_hit_never_uses_clear_sidewalls():
    out,s=fixture();rays=list(s.rays)
    rays[250]=replace(rays[250],status='unknown',hit_xy_m=None)
    assert estimate(out,replace(s,rays=tuple(rays)),0.,1.7,CFG)==(out[0],'partial')
    out,s=fixture([WALL,RIGHT,*ROOM,(1.85,-.06,1.85,.06)])
    # 낙관적인 과거 점유 거리 값을 넣어도 현재 반환 검사가 독립적으로 거부한다.
    assert estimate(out,s,0.,1.7,CFG)==(out[0],'observed_hit_remains')


def test_recess_then_aligned_far_wall_never_bridges_lateral_discontinuity():
    walls=[(-1.,.4,1.3,.4),(2.,.4,5.,.4), RIGHT,*ROOM]
    out,scene=fixture(walls)
    assert estimate(out,scene,0.,1.7,CFG)==(out[0],'no_longer_budget')


def test_previous_actual_steer_must_fit_both_sides_not_just_forward_centerline():
    out,scene=fixture()
    assert estimate(out,scene,.14,1.7,CFG)==(out[0],'arc_leaves_observed_sides')


@pytest.mark.parametrize('edit',[{'boundary_from_memory':True},{'corridor_selected':True},
    {'partial_observation':{'reason':'bounded_local'}},{'reason':'follow_release_wait'}])
def test_limited_or_recovery_output_is_not_lifted(edit):
    out,scene=fixture();v,angle,d=out
    assert estimate((v,angle,{**d,**edit}),scene,0.,1.7,CFG)==(v,'original_state')


def test_stored_path_turning_into_curve_is_not_eligible():
    out,scene=fixture();v,angle,d=out
    curve=PATH.copy();curve[-5:,1]=np.linspace(0,.16,5)
    assert estimate((v,angle,{**d,'path_points':curve}),scene,0.,1.7,CFG)==(v,'existing_path_curved')
