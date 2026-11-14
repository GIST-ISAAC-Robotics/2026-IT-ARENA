"""직선 외벽 재관측은 현재 반환의 범위와 잡음을 따르고 기억으로 빈 면을 만들지 않는다."""
from dataclasses import replace
import math
import numpy as np
import pytest
from arena_autonomy.bypass_wall_fit import tracked_straight_wall
from arena_autonomy.bypass_entry_trial import EntryGuide,current_outer_wall_padding,observed_straight_wall
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
from arena_autonomy.scene_motion import motion_scene
from probe_bypass_entry_sequence import render_scan
from bypass_probe_sensors import ProbeSensors


def angled_scene(noise=False):
    x,y,yaw=.7,.18,.28
    rear=[x-.0675*math.cos(yaw),y-.0675*math.sin(yaw),yaw]
    physical=[dict(t=t,rear_pose=rear,speed=0.,steering=0.) for t in (0.,.6)]
    scan=render_scan(physical,.6,0.,False,ProbeSensors('range' if noise else 'ideal'))
    history=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06))
    for t in np.arange(.2,.61,.01):history.add_mean_wheel(float(t),0.);history.add_gyro(float(t),0.)
    scene=motion_scene(scan,history,.6,timing_verified=True,rear_half_angle_deg=30.)
    xx=np.array([-.2,1.5]);line=np.c_[xx,-math.tan(yaw)*xx-y/math.cos(yaw)]
    return scene,line


@pytest.mark.parametrize('noise',[False,True])
def test_current_wall_crossing_forward_axis_is_not_cut_by_side_sign(noise):
    scene,line=angled_scene(noise);before=scene.rays;cfg=FollowConfig()
    fit,wall=tracked_straight_wall(scene,cfg,'left',line)
    assert fit is not None and fit[3]>1.2
    assert abs(math.atan(fit[0])+.28)<.02
    assert abs(fit[1]+.18/math.cos(.28))<.015
    assert scene.rays==before and wall['all_returns_remain_occupied']
    accepted=set(wall['source_ray_indices'])
    assert any(r.source_index in accepted and r.hit_xy_m[1]<.1 for r in scene.rays if r.status=='return')
    old,_=observed_straight_wall(scene,cfg,'left')
    assert old is None or old[3]<fit[3]-.3
    padding,detail=current_outer_wall_padding(scene,cfg,'left',wall)
    assert len(padding)==len(scene.occupied_endpoints_xy_m)
    assert detail['static_return_count']>=12
    ids=[r.source_index for r in scene.rays if r.status=='return']
    assert all(p>=.055 for p in padding)
    assert all(ids[i] in accepted for i,p in enumerate(padding) if p==.055)


def test_missing_current_wall_cannot_be_reconstructed_from_memory():
    scene,line=angled_scene();cfg=FollowConfig()
    fit,wall=tracked_straight_wall(scene,cfg,'left',line)
    removed=set(wall['source_ray_indices'])
    scene=replace(scene,rays=tuple(replace(r,status='unknown',hit_xy_m=None) if r.source_index in removed else r for r in scene.rays))
    assert tracked_straight_wall(scene,cfg,'left',line)[0] is None


def test_separate_short_pieces_are_not_a_continuous_wall():
    scene,line=angled_scene();cfg=FollowConfig()
    scene=replace(scene,rays=tuple(replace(r,status='unknown',hit_xy_m=None) if r.source_index%3==0 else r for r in scene.rays))
    assert tracked_straight_wall(scene,cfg,'left',line)[0] is None


def test_large_wall_relocation_does_not_reuse_previous_wall_identity():
    scene,line=angled_scene()
    moved=line+np.array([0.,.12])
    assert tracked_straight_wall(scene,FollowConfig(),'left',moved)[0] is None


def test_tracked_fit_is_only_an_explicit_stationary_wall_trial():
    assert not EntryGuide().tracked_wall_fit
    with pytest.raises(ValueError):EntryGuide(tracked_wall_fit=True)
    assert EntryGuide(stationary_outer_boundary=True,wall_reanchor=True,tracked_wall_fit=True).tracked_wall_fit
