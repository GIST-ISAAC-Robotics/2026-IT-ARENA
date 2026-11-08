from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest
from arena_autonomy.follow_trace import FollowConfig, MotionConfig, restore_call
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_scene import SensorRay
from loader import load

A=Path(__file__).resolve().parent
m=load()

def fixture():
    f=json.loads((A/'fixture.json').read_bytes())
    return f,np.asarray(f['current']['path']),np.asarray(f['reference']),FollowConfig(**f['config'])

def baseline(expired=False, initial=False, obstacle=False):
    f,path,ref,cfg=fixture()
    data,scan,h=restore_call(f['input'],MotionConfig(**f['motion_config']))
    scene=motion_scene(scan,h,data['reference_s'],timing_verified=data['timing_verified'],
        epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
    if obstacle:
        rays=list(scene.rays)
        for i,y in enumerate(np.linspace(-.09,.09,20)):
            point=np.array([.13,y]);origin=np.array([.06,0.]);direction=point-origin;direction/=np.linalg.norm(direction)
            rays.append(SensorRay(tuple(origin),tuple(point),scene.source_first_s,'return','laser_frame',0,1000+i,tuple(direction)))
        scene=replace(scene,rays=tuple(rays))
    c=m.LocalFollow(cfg);c.epoch=scene.source_epoch;c.side=data['side'];c.last_ref=scene.t_ref_s-.02;c.last_source=scene.source_first_s-.1
    if not initial:
        c.path=ref;c.path_ref=scene.t_ref_s;c.path_source=scene.t_ref_s-.31 if expired else f['prior']['source'];c.path_wall=f['prior']['wall']
    c.held=False
    out=c._baseline_command(scene,h,data['speed_mps'],data['side'],data['previous_steering_rad'],data['dt_s'],allow_partial=True)
    return out,c

def test_current_prefix_replaces_memory_without_extending_the_observed_end():
    f,p,r,cfg=fixture()
    assert .5 < f['current']['ahead'] < cfg.boundary_min_ahead_m
    assert m.traffic_corner_prefix_agreement(p,r,f['current']['info'],cfg) is not None
    out,c=baseline()
    assert out[0]>0 and out[2]['boundary_selection']=='current_corner_prefix_agreement'
    assert out[2]['boundary_from_memory'] is False
    assert out[2]['observed_path_ahead_m']<.65
    np.testing.assert_allclose(c.path,p,atol=1e-10)

def test_short_current_wall_does_not_initialize_or_revive_expired_memory():
    for kw in ({'initial':True},{'expired':True}):
        out,_=baseline(**kw)
        assert out[0]==0 and out[2]['reason']=='follow_boundary_unconfirmed'

def test_actual_close_obstacle_still_stops_with_current_prefix():
    out,_=baseline(obstacle=True)
    assert out[0]==0
    assert out[2]['boundary_selection']=='current_corner_prefix_agreement'
    assert out[2]['swept_clearance_m']<=.25

@pytest.mark.parametrize('flag',['traffic_boundary_refresh_trial','piecewise_wall_trial','short_boundary_prefix_enabled'])
def test_disabled_feature_does_not_change_the_contract(flag):
    f,p,r,cfg=fixture()
    assert m.traffic_corner_prefix_agreement(p,r,f['current']['info'],replace(cfg,**{flag:False})) is None

def test_mirrored_current_corner_has_same_evidence():
    f,p,r,cfg=fixture()
    assert m.traffic_corner_prefix_agreement(p*[1,-1],r*[1,-1],f['current']['info'],cfg) is not None

def test_conflicting_corner_and_insufficient_common_extent_are_rejected():
    f,p,r,cfg=fixture()
    for other in (p+[0,.3],p[:4]):
        assert m.traffic_corner_prefix_agreement(other,r,f['current']['info'],cfg) is None

def test_new_long_path_and_other_models_keep_the_original_route():
    f,p,r,cfg=fixture()
    for update in ({'boundary_observed_ahead_m':.9},{'wall_model':'straight'},{'path_anchor_error_m':.3}):
        assert m.traffic_corner_prefix_agreement(p,r,{**f['current']['info'],**update},cfg) is None
