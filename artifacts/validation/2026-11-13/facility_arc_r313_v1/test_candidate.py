from pathlib import Path
import json,sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parent))
import candidate as target
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import FollowConfig,per_ray_padding
from arena_autonomy.lidar_motion import MotionConfig

@pytest.fixture(scope='module')
def scene_inputs():
    p=Path(__file__).resolve().parents[1]/'occluder_pass_drive_r310_v1/run1/follow_decisions.jsonl'
    with p.open() as f:
        header=json.loads(next(f))['data'];calls=[r['data'] for l in f if (r:=json.loads(l))['kind']=='call']
    cfg=FollowConfig(**header['follow_config']);mc=MotionConfig(**header['motion_config'])
    def get(t):
        call=min(calls,key=lambda c:abs(c['input']['reference_s']-t));d,scan,h=restore_call(call['input'],mc)
        scene=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        return np.asarray(call['output'][2]['path_points']),np.asarray(scene.occupied_endpoints_xy_m),per_ray_padding(scene,.025+cfg.measurement_padding_m,cfg.object_motion_bound_mps),cfg,d['speed_mps'],call['output'][1]
    return get

@pytest.mark.parametrize('time',[21.8,22.,24.])
def test_nearby_arc_preserves_obstacle_padding_and_reference(scene_inputs,time):
    args=scene_inputs(time);before=[v.copy() if isinstance(v,np.ndarray) else v for v in args]
    angle,proof=target.choose(*args)
    assert proof['applied'] and 0<angle<args[-1]
    assert proof['clearance_m']>=args[3].resume_clearance_m
    assert proof['path_error_m']<=args[3].boundary_agreement_m
    for a,b in zip(args[:3],before[:3]):np.testing.assert_array_equal(a,b)

@pytest.mark.parametrize('time',[26.,30.])
def test_no_escape_when_route_and_occupancy_cannot_both_hold(scene_inputs,time):
    args=scene_inputs(time);angle,proof=target.choose(*args)
    assert angle==args[-1] and not proof['applied']

@pytest.mark.parametrize('speed',[-.01,.251,1.,float('nan')])
def test_low_speed_scope_only(scene_inputs,speed):
    path,pts,pad,cfg,_,desired=scene_inputs(22.)
    angle,proof=target.choose(path,pts,pad,cfg,speed,desired)
    assert angle==desired and not proof['applied']

def test_current_footprint_occupied_stays_blocked(scene_inputs):
    path,pts,pad,cfg,speed,desired=scene_inputs(22.)
    pts=np.r_[pts,[[0.,0.]]];pad=np.r_[pad,.055]
    angle,proof=target.choose(path,pts,pad,cfg,speed,desired)
    assert angle==desired and not proof['applied']

def test_mirrored_turn_has_mirrored_choice(scene_inputs):
    path,pts,pad,cfg,speed,desired=scene_inputs(22.)
    a,p=target.choose(path,pts,pad,cfg,speed,desired)
    b,q=target.choose(path*[1,-1],pts*[1,-1],pad,cfg,speed,-desired)
    assert b==pytest.approx(-a) and q['clearance_m']==p['clearance_m']

def test_no_change_for_clear_original(scene_inputs):
    args=list(scene_inputs(21.2));args[-2]=0.
    a,p=target.choose(*args)
    assert a==args[-1] and not p['applied'] and p['reason']=='original_clear'

def test_no_obstacles_is_not_free_space(scene_inputs):
    path,_,_,cfg,speed,desired=scene_inputs(22.)
    a,p=target.choose(path,np.empty((0,2)),np.empty(0),cfg,speed,desired)
    assert a==desired and not p['applied']
