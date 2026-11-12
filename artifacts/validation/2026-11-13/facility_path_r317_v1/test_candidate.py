from pathlib import Path
import importlib.util, json, sys
from types import SimpleNamespace
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parent))
import geometry
import candidate
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig

@pytest.fixture(scope='module')
def inputs():
    p=Path(__file__).resolve().parents[1]/'facility_arc_drive_r314_v1/run1/follow_decisions.jsonl'
    with p.open() as f:
        head=json.loads(next(f))['data'];calls=[e['data'] for l in f if (e:=json.loads(l))['kind']=='call']
    cfg=FollowConfig(**head['follow_config']); mc=MotionConfig(**head['motion_config'])
    def get(t):
        c=min(calls,key=lambda c:abs(c['input']['reference_s']-t));d,s,h=restore_call(c['input'],mc)
        scene=motion_scene(s,h,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        return [np.asarray(c['output'][2]['path_points']),np.asarray(scene.occupied_endpoints_xy_m),
                geometry.selection_padding(scene,cfg),cfg,d['speed_mps'],scene.t_ref_s-scene.source_first_s],scene
    return get

@pytest.mark.parametrize('time',[21.8,21.9,22.0,22.2,22.4,22.8])
def test_observed_offset_keeps_occupancy_and_clears_steady_selection_budget(inputs,time):
    args,scene=inputs(time); before=[x.copy() for x in args[:3]]
    shifted,detail=geometry.choose(*args)
    assert detail['applied'] and detail['offset_m']<0
    assert detail['clearance_m']>=args[3].resume_clearance_m
    assert np.max(np.linalg.norm(shifted-args[0],axis=1))<=.060000001
    for x,y in zip(before,args[:3]):np.testing.assert_array_equal(x,y)

def test_mirrored_geometry_has_mirrored_offset(inputs):
    args,_=inputs(21.9); p,d=geometry.choose(*args)
    args[0]=args[0]*[1,-1];args[1]=args[1]*[1,-1]
    q,e=geometry.choose(*args)
    np.testing.assert_allclose(q,p*[1,-1]);assert e['offset_m']==pytest.approx(-d['offset_m'])

@pytest.mark.parametrize('time',[20.8,20.9,21.0,21.1])
def test_original_clear_arc_is_byte_preserved(inputs,time):
    args,_=inputs(time);p,d=geometry.choose(*args)
    assert not d['applied'];np.testing.assert_array_equal(p,args[0])

def test_opposing_wall_and_front_object_do_not_create_permission(inputs):
    args,_=inputs(21.9)
    args[1]=np.r_[args[1],[[0.,0.]]];args[2]=np.r_[args[2],.15]
    p,d=geometry.choose(*args);assert not d['applied'];np.testing.assert_array_equal(p,args[0])

@pytest.mark.parametrize('speed',[-.01,float('nan'),float('inf')])
def test_invalid_or_reverse_speed_is_not_new_route(inputs,speed):
    args,_=inputs(21.9);args[4]=speed;p,d=geometry.choose(*args);assert not d['applied']

def test_missing_endpoints_is_not_free(inputs):
    args,_=inputs(21.9);args[1]=np.empty((0,2));args[2]=np.empty(0)
    p,d=geometry.choose(*args);assert not d['applied']

def test_forecast_only_adds_margin_and_is_stable_through_scan(inputs):
    args,s=inputs(21.9);cfg=args[3]
    from dataclasses import replace
    p=geometry.selection_padding(s,cfg)
    for dt in (.01,.02):
        later=replace(s,t_ref_s=s.t_ref_s+dt)
        q=geometry.selection_padding(later,cfg)
        assert np.all(q>=p-1e-12)
        assert np.all(q>=geometry.per_ray_padding(later,.025+cfg.measurement_padding_m,cfg.object_motion_bound_mps))

def test_traffic_mode_does_not_take_solo_candidate(inputs):
    args,s=inputs(21.9)
    owner=SimpleNamespace(config=FollowConfig())
    p,d=candidate.execution_path(owner,s,args[0],args[4],args[5],None)
    assert not d;np.testing.assert_array_equal(p,args[0])

def test_installer_is_single_baseline_and_preserves_prior_uninstall():
    import arena_autonomy.local_follow as lf
    original=lf.LocalFollow._baseline_command
    candidate.install()
    assert lf.LocalFollow._baseline_command.__globals__['pursue_path'] is lf.pursue_path
    candidate.uninstall();assert lf.LocalFollow._baseline_command is original
