"""선택형 평행 경로의 측정속도 범위 보완 반례. 장면은 합성 센서이며 실제 차량 검증이 아니다."""
import sys, math
from pathlib import Path
from dataclasses import replace
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests'))
import arena_autonomy.parallel_lane_trial as candidate
from arena_autonomy.parallel_lane_trial import ParallelLaneTrial
import zipfile, types
_old=types.ModuleType('parallel_before_r24')
with zipfile.ZipFile(ROOT/'artifacts/validation/2026-11-08/rear_overtake_continuity_connected_v1/before.zip') as _z:
    _text=_z.read('src/arena_autonomy/arena_autonomy/parallel_lane_trial.py').decode('utf-8')
exec(compile(_text,'parallel_before_r24.py','exec'),_old.__dict__)
original=_old.proposal
import arena_autonomy.parallel_lane_trial as module
from test_parallel_lane_trial import config,base,scene
from test_parallel_side_departure import scene as side_scene,cfg as side_config
from test_g1_partial_policy import corridor
from test_local_follow import history
from arena_autonomy.scene_motion import motion_scene

@pytest.mark.parametrize('speed',[0.,.3,.7,1.])
@pytest.mark.parametrize('obstacle',[None,.25])
def test_nominal_outputs_unchanged(speed,obstacle):
    s=scene(obstacle=obstacle)
    assert candidate.proposal(s,base(),speed,0.,.02,config())==original(s,base(),speed,0.,.02,config())

@pytest.mark.parametrize('speed',[1.021,1.1,1.25,1.55])
def test_real_ray_scene_above_old_range_uses_actual_speed(speed):
    s=scene();before=np.array(s.occupied_endpoints_xy_m)
    r=candidate.proposal(s,base(),speed,0.,.02,config())
    assert r['eligible'],r
    assert 0<r['target_speed_mps']<=.35
    assert r['measured_speed_mps']==speed
    assert r['measured_distance_m']==pytest.approx(speed*.22+speed*speed/4)
    assert r['measured_region']['unobserved']==0
    np.testing.assert_array_equal(s.occupied_endpoints_xy_m,before)

@pytest.mark.parametrize('speed',[1.1,1.55])
def test_observed_front_obstacle_remains_stop(speed):
    r=candidate.proposal(scene(obstacle=.25),base(),speed,0.,.02,config())
    assert not r['eligible']

@pytest.mark.parametrize('speed',[3.,1e100,-1.,math.nan,math.inf])
def test_invalid_or_outside_numerical_sweep_stays_rejected(speed):
    assert not candidate.proposal(scene(),base(),speed,0.,.02,config())['eligible']

def test_actual_path_must_cover_measured_stop():
    b=list(base()); b[2]['path_points']=np.c_[np.linspace(-.1,.45,40),np.zeros(40)].tolist()
    r=candidate.proposal(scene(),b,1.55,0.,.02,config())
    assert not r['eligible'] and r['reason']=='measured_stop_space_insufficient'

def test_real_unknown_front_rays_not_filled_by_positive_baseline():
    raw=corridor(.5,missing=range(242,259))
    s=motion_scene(raw,history(),.6,timing_verified=True)
    b=list(base()); b[0]=1.
    r=candidate.proposal(s,b,1.1,0.,.02,config())
    assert not r['eligible'] and r['reason']=='measured_region_unobserved'

@pytest.mark.parametrize('angle',[-.03,.03])
def test_high_measured_turning_still_checks_lateral_motion(angle):
    r=candidate.proposal(scene(),base(),1.55,angle,.02,config(pass_release_trial=True))
    assert not r['eligible'] and r['reason']=='lateral_motion_not_small'

@pytest.mark.parametrize('speed',[0.,.4,.8])
def test_existing_side_departure_not_dropped(speed):
    cfg=side_config();s=side_scene()
    assert candidate.proposal(s,base(),speed,0.,.02,cfg)==original(s,base(),speed,0.,.02,cfg)

def test_two_fresh_confirmations_preserved(monkeypatch):
    monkeypatch.setattr(module,'proposal',candidate.proposal)
    c=ParallelLaneTrial(config())
    assert c.apply(scene(.5),base(),1.1,0.,.02)[0]==0
    assert c.apply(scene(.5),base(),1.1,0.,.02)[0]==0
    assert c.apply(scene(.6),base(),1.1,0.,.02)[0]>0
