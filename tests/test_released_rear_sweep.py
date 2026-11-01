"""작은 후반 무반환과 현재 장애물/상태 경계를 분리한다."""
from dataclasses import replace
import math
import numpy as np
import pytest
from arena_autonomy.released_rear_sweep import bounded_released_rear_sweep
from arena_autonomy.local_follow import FollowConfig
from test_rear_fringe_trial import case
import test_traffic_release_fringe as linked


def probe(*,point=(-.047,.0765),speed=.2,target=.35,limit=.01,**kwargs):
    old,raw,scene=case(point=point,**kwargs)
    result=bounded_released_rear_sweep(scene,raw,[point],np.full(500,.1),.2,.15,speed,target,limit)
    return old,result,raw,scene


def test_one_mm_original_is_unchanged_and_new_risk_is_explicit():
    old,result,raw,scene=probe()
    assert not old['accepted'] and result['accepted']
    assert not result['unknown_filled'] and result['residual_hidden_object_risk']
    assert scene.rays[451].status=='unknown' and math.isinf(raw.ranges[451])
    assert result['maximum_exterior_m']==.01


@pytest.mark.parametrize('kwargs',[
    {'point':(.001,.0765)},{'point':(-.101,.0765)},{'point':(-.047,.085001)},
    {'point':(-.047,.075)},{'speed':.351},{'speed':-.01},{'speed':math.nan},
    {'target':.351},{'target':0.},{'limit':.010001},{'limit':-1.},
    {'value':math.nan},{'value':-math.inf},{'value':.02},{'neighbor':.19},{'neighbor':.4},
    {'missing':tuple(range(449,455))},{'mismatch':True},
])
def test_forward_large_fast_close_invalid_and_wide_cases_are_rejected(kwargs):
    assert not probe(**kwargs)[1]['accepted']


def test_reflection_and_zero_speed_recovery():
    assert probe(point=(-.047,-.0765),missing=(48,49),speed=0.)[1]['accepted']


def test_config_off_and_explicit_dependency():
    assert not FollowConfig().released_rear_sweep_trial
    with pytest.raises(ValueError,match='released rear sweep'):
        FollowConfig(released_rear_sweep_trial=True)
    with pytest.raises(ValueError,match='released rear sweep'):
        FollowConfig(released_rear_sweep_trial=1)


def execute(monkeypatch,*,enabled=True,proof=True,**kwargs):
    original=linked.ps.PassReleaseSelector
    def selector(config):return original(replace(config,released_rear_sweep_trial=enabled))
    monkeypatch.setattr(linked.ps,'PassReleaseSelector',selector)
    output=linked.output
    def with_proof(*args,**kw):
        v,a,d=output(*args,**kw)
        return v,a,{**d,'partial_observation':dict(allowed=proof,reason='fully_observed')}
    monkeypatch.setattr(linked,'output',with_proof)
    return linked.execute(monkeypatch,point=(-.047,.0765),**kwargs)


def test_only_released_retained_low_speed_has_new_candidate(monkeypatch):
    ctl,out=execute(monkeypatch)
    assert ctl.mode=='RELEASED' and out[0]==.35
    assert out[2]['selector_reason']=='released_bounded_rear_sweep'
    assert out[2]['selector_released_rear_sweep_risk_accepted']
    assert not out[2]['selector_bounded_rear_fringe_risk_accepted']
    assert out[2]['selector_region']['unobserved']==1


@pytest.mark.parametrize('kwargs',[
    {'enabled':False},{'proof':False},{'present':True},{'retained':False,'target':.35},
    {'target':0.,'baseline_reason':'follow_gap_hold'},
    {'target':0.,'no_path':True,'baseline_reason':'follow_no_path'},
    {'speed':.36},{'front_mask':1},{'tail_extent':.01001},{'neighbor':.19},
])
def test_no_new_permission_for_current_threat_or_stop(monkeypatch,kwargs):
    assert execute(monkeypatch,**kwargs)[1][0]==0.


def test_evaluation_default_and_explicit_command():
    import g1_handoff_eval as ev
    launch=dict(actuation_mode='virtual_mcu',autonomy_mode='local_pursuit',speed_bump=False,
        lidar_rate_hz=10.,lidar_compensation='both',lidar_acquisition='sequential',
        speed_profile='g1_open_1p5',control_rate_hz=50.)
    assert 'evaluation_handoff_rear_sweep:=true' not in ev.launch_arguments('g1g2','paced',launch,True)
    assert 'evaluation_handoff_rear_sweep:=true' in ev.launch_arguments('g1g2','paced',launch,True,rear_sweep=True)
    with pytest.raises(ValueError):ev.launch_arguments('g1',None,launch,True,rear_sweep=True)
