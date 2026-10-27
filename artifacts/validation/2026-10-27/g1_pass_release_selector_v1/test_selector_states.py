"""시제품의 상태 계약 반례. 주입 view는 상태 분기만 검증하며 센서 인식 증명이 아니다."""
import math
from types import SimpleNamespace
import numpy as np
import pytest
import pass_release_selector as ps
from arena_autonomy.local_follow import FollowConfig


def config():
    return FollowConfig(partial_observation_policy='bounded_local_v4',
        boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate')


def output(speed=1., angle=0., reason='local_follow'):
    return speed, angle, {'reason': reason, 'path_points': [[0., 0.], [1., 0.]]}


def view(present=False, rear=.25):
    return {'frame':None, 'state':None, 'left':{'present':present,'points':4 if present else 0,
        'rearmost_ds_m':rear if present else None,'frontmost_ds_m':rear+.2 if present else None},
        'right':{'present':False,'points':0,'rearmost_ds_m':None,'frontmost_ds_m':None}}


@pytest.fixture
def setup(monkeypatch):
    state={'view':view(), 'band':{'free':True,'samples':10,'observed_free':10,'masked':0,'unobserved':0}}
    monkeypatch.setattr(ps,'threat_view',lambda *a:state['view'])
    monkeypatch.setattr(ps,'band_check',lambda *a:state['band'])
    ctl=ps.PassReleaseSelector(config())
    def call(t, *,speed=.3, previous=0., dt=.02, epoch=0, base=None, parallel=None):
        scene=SimpleNamespace(source_first_s=t-.1,t_ref_s=t,source_epoch=epoch)
        return ctl.select(scene,output() if base is None else base,
            output(.2) if parallel is None else parallel,speed,previous,dt)
    return ctl,state,call


def test_never_seen_solo_does_not_wait_for_impossible_pass(setup):
    ctl,state,call=setup
    assert call(1.)[0]==1. and ctl.mode=='SOLO'


def test_two_distinct_clear_scans_after_forward_observation_allow_release(setup):
    ctl,state,call=setup;state['view']=view(True,.25)
    assert call(1.)[0]==.2
    state['view']=view()
    assert call(1.1)[0]==.2
    assert call(1.1)[0]==.2
    assert call(1.2)[0]==1. and ctl.mode=='RELEASED'


def test_disappearance_behind_never_becomes_forward_pass(setup):
    ctl,state,call=setup;state['view']=view(True,-.2);call(1.)
    state['view']=view()
    for t in (1.1,1.2,1.3):out=call(t)
    assert ctl.sides['left']['vanish']=='rear'
    assert ctl.mode=='GUARD' and out[0]==.2


def test_second_object_passing_does_not_erase_first_unresolved_disappearance(setup):
    ctl,state,call=setup;state['view']=view(True,-.2);call(1.)
    state['view']=view();call(1.1)
    state['view']=view(True,.25);call(1.2)
    state['view']=view();call(1.3);out=call(1.4)
    assert ctl.mode=='GUARD' and out[0]==.2


def test_remaining_observed_object_prevents_release(setup):
    ctl,state,call=setup;state['view']=view(True,-.2)
    for t in (1.,1.1,1.2):out=call(t)
    assert ctl.mode=='GUARD' and out[0]==.2


def test_reapproaching_object_reenters_guard(setup):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();call(1.1);call(1.2)
    state['view']=view(True,-.15)
    assert call(1.3)[0]==.2 and ctl.mode=='GUARD'


@pytest.mark.parametrize('fault',['nan_speed','reverse','nan_previous','bad_dt','nan_baseline','reverse_parallel',
    'inf_steering','excess_baseline','excess_previous','zero_dt'])
def test_invalid_motion_or_output_must_hold_even_in_solo(setup,fault):
    ctl,state,call=setup
    kwargs={'nan_speed':{'speed':math.nan},'reverse':{'speed':-.01},
        'nan_previous':{'previous':math.nan},'bad_dt':{'dt':.3},
        'nan_baseline':{'base':output(math.nan)},'reverse_parallel':{'parallel':output(-.1)},
        'inf_steering':{'base':output(angle=math.inf)},'excess_baseline':{'base':output(1.01)},
        'excess_previous':{'previous':.38},'zero_dt':{'dt':0.}}[fault]
    out=call(1.,**kwargs)
    assert out[0]==0. and math.isfinite(out[1])


@pytest.mark.parametrize('fault',['rewind','epoch','reset'])
def test_time_discontinuity_never_turns_seen_threat_into_solo(setup,fault):
    ctl,state,call=setup;state['view']=view(True,-.2);call(1.)
    state['view']=view()
    if fault=='reset':ctl.invalidate('sensor_reset');out=call(1.1)
    elif fault=='rewind':out=call(.8)
    else:out=call(1.1,epoch=1)
    assert out[0]<=.2
    for t in (1.2,1.3,1.4):out=call(t,epoch=1 if fault=='epoch' else 0)
    assert ctl.mode=='GUARD' and out[0]<=.2


def test_unobserved_band_cannot_clear_seen_side(setup):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();state['band'].update(free=False,unobserved=1)
    for t in (1.1,1.2,1.3):out=call(t)
    assert ctl.mode=='GUARD' and out[0]==.2


def test_front_stop_stays_stop_after_release(setup):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();call(1.1);call(1.2)
    assert call(1.3,base=output(0.,reason='follow_gap_hold'))[0]==0.


def test_fault_reason_cannot_fall_back_to_positive_parallel(setup):
    ctl,state,call=setup;state['view']=view(True,-.2);call(1.)
    assert call(1.1,base=output(0.,reason='follow_scene_stale'))[0]==0.


def test_clear_confirmation_interrupted_by_no_path(setup):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();call(1.1)
    state['view']=None;call(1.2)
    state['view']=view();out=call(1.3)
    assert ctl.mode=='GUARD' and out[0]==.2
    assert call(1.4)[0]==1.


@pytest.mark.parametrize('age',[.3,.31,-.01,math.nan])
def test_invalid_time_age_preserves_unresolved_threat(setup,age):
    ctl,state,call=setup;state['view']=view(True,-.2);call(1.)
    scene=SimpleNamespace(source_first_s=2.-age,t_ref_s=2.,source_epoch=0)
    out=ctl.select(scene,output(),output(.2),.3,0.,.02)
    assert out[0]==0. and ctl.mode=='GUARD'


def test_large_motion_with_unobserved_region_falls_back(setup,monkeypatch):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();call(1.1);call(1.2)
    monkeypatch.setattr(ps,'motion_region',lambda *a:{'unobserved':1,'masked_left':0,'masked_right':0})
    out=call(1.3,base=output(angle=.2))
    assert out[0]==.2 and out[2]['selector_reason']=='large_motion_region_unobserved'


def test_previous_steer_counts_when_new_command_straight(setup,monkeypatch):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();call(1.1);call(1.2)
    monkeypatch.setattr(ps,'motion_region',lambda *a:{'unobserved':1,'masked_left':0,'masked_right':0})
    out=call(1.3,previous=.2)
    assert out[0]==.2 and out[2]['selector_reason']=='large_motion_region_unobserved'


def test_finite_reverse_holds_without_inventing_opposite_side_unknown(setup):
    ctl,state,call=setup;state['view']=view(True,-.1);call(1.)
    assert call(1.1,speed=-.05,base=output(0.,reason='follow_reverse_motion'))[0]==0.
    assert not ctl.sides['right']['seen']
    state['view']=view(True,.25);call(1.2)
    state['view']=view();call(1.3)
    assert call(1.4)[0]==1. and ctl.mode=='RELEASED'


def test_finite_reverse_does_not_erase_unresolved_rear_object(setup):
    ctl,state,call=setup;state['view']=view(True,-.1);call(1.)
    state['view']=view();call(1.1)
    assert call(1.2,speed=-.05)[0]==0.
    state['view']=view(True,.25);call(1.3)
    state['view']=view();call(1.4);out=call(1.5)
    assert ctl.mode=='GUARD' and out[0]==.2


@pytest.mark.parametrize('masked',['masked_left','masked_right'])
def test_large_move_mask_is_not_free_even_on_previously_passed_or_unseen_side(setup,monkeypatch,masked):
    ctl,state,call=setup;state['view']=view(True,.25);call(1.)
    state['view']=view();call(1.1);call(1.2)
    region={'unobserved':0,'masked_left':0,'masked_right':0};region[masked]=1
    monkeypatch.setattr(ps,'motion_region',lambda *a:region)
    assert call(1.3,base=output(angle=.2))[0]==.2


@pytest.mark.parametrize('angle',[-.37,-.12,.001,.12,.37])
def test_arc_contains_dense_corner_coordinate_extrema(angle):
    cfg=config();distance=.53
    poses=ps.extrema_arc(angle,distance,ps.lateral_config(cfg))
    k=math.tan(angle)/cfg.wheelbase_m
    theta=np.linspace(0.,distance*k,10001)
    for x in (-cfg.length_m/2,cfg.length_m/2):
        for y in (-cfg.width_m/2,cfg.width_m/2):
            a,b=1/k-y,cfg.wheelbase_m/2+x
            for sampled,dense in ((a*np.sin(poses[:,2])+b*np.cos(poses[:,2])-cfg.wheelbase_m/2,
                                  a*np.sin(theta)+b*np.cos(theta)-cfg.wheelbase_m/2),
                                 (1/k-a*np.cos(poses[:,2])+b*np.sin(poses[:,2]),
                                  1/k-a*np.cos(theta)+b*np.sin(theta))):
                assert sampled.min()<=dense.min()+1e-10
                assert sampled.max()>=dense.max()-1e-10
