from dataclasses import replace
import numpy as np
import pytest

from arena_autonomy.corridor_observer import CorridorObserver,compare_boundary_hold
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from test_corner_corridor_shadow import ring_scene
from test_local_follow import history


def config(enabled=True):
    return FollowConfig(boundary_evidence_mode='corner_distance_candidate',
                        occupancy_timing_mode='per_ray_candidate',corridor_fallback_enabled=enabled)


@pytest.mark.parametrize('value',[1,'true',None])
def test_only_boolean_option_is_accepted(value):
    with pytest.raises(ValueError):replace(config(),corridor_fallback_enabled=value)


def test_option_is_off_by_default_and_requires_explicit_baselines():
    assert LocalFollow().corridor_observer is None
    with pytest.raises(ValueError):FollowConfig(corridor_fallback_enabled=True)
    with pytest.raises(ValueError):FollowConfig(boundary_evidence_mode='corner_distance_candidate',corridor_fallback_enabled=True)


@pytest.mark.parametrize('sign',[1,-1])
def test_enabled_wrapper_matches_approved_shadow_and_does_not_write_boundary_memory(sign):
    baseline,enabled=LocalFollow(config(False)),LocalFollow(config())
    observer=CorridorObserver(config(False))
    side='left' if sign==1 else 'right'
    positive=0
    for now in (.4,.5,.6,.62,.7):
        scene=ring_scene(.15,sign,now)
        args=(scene,history(),0.,side,sign*.2,.02)
        original=baseline.command(*args)
        expected=compare_boundary_hold(original,observer.evaluate(*args))
        actual=enabled.command(*args)
        assert actual[0]==expected['proposed_speed_mps']
        assert actual[1]==expected['proposed_steering_rad']
        assert actual[2]['corridor_selected']==expected['selected']
        assert enabled.path is None and not actual[2]['boundary_memory_from_corridor']
        if expected['selected']:
            positive+=1
            assert actual[2]['unfiltered_occupancy']
            assert actual[2]['occupied_points']==len(scene.occupied_endpoints_xy_m)
            assert actual[2]['occupancy_padding_points']==actual[2]['occupied_points']
            assert actual[2]['occupancy_padding_min_m']>=.055
    assert positive>0


@pytest.mark.parametrize('reason',['follow_gap_hold','follow_reverse_motion','follow_front_unobserved',
    'follow_scene_stale','follow_input_stop','follow_release_wait','follow_invalid_state','unrecognized_reason'])
def test_other_hold_reasons_cannot_be_overridden_even_by_positive_observer(monkeypatch,reason):
    ctl=LocalFollow(config())
    monkeypatch.setattr(ctl,'_baseline_command',lambda *args:(0.,0.,{'reason':reason,'behavior':'HOLD'}))
    monkeypatch.setattr(ctl.corridor_observer,'evaluate',lambda *args:{'positive':True,'reason':'test_positive',
        'proposed_speed_mps':1.,'steering_rad':.2})
    out=ctl.command(ring_scene(),history(),0.,'left')
    assert out[0]==0 and out[2]['reason']==reason and not out[2]['corridor_selected']


def test_reset_and_epoch_clear_fresh_confirmation():
    ctl=LocalFollow(config())
    for t in (.4,.5,.6):result=ctl.command(ring_scene(.15,now=t),history(),0.,'left',.2)
    assert result[0]>0
    ctl.reset()
    assert ctl.corridor_observer.confirmations==0 and ctl.corridor_observer.path is None
    assert ctl.command(ring_scene(.15,now=.7),history(),0.,'left',.2)[0]==0


def test_reverse_does_not_use_boundary_missing_reason_to_drive():
    ctl=LocalFollow(config())
    for t in (.4,.5,.6):ctl.command(ring_scene(.15,now=t),history(),0.,'left',.2)
    result=ctl.command(ring_scene(.15,now=.7),history(),-.01,'left',.2)
    assert result[0]==0 and not result[2]['corridor_selected']
    assert result[2]['corridor_candidate']['reason']=='reverse_motion'
