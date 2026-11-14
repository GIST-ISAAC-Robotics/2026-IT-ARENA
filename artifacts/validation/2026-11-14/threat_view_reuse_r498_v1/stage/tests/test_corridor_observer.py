"""휴대 가능한 shadow의 엄격한 입력 경계와 보호 사유 비대체 계약."""
from dataclasses import replace
import numpy as np
import pytest

import arena_autonomy.corridor_observer as module
from arena_autonomy.local_follow import FollowConfig
from probe_corner_temporal import TemporalCornerShadow
from test_corner_corridor_shadow import ring_scene
from test_local_follow import history


def test_observer_matches_prototype_on_three_real_geometry_scans():
    config=FollowConfig(occupancy_timing_mode='per_ray_candidate')
    new, old = module.CorridorObserver(config), TemporalCornerShadow(config)
    for t in (.4,.5,.6,.62,.7):
        scene=ring_scene(.15,now=t)
        a=new.evaluate(scene,history(),0.,'left',.2,.02)
        b=old.observe(scene,history(),0.,'left',.2,.02)
        assert a['positive'] == b['temporal_candidate']
        for field in ('reason','fresh','confirmations','release_count','proposed_speed_mps'):
            assert a[field] == b[field]
        assert not a['runtime_connected'] and not a['driving_authorized']


@pytest.mark.parametrize('value', [float('nan'),float('inf'),.38])
def test_invalid_previous_steering_cannot_create_candidate(value):
    result=module.current_proposal(ring_scene(),0.,'left',value,.02,FollowConfig())
    assert result['reason']=='invalid_motion_state' and not result['point_occupancy_supported']


def test_configured_shorter_ttl_is_respected():
    scene=replace(ring_scene(now=.4),t_ref_s=.5)
    result=module.current_proposal(scene,0.,'left',.1,.02,FollowConfig(memory_ttl_s=.15))
    assert result['reason']=='scene_stale'


def test_source_frame_mismatch_never_keeps_confirmation():
    observer=module.CorridorObserver(FollowConfig())
    for t in (.4,.5,.6):observer.evaluate(ring_scene(.15,now=t),history(),0.,'left',.2,.02)
    scene=ring_scene(.15,now=.7)
    other=replace(scene,source_frame_id='other_laser',
                  rays=tuple(replace(r,source_frame_id='other_laser') for r in scene.rays))
    result=observer.evaluate(other,history(),0.,'left',.2,.02)
    assert result['reason']=='source_frame_mismatch'
    assert result['confirmations']==0 and observer.path is None


@pytest.mark.parametrize('reason',['follow_gap_hold','follow_reverse_motion','follow_front_unobserved',
    'follow_scene_stale','follow_input_stop','follow_release_wait','follow_invalid_state','new_unknown_reason'])
def test_only_boundary_loss_is_eligible_never_other_hold_reasons(reason):
    proposal={'positive':True,'proposed_speed_mps':.8,'steering_rad':.2}
    result=module.compare_boundary_hold((0.,0.,{'reason':reason}),proposal)
    assert not result['selected'] and result['proposed_speed_mps']==0
    assert not result['boundary_memory_modified']


def test_active_baseline_is_not_replaced_with_faster_shadow():
    result=module.compare_boundary_hold((.3,.1,{'reason':'local_follow'}),
                                       {'positive':True,'proposed_speed_mps':1.,'steering_rad':.2})
    assert not result['selected'] and result['proposed_speed_mps']==.3


def test_boundary_hold_comparison_does_not_grant_drive_permission():
    result=module.compare_boundary_hold((0.,0.,{'reason':'follow_boundary_unconfirmed'}),
                                       {'positive':True,'proposed_speed_mps':.6,'steering_rad':.2})
    assert result['selected'] and result['proposed_speed_mps']==.6
    for field in ('driving_authorized','runtime_connected','boundary_memory_modified','transition_dynamics_verified'):
        assert result[field] is False
