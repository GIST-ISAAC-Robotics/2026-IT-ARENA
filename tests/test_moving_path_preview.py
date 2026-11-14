"""같은 현재 경로의 목표점 대조: 정책 범위와 원 점유 비교 계약."""
from dataclasses import replace
import copy
import numpy as np
import pytest

import arena_autonomy.bypass_entry_trial as entry
from arena_autonomy.local_path import braking_speed
from test_moving_pass_current_return import current_guide


def fixture():
    guide,scene,current,_=current_guide()
    guide.config=replace(guide.config,traffic_recovery_trial=True,
        guard_observation_policy='selected_r181',guard_curve_trial=True,
        pass_release_trial=True,parallel_hold_trial=True,
        partial_observation_policy='bounded_local_v4',occupancy_timing_mode='per_ray_candidate',
        boundary_evidence_mode='corner_distance_candidate')
    row=current['candidates'][0]
    row['reason']='observed_entry_command_candidate'
    row['command_clearance_m']=.12
    row['target_speed_mps']=braking_speed(.12,scene.t_ref_s-scene.source_first_s)
    return guide,scene,current


@pytest.mark.parametrize('skip',['default','follow','missing_path','lost_wall','occupied_other_reason'])
def test_noncurrent_or_following_does_not_change_command(skip,monkeypatch):
    guide,scene,current=fixture()
    if skip=='default': guide.config=replace(guide.config,guard_observation_policy='original_v1')
    elif skip=='follow': guide.follow_headway=True
    elif skip=='missing_path': guide.path=None
    elif skip=='lost_wall': current['reference']['reason']='wall_lost'
    elif skip=='occupied_other_reason': current['candidates'][0]['reason']='path_unavailable'
    def forbidden(*a,**kw): pytest.fail('unnecessary pursuit comparison')
    monkeypatch.setattr(entry,'_compare_entries',forbidden)
    assert guide._current_path_preview(scene,.4,0.,.1,current) is current


@pytest.mark.parametrize('candidate_speed',[0.,.1,.6])
@pytest.mark.parametrize('primary_speed',[None,.9])
def test_selection_keeps_same_path_actual_steering_and_comparator_checks(candidate_speed,primary_speed,monkeypatch):
    guide,scene,current=fixture()
    if primary_speed is not None:
        current['candidates'][0]['target_speed_mps']=primary_speed
    old_path=guide.path.copy();old=copy.deepcopy(current)
    seen=[]
    def compare(got,now,speed,previous,dt,**kw):
        assert got is scene and now==scene.t_ref_s
        assert (speed,previous,dt)==(.4,-.16,.1)
        assert kw['guide']['path'] is guide.path
        assert kw['padding_provider']==guide.padding_provider
        assert not kw['follow_headway']
        assert kw['pursuit_provider'].keywords=={'lookahead_m':.24}
        seen.append(kw)
        return dict(selected=None if candidate_speed==0 else
                    dict(speed_mps=candidate_speed,steering_rad=-.28),candidates=[])
    monkeypatch.setattr(entry,'_compare_entries',compare)
    out=guide._current_path_preview(scene,.4,-.16,.1,current)
    assert len(seen)==1
    assert out['moving_path_preview']['applied']==(candidate_speed>0)
    if not out['moving_path_preview']['applied']:
        assert out['candidates']==old['candidates']
    np.testing.assert_array_equal(guide.path,old_path)
    assert current==old and guide.phase=='entry' and not guide.follow_headway


@pytest.mark.parametrize('lower,upper,accepted',[
    (-.0517,1.913,True),(.047,1.913,True),(.104,1.913,False),(.047,.29,False)])
def test_current_entry_uses_visible_forward_segment_without_rear_extrapolation(lower,upper,accepted):
    guide,_,_=fixture()
    wall={'reason':'current_connected_straight_wall_fit','actual_return_count':118}
    assert guide._wall_covers_guide((.359,-.132,lower,upper),wall)==accepted
    if accepted:
        assert wall['guide_extent']=='current_body_front_and_forward_entry_only'


def test_forward_only_allowance_does_not_change_legacy_or_unconfirmed_wall():
    guide,_,_=fixture();fit=(.359,-.132,.047,1.913)
    assert not guide._wall_covers_guide(fit,{'reason':'unconfirmed'})
    guide.config=replace(guide.config,guard_observation_policy='original_v1')
    assert not guide._wall_covers_guide(fit,{'reason':'current_connected_straight_wall_fit'})
