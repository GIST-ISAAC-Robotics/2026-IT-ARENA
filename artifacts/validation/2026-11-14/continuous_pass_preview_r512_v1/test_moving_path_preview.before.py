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


@pytest.mark.parametrize('skip',['default','follow','missing_path','lost_wall','occupied_other_reason','full_speed'])
def test_healthy_or_noncurrent_or_following_does_not_change_command(skip,monkeypatch):
    guide,scene,current=fixture()
    if skip=='default': guide.config=replace(guide.config,guard_observation_policy='original_v1')
    elif skip=='follow': guide.follow_headway=True
    elif skip=='missing_path': guide.path=None
    elif skip=='lost_wall': current['reference']['reason']='wall_lost'
    elif skip=='occupied_other_reason': current['candidates'][0]['reason']='path_unavailable'
    else: current['candidates'][0]['target_speed_mps']=guide.entry.speed_cap_mps
    def forbidden(*a,**kw): pytest.fail('unnecessary pursuit comparison')
    monkeypatch.setattr(entry,'_compare_entries',forbidden)
    assert guide._current_path_preview(scene,.4,0.,.1,current) is current


@pytest.mark.parametrize('candidate_speed',[0.,.1,.6])
def test_selection_keeps_same_path_actual_steering_and_comparator_checks(candidate_speed,monkeypatch):
    guide,scene,current=fixture();old_path=guide.path.copy();old=copy.deepcopy(current)
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
    assert out['moving_path_preview']['applied']==(candidate_speed>old['candidates'][0]['target_speed_mps'])
    if not out['moving_path_preview']['applied']:
        assert out['candidates']==old['candidates']
    np.testing.assert_array_equal(guide.path,old_path)
    assert current==old and guide.phase=='entry' and not guide.follow_headway
