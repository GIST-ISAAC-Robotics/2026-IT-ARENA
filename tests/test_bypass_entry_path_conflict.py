"""새 우회 진입에서 현재 보이는 막힌 길을 피하되 미래 여유로 과도하게 거부하지 않는다."""
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from arena_autonomy.bypass_handoff_corridor_trial import entry_path_conflict
from arena_autonomy.local_follow import FollowConfig
from test_bypass_handoff_corridor_trial import subject
from test_bypass_handoff import context


@pytest.mark.parametrize('lateral,rejected',[(0.,True),(.04,True),(.06,False),(.08,False)])
def test_body_overlap_not_expanded_future_envelope(lateral,rejected):
    config=FollowConfig()
    row=dict(path_points=[[0.,0.],[.5,0.],[1.,0.]])
    scene=SimpleNamespace(occupied_endpoints_xy_m=[(.5+config.wheelbase_m/2,lateral)])
    result=entry_path_conflict(row,scene,config)
    assert result['rejected'] is rejected
    assert result['free_space_verified'] is False


def test_turning_body_and_obstacle_after_initial_transition_are_included():
    config=FollowConfig()
    row=dict(path_points=[[0.,0.],[.3,.1],[.6,.3],[.9,.6]])
    direction=np.array([1.,1.])/np.sqrt(2)
    point=np.array([.9,.6])+config.wheelbase_m/2*direction
    result=entry_path_conflict(row,SimpleNamespace(occupied_endpoints_xy_m=[point]),config)
    assert result['rejected'] is True
    assert result['body_gap_m']==pytest.approx(-config.width_m/2)


def test_no_returns_is_not_free_space_certificate():
    result=entry_path_conflict(dict(path_points=[[0.,0.],[1.,0.]]),
        SimpleNamespace(occupied_endpoints_xy_m=[]),FollowConfig())
    assert result['body_gap_m'] is None and result['free_space_verified'] is False


@pytest.mark.parametrize('enabled',[False,True])
def test_known_path_conflict_only_changes_explicit_entry_admission(enabled):
    h=subject();h.execution=replace(h.execution,entry_motion_gate=enabled)
    _,_,scene=context(.7)
    h.monitor.rows=[dict(source=.5,time=.6),dict(source=.6,time=.7)]
    plan=dict(chosen='right',rows={'right':dict(offset_m=-.263,transition_m=.9)})
    row=dict(eligible=True,target_speed_mps=.35,offset_m=-.263,transition_m=.9)
    conflict=dict(rejected=True,reason='current_return_inside_bypass_path',body_gap_m=-.07)
    module='arena_autonomy.bypass_handoff_corridor_trial'
    with patch.object(h.bypass.guide,'_plan',return_value=plan), \
         patch(module+'._compare_entries',return_value=dict(reason='candidate',candidates=[row])), \
         patch(module+'.entry_path_conflict',return_value=conflict) as check:
        usable,detail=h._execution_entries(scene,0.,0.,.02)
    assert usable==([] if enabled else [(-.263,.9)])
    assert check.call_count==int(enabled)
    assert ('entry_path_conflict' in detail['candidates'][0]) is enabled


def test_existing_current_command_rejection_is_never_overridden():
    h=subject();h.execution=replace(h.execution,entry_motion_gate=True)
    _,_,scene=context(.7)
    h.monitor.rows=[dict(source=.5,time=.6),dict(source=.6,time=.7)]
    plan=dict(chosen='right',rows={'right':dict(offset_m=-.263,transition_m=.9)})
    row=dict(eligible=False,target_speed_mps=0.,offset_m=-.263,transition_m=.9)
    module='arena_autonomy.bypass_handoff_corridor_trial'
    with patch.object(h.bypass.guide,'_plan',return_value=plan), \
         patch(module+'._compare_entries',return_value=dict(reason='blocked',candidates=[row])), \
         patch(module+'.entry_path_conflict') as check:
        usable,_=h._execution_entries(scene,0.,0.,.02)
    assert usable==[] and check.call_count==0
