"""선택형 교통 회복의 기본값·선행 계약·독립 상태 검사."""
from dataclasses import replace
import numpy as np
import pytest
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from test_local_follow import scan,history
from test_timed_policy_probe import points

def enabled():
    return FollowConfig(traffic_recovery_trial=True,guard_curve_trial=True,pass_release_trial=True,
        parallel_hold_trial=True,partial_observation_policy='bounded_local_v4',
        boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate')

def test_default_and_explicit_configuration():
    assert not FollowConfig().traffic_recovery_trial
    assert LocalFollow().partial_policy is None
    assert isinstance(LocalFollow(enabled()).partial_policy,RecoveryPolicy)

@pytest.mark.parametrize('changes',[
    dict(traffic_recovery_trial=1),dict(guard_curve_trial=False),dict(pass_release_trial=False),
    dict(parallel_hold_trial=False),dict(partial_observation_policy='bounded_local_v3'),
    dict(occupancy_timing_mode='oldest'),dict(boundary_evidence_mode='connected'),dict(max_speed_mps=1.01)])
def test_requires_complete_existing_trial_contract(changes):
    with pytest.raises(ValueError):replace(enabled(),**changes)

def test_planner_and_safety_state_are_separate():
    planner=LocalFollow(enabled());safety=RecoveryPolicy(True,True,True)
    safety.latched=True;safety.latch_reason='test retained'
    planner.reset()
    assert safety.latched and safety.latch_reason=='test retained'
    assert planner.partial_policy.anchor_probe is not safety.anchor_probe

def test_bad_projected_order_stops_without_erasing_latch():
    p=RecoveryPolicy(True,True,True);p.latched=True;p.latch_reason='test retained'
    h=history();raw=scan(.5,[8.]*500)
    r=p.evaluate(raw,points(h,raw,.6)[::-1],h,.6,.2)
    assert not r['allowed'] and r['reason']=='traffic_recovery_invalid'
    assert p.latched and p.latch_reason=='test retained'
