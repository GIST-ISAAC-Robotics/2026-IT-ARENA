"""교집합 기하를 프로세스 안에서만 교체하여 기존 고장/소실 계약을 확인한다."""
import pytest
from arena_autonomy.partial_observation import PartialObservationPolicy
from probe_joint_missing_replay import evaluate_joint
import test_partial_observation as existing


@pytest.fixture(autouse=True)
def shadow_only(monkeypatch):
    monkeypatch.setattr(PartialObservationPolicy,'evaluate',evaluate_joint)


@pytest.mark.parametrize('check',[
    existing.test_known_obstacle_disappearance_does_not_expire_into_permission,
    existing.test_close_return_at_gap_edge_blocks_even_though_other_edge_is_far,
    existing.test_no_previous_observation_is_not_free_space_and_stale_history_latches,
    existing.test_latch_diagnostic_keeps_first_cause_until_confirmed_recovery,
    existing.test_still_visible_point_is_not_treated_as_disappeared,
    existing.test_same_scan_rechecks_old_unmatched_point_after_steering_changes,
    existing.test_forward_hidden_known_object_remains_latched_after_it_disappears,
    existing.test_wide_blind_interval_does_not_recover_from_one_narrow_gap,
    existing.test_independent_gate_caps_request_not_scan_and_preserves_zero_command,
    existing.test_reverse_feedback_stops_output_but_does_not_expire_live_observations,
])
def test_existing_essential_loss_and_recovery_contracts(check):
    check()


@pytest.mark.parametrize('fault',['near_obstacle','stale_command','stale_scan','transport',
                                 'missing_motion','reverse_motion','invalid_scan','central_gap'])
def test_independent_faults_remain_stop(fault):
    existing.test_partial_policy_does_not_bypass_independent_faults(fault)


@pytest.mark.parametrize('surface',[False,True])
def test_current_obstacle_brakes_after_reobservation(surface):
    existing.test_current_close_obstacle_still_brakes_when_reobserved_match_is_removed_from_loss_check(surface)


def test_second_hidden_object_survives_far_moving_surface_exclusion():
    policy,history=existing.warmed([[.386,.432],[.06,.15]])
    result=policy.evaluate(existing.sample(indices=[375]),[],history,.6,.15,.175,.175)
    assert result['reason']=='known_obstacle_latched'
    assert any(w['previous_point_now_xy'][1]<.2 for w in result['latch_evidence']['witnesses'])


def test_more_uncertainty_can_retain_the_same_previously_excluded_point():
    policy,history=existing.warmed([[.386,.432]])
    result=policy.evaluate(existing.sample(.5,[375]),[],history,.6,.15,.175,.175)
    assert result['allowed']
    # fresh scan을 가장해 이전 위협을 지우지 않고, 같은 scan의 지연만 증가시킨다.
    result=policy.evaluate(existing.sample(.5,[375]),[],history,.78,.15,.175,.175)
    assert result['reason']=='known_obstacle_latched'


def test_invalid_joint_geometry_keeps_original_latch(monkeypatch):
    def fail(*args,**kwargs):raise ValueError('unsupported bounded geometry')
    monkeypatch.setattr('probe_joint_missing_replay.joint_missing_sweep',fail)
    policy,history=existing.warmed([[.06,.15]])
    assert policy.evaluate(existing.sample(indices=[375]),[],history,.6,.15)['reason']=='known_obstacle_latched'
