"""감사 의미 수정으로 기존 행동/자료/정지 gate를 우회할 수 없다."""
import pytest

from audit_g1_case_evidence import combine


def inputs():
    return (dict(checks={'hold_matches_dense_pose': False, 'stop_matches_dense_pose': True,
                         'boundary_times_valid': True, 'dense_ego_received': True},
                 hold_pose_contradictions=[{'t0': 6.059, 't1': 6.161}]),
            dict(hold_requirement_applicable=False, dense_hold_requirement_met=None,
                 sampled_zero_target_episodes=[{'movement_after_settled': False}]))


def test_sparse_inference_remains_visible_but_is_not_a_stationary_claim():
    result = combine(*inputs(), True)
    assert result['passed']
    assert result['legacy_sparse_hold_check']['passed'] is False
    assert result['legacy_sparse_hold_check']['contradictions']
    assert not result['original_verdict_changed'] and not result['driving_authorized']


@pytest.mark.parametrize('check', ['stop_matches_dense_pose', 'boundary_times_valid', 'dense_ego_received'])
def test_every_other_check_still_required(check):
    old, hold = inputs()
    old['checks'][check] = False
    assert not combine(old, hold, True)['passed']


@pytest.mark.parametrize('met', [False, None, 'true'])
def test_real_required_hold_cannot_become_optional(met):
    old, hold = inputs()
    hold.update(hold_requirement_applicable=True, dense_hold_requirement_met=met)
    assert not combine(old, hold, True)['passed']


def test_required_hold_pass_and_original_failure_cannot_be_upgraded():
    old, hold = inputs()
    hold.update(hold_requirement_applicable=True, dense_hold_requirement_met=True)
    assert combine(old, hold, True)['passed']
    assert not combine(old, hold, False)['passed']


def test_motion_after_established_sampled_stop_still_fails():
    old, hold = inputs()
    hold['sampled_zero_target_episodes'][0]['movement_after_settled'] = True
    assert not combine(old, hold, True)['passed']


def test_incomplete_old_audit_cannot_produce_a_pass():
    old, hold = inputs()
    del old['checks']['hold_matches_dense_pose']
    with pytest.raises(ValueError): combine(old, hold, True)
