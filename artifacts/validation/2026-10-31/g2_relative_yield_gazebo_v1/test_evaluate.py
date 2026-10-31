"""제한 판정도 실제 장면/고장/정지/접촉 실패를 통과로 바꾸지 않는다."""
from copy import deepcopy
from evaluate import limited_checks


def example():
    return dict(checks=dict(course_full_lap=False, postreturn_course_progress=False,
        restart_follow_progress=False, restart_during_bypass=True,
        normal_return_after_restart=True, stop_model_stop_stable=True,
        all_actor_contacts=True, replay_exact=True, runtime_contract=True),
        restart=dict(post_restart_follow_progress_m=3.1))


def test_limited_scope_preserves_original_and_requires_actual_caps():
    original = example(); before = deepcopy(original)
    assert all(limited_checks(original, 10, 8).values())
    assert original == before
    for requested, positive in ((0, 0), (10, 0)):
        assert not all(limited_checks(original, requested, positive).values())


def test_no_scene_or_failed_stop_contact_replay_is_still_failure():
    for key in ('restart_during_bypass', 'normal_return_after_restart',
                'stop_model_stop_stable', 'all_actor_contacts', 'replay_exact', 'runtime_contract'):
        value = example(); value['checks'][key] = False
        assert not all(limited_checks(value, 10, 8).values()), key


def test_three_meters_is_required_and_old_five_meter_result_not_changed():
    value = example(); value['restart']['post_restart_follow_progress_m'] = 2.99
    assert not all(limited_checks(value, 10, 8).values())
    value['restart']['post_restart_follow_progress_m'] = 3.
    assert all(limited_checks(value, 10, 8).values())
    assert value['checks']['restart_follow_progress'] is False
