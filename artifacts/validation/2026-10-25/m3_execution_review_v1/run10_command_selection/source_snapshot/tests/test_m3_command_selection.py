"""실제 명령 거절 뒤 후보 선택·재개 상태의 경계. 관측/간격 기준을 면제하지 않는다."""
import pytest
import arena_autonomy.m3_execution_trial as mx
from test_m3_execution_trial import GRASS, padding, ctl
from test_m3_observed_follow import timed_scene
from test_local_follow import history


def active_trial():
    trial = mx.ExecutionLateralTrial(GRASS, mx.ExecutionConfig(command_fallback=True))
    scene, path = timed_scene(.3)
    trial.step(scene, [], path, .35, padding(scene))
    trial.active, trial.clear_fresh = True, 1
    return trial


def test_eligible_center_command_rejection_does_not_end_search_or_clear_intent():
    scene, path = timed_scene(.4)
    trial = active_trial()
    checked = []

    def accept(name, candidate):
        checked.append(name)
        return {'accepted': name == 'straight', 'reason': None if name == 'straight' else 'unobserved_command'}

    out = trial.step(scene, [], path, .35, padding(scene), accept)
    assert checked == ['center', 'hold', 'straight']
    assert out['selected'] == 'straight' and trial.active
    assert all(v['eligible_for_trial'] for v in out['checks'].values())
    assert not out['checks']['center']['command']['accepted']


def test_all_command_rejections_hold_without_clearing_intent():
    scene, path = timed_scene(.4)
    trial = active_trial()
    out = trial.step(scene, [], path, .35, padding(scene),
                     lambda *args: {'accepted': False, 'reason': 'unobserved_command'})
    assert out['path'] is None and out['mode'] == 'HOLD_STOP_TRIAL' and trial.active
    assert all(not v['command']['accepted'] for v in out['checks'].values())


def test_command_fallback_requires_explicit_bool():
    assert mx.ExecutionConfig().command_fallback is False
    with pytest.raises(ValueError):
        mx.ExecutionConfig(command_fallback=1)


def test_rejected_alternatives_do_not_reset_or_multiply_fresh_resume(monkeypatch):
    """실제 경로/관측은 그대로 두고 우선 후보 거절만 주입. 재개는 선택된 출력에 한 번 적용된다."""
    follow = ctl(mx.ExecutionConfig(command_fallback=True))
    h = history()
    scene, path = timed_scene(.3)
    # 실제 G1/관측을 두고 측방 의도만 이미 활성인 상태를 만든다.
    follow.command(scene, h, 0., 'left')
    follow.lateral.active, follow.lateral.clear_fresh = True, 1
    follow.held, follow.release_count = True, 0
    real_check = follow._evaluate_command
    seen = []

    def reject_preferred(name, *args):
        seen.append(name)
        check = real_check(name, *args)
        if name != 'straight':
            check.update(accepted=False, reason='m3_command_unobserved')
        return check

    monkeypatch.setattr(follow, '_evaluate_command', reject_preferred)
    first = follow.command(timed_scene(.4)[0], h, 0., 'left')
    assert first[0] == 0 and first[2]['reason'] == 'm3_release_wait'
    assert follow.release_count == 1 and 'center' in seen and 'straight' in seen
    second = follow.command(timed_scene(.5)[0], h, 0., 'left')
    assert second[0] > 0 and second[2]['m3_candidate'] == 'straight'
    assert not follow.held and follow.release_count == 2
