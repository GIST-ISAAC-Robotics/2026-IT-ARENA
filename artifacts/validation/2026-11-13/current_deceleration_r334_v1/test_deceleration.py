import copy
from unittest.mock import patch
import pytest
from deceleration import CurrentDecelerationProcess, same_positive_intent, OverlapProcess


def records():
    original = dict(stage='plan', generation=0, sequence=1,
        input=dict(epoch=0, side='left', reference_s=1.,
                   scan=dict(representation='native_measurements_v1',
                             measurements=dict(source_epoch=0, acquired_at_s=[.9]))),
        output=[.5, .2, dict(handoff_state='FOLLOW', reason='handoff_follow', normal=dict(
            mode='g1_follow', preferred_wall='left', path_wall='left',
            selector_mode='SOLO', selector_choice='baseline', reason='local_follow'))],
        route_check=dict(revision=8, retained=False))
    new = copy.deepcopy(original)
    new['sequence'] = 3
    new['input']['reference_s'] = 1.1
    new['output'][0] = .45
    current = dict(stage='current', generation=0, sequence=2, plan=copy.deepcopy(original),
        input=dict(reference_s=1.1),
        output=dict(speed_mps=.48, steering_rad=.22, reference_s=1.1))
    return original, new, current


def worker(original):
    w = CurrentDecelerationProcess.__new__(CurrentDecelerationProcess)
    w.current_input = dict(plan=original)
    w.ready = w.saved_plan = w.pending_speed_cap = w.fault = None
    w.retained = original
    w.revision, w.generation = 8, 0
    return w


def collect(w, rows):
    def base(owner):
        assert owner is w
        for row in rows:
            if row['stage'] == 'plan': w.saved_plan = copy.deepcopy(row)
        if any(row['stage'] == 'current' for row in rows): w.current_input = None
        return rows
    with patch.object(OverlapProcess, 'collect', side_effect=base):
        return w.collect()


def test_smaller_positive_target_caps_completed_check_without_retimestamp():
    old, new, current = records(); w = worker(old)
    untouched = copy.deepcopy(current)
    collect(w, [current, new])
    assert w.ready['output']['speed_mps'] == .45
    assert w.ready['output']['steering_rad'] == .22
    assert w.ready['output']['reference_s'] == 1.1
    assert w.ready['plan'] == untouched['plan']
    assert w.ready['input'] == untouched['input']
    assert current['output'] == untouched['output']
    assert w.saved_plan == {k: v for k, v in new.items() if k != 'deferred_positive_transition'}
    assert w.revision == 8


def test_current_safety_cap_is_never_increased():
    old, new, current = records(); w = worker(old)
    current['output']['speed_mps'] = .15
    collect(w, [new, current])
    assert w.ready['output']['speed_mps'] == .15


def test_later_acceleration_does_not_cancel_pending_deceleration():
    old, new, current = records(); w = worker(old)
    collect(w, [new])
    faster = copy.deepcopy(new); faster['sequence'] = 4; faster['output'][0] = .6
    collect(w, [faster, current])
    assert w.ready['output']['speed_mps'] == .45
    assert w.saved_plan['output'][0] == .6
    assert w.ready['output']['new_plan_deceleration_cap']['sequence'] == 3


def test_ready_result_can_receive_a_later_lower_cap():
    old, new, current = records(); w = worker(old)
    collect(w, [current])
    collect(w, [new])
    lower = copy.deepcopy(new); lower['output'][0] = .3; lower['sequence'] = 4
    collect(w, [lower])
    assert w.ready['output']['speed_mps'] == .3
    assert w.ready['output']['new_plan_deceleration_cap']['checked_target_mps'] == .48


@pytest.mark.parametrize('fault', ['zero', 'negative', 'nan', 'bool', 'handoff', 'wall',
                                  'selector', 'epoch', 'sensor', 'side', 'earlier', 'generation'])
def test_state_and_stop_transitions_still_cancel(fault):
    old, new, current = records(); w = worker(old)
    if fault == 'zero': new['output'][0] = 0.
    if fault == 'negative': new['output'][0] = -.1
    if fault == 'nan': new['output'][0] = float('nan')
    if fault == 'bool': new['output'][0] = True
    if fault == 'handoff': new['output'][2]['handoff_state'] = 'BYPASS'
    if fault == 'wall': new['output'][2]['normal']['path_wall'] = 'right'
    if fault == 'selector': new['output'][2]['normal']['selector_mode'] = 'GUARD'
    if fault == 'epoch': new['input']['epoch'] = 1
    if fault == 'sensor': new['input']['scan']['measurements']['source_epoch'] = 1
    if fault == 'side': new['input']['side'] = 'right'
    if fault == 'earlier': new['input']['reference_s'] = .5
    if fault == 'generation': new['generation'] = 1
    assert not same_positive_intent(old, new)
    collect(w, [new, current])
    assert w.ready is None and w.revision == 9 and w.pending_speed_cap is None
    assert current['discard_reason'] == 'new_plan_superseded_current_check'


def test_failed_retained_check_is_not_turned_into_permission():
    old, new, current = records(); w = worker(old)
    current['plan']['route_check']['retained'] = True
    current['output']['speed_mps'] = 0.
    collect(w, [new, current])
    assert w.ready is None and current['discard_reason'] == 'retained_route_not_proven'


def test_generation_invalidation_clears_deceleration_cap():
    old, new, current = records(); w = worker(old)
    collect(w, [new, current])
    # 외부 프로세스 없이 자식 정리 호출의 위임과 새 상태의 제거만 확인한다.
    from deadline import CurrentDeadlineProcess
    with patch.object(CurrentDeadlineProcess, 'invalidate') as parent:
        w.invalidate('explicit_stop')
    parent.assert_called_once_with('explicit_stop')
    assert w.pending_speed_cap is None
