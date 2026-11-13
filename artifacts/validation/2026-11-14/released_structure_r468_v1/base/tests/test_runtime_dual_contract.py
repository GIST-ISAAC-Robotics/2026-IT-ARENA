import copy
from unittest.mock import patch
import pytest
from arena_autonomy.runtime_dual_state import RuntimeDualProcess as CurrentDecelerationProcess, same_positive_intent
from arena_autonomy.runtime_dual_process import DualSlots as OverlapProcess
from arena_autonomy.runtime_dual_delivery import RuntimeDualDelivery as Delivery
from types import SimpleNamespace


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
    w.supported = lambda output: output[0] > 0
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
    w.invalidate('explicit_stop')
    assert w.generation == 1 and w.revision == 9
    assert w.pending_speed_cap is w.ready is w.retained is w.saved_plan is None
    assert w.reset_reason is None


def setup_case():
    old, new, current = records()
    old['output'][2]['normal']['path_source_s'] = .9
    current['plan'] = copy.deepcopy(old)
    current['route_check'] = dict(revision=8, retained=False)
    current['input'] = copy.deepcopy(old['input'])
    current['input'].update(reference_s=1.1, previous_steering_rad=.1)
    current['input']['scan']['measurements']['acquired_at_s'] = [1.]
    w = worker(old)
    w.ready, w.current_input, w.retained = current, None, None
    w.current = SimpleNamespace(inflight=None)
    w.last_application = -float('inf')
    w.geometry_age_limit = .5
    d = Delivery(w, lambda: 1.12)
    d.collect = lambda: None
    return d, w


def poll(d):
    return d.poll(1.12, previous_steering_rad=.1, side='left', motion_epoch=0, sensor_epoch=0)


def ack(d, **changes):
    args = dict(published=True, reference_s=1.1, speed_mps=.4, steering_rad=.22, now_s=1.12)
    args.update(changes)
    return d.acknowledge_publication(**args)


def test_poll_does_not_claim_publication_or_discard_ready():
    d, w = setup_case()
    result = poll(d)
    assert result['kind'] == 'candidate'
    assert w.ready is not None and w.retained is None
    assert d.pending_publication is not None
    assert w.last_application == -float('inf')


def test_checked_publication_only_then_retains_original_route():
    d, w = setup_case(); old = copy.deepcopy(w.ready['plan'])
    poll(d); result = ack(d)
    assert result['retained']
    assert w.ready is None and w.retained['input'] == old['input']
    assert w.retained['output'] == old['output']
    assert w.last_application == d.last_application == 1.12
    assert 'route_check' not in w.retained
    assert ack(d)['reason'] == 'no_current_candidate'


@pytest.mark.parametrize('changes', [dict(published=False), dict(speed_mps=0.),
    dict(speed_mps=.6), dict(reference_s=1.12), dict(steering_rad=.21), dict(speed_mps=float('nan'))])
def test_unpublished_or_different_command_cannot_retain_route(changes):
    d, w = setup_case(); poll(d)
    assert not ack(d, **changes)['retained']
    assert w.ready is None and w.retained is None


def test_late_ack_does_not_renew_expired_proof():
    d, w = setup_case(); poll(d)
    result = ack(d, now_s=1.31)
    assert not result['retained'] and result['reason'].startswith('post_publication_')
    assert w.ready is None and w.retained is None


def test_duplicate_poll_cannot_apply_unacknowledged_candidate():
    d, _ = setup_case(); poll(d)
    with pytest.raises(RuntimeError, match='acknowledgement'): poll(d)


def test_stop_between_selection_and_publish_cancels_pending_ack():
    d, w = setup_case(); poll(d)
    def invalidated():
        w.ready = w.retained = w.saved_plan = None
        w.revision += 1
    w.invalidate = invalidated
    d.invalidate()
    assert d.pending_publication is None and not ack(d)['retained']


def test_retained_path_requests_fresh_current_check_not_old_command():
    d, w = setup_case(); poll(d); ack(d)
    assert poll(d) == dict(kind='needs_recheck')


@pytest.mark.parametrize('change,reason', [
    (dict(now_s=1.09), 'current_clock_boundary'),
    (dict(motion_epoch=1), 'current_input_epoch_or_side_changed'),
    (dict(sensor_epoch=1), 'current_input_epoch_or_side_changed'),
    (dict(side='right'), 'current_input_epoch_or_side_changed'),
    (dict(previous_steering_rad=.11), 'current_previous_request_changed'),
])
def test_ready_result_cannot_cross_changed_clock_epoch_or_request(change, reason):
    d, w = setup_case()
    state = dict(now_s=1.12, previous_steering_rad=.1, side='left', motion_epoch=0, sensor_epoch=0)
    result = d.poll(**{**state, **change})
    assert result['kind'] == 'discarded' and result['reason'] == reason
    assert w.ready is None and d.pending_publication is None


def test_retained_route_requires_current_proof_even_while_geometry_is_young():
    d, w = setup_case()
    w.ready['route_check']['retained'] = True
    result = poll(d)
    assert result['reason'] == 'new_current_route_proof_required'
    assert w.ready is None and w.retained is None


def test_positive_interval_keeps_candidate_without_restamping_it():
    d, w = setup_case()
    d.last_application = 1.08
    original = copy.deepcopy(w.ready)
    assert poll(d) is None
    assert w.ready == original and d.pending_publication is None
