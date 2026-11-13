"""호스트 제어 요청 후보: 응답 지연/역전/정지 선점. 실제 송신·구동은 없음."""
from dataclasses import FrozenInstanceError
from pathlib import Path
import ast

import pytest

from actuation_host_session import CompactHostSession
from audit_actuation_offer_clock import fixture
from audit_actuation_tx_policy import compile_driver
from arena_vehicle_interface.clock_bounds import ClockAssumptions, ClockExchange, fit_clock
from arena_vehicle_interface.clock_freshness import HostTimeWindow
from test_actuation_tx_policy import Mcu

EPOCH = 'host-epoch-A'
LINK = 'connection-A'


def window(ns, upper=None, epoch=EPOCH):
    return HostTimeWindow(epoch, ns, ns if upper is None else upper)


def session(timeout=100_000_000):
    return CompactHostSession(boot='boot-A', host_epoch=EPOCH, connection_id=LINK,
        client_id='host-A', max_lease_us=100_000, request_timeout_ns=timeout)


def offer(**kwargs):
    return fixture()[0] | dict(state='DISARMED', owner=None) | kwargs


def fresh_offer(**kwargs):
    return offer(generation=4, state='ARMED', owner='host-A', lease=9,
                 lease_issued_us=1_020_100, lease_expires_us=1_120_100) | kwargs


def record(item, payload, ns, **kwargs):
    return item.observe(payload, received=window(ns), connection_id=kwargs.get('connection_id', LINK))


def begin(item, kind='ARM', ns=9_020_000_000):
    return item.begin_control(kind, window(ns), fit=fixture(ppm=200)[1],
        max_issue_age_ns=100_000_000, forward_delay_upper_ns=5_000_000, policy_note='합성 시험')


def start(item=None):
    item = item or session()
    record(item, offer(), 9_010_000_000)
    request = begin(item)['envelope']
    assert request is not None
    return item, request


def ack(envelope, **kwargs):
    return dict(type='ack', request=envelope['request'], accepted=True, reason='explicit_arm') | kwargs


def armed(order='ack_first'):
    item, request = start()
    first, second = (ack(request), fresh_offer()) if order == 'ack_first' else (fresh_offer(), ack(request))
    record(item, first, 9_030_000_000)
    assert item.arm_barrier is None
    record(item, second, 9_032_000_000)
    assert item.arm_barrier == window(9_032_000_000)
    return item, request


def review(item, ns=9_036_000_000, source=None, **kwargs):
    return item.review_source(fixture(ppm=200)[1], window(ns), source or window(9_034_000_000),
        max_issue_age_ns=100_000_000, forward_delay_upper_ns=kwargs.get('delay', 5_000_000),
        source_ttl_ns=100_000_000, policy_note='합성 시험')


@pytest.mark.parametrize('order', ['ack_first', 'offer_first'])
def test_both_evidence_orders_require_new_source_and_never_drive(order):
    item, _ = armed(order)
    assert item.pending is None
    assert not review(item, source=window(9_031_000_000))['session_conditions_satisfied']
    result = review(item)
    assert result['session_conditions_satisfied'] and result['arm_ack_correlated']
    assert result['source_watermark_checked']
    assert result['command_created'] is False and result['ready_to_drive'] is False
    assert result['runtime_connected'] is False
    assert not hasattr(item, 'send') and not hasattr(item, 'drive')
    assert item.outcomes[-1]['outcome'] == 'transition_evidenced_not_output_permission'


@pytest.mark.parametrize('only', ['ack', 'offer', 'state', 'diag'])
def test_one_side_of_evidence_never_establishes_arm(only):
    item, req = start()
    from test_actuation_host_observer import state, diag
    payload = {'ack': ack(req), 'offer': fresh_offer(),
               'state': state(generation=4, state='ARMED'),
               'diag': diag(generation=4, state='ARMED')}[only]
    record(item, payload, 9_032_000_000)
    assert item.arm_barrier is None and not review(item)['session_conditions_satisfied']


def test_stop_preempts_pending_arm_before_any_reply():
    item, old = start()
    stop = begin(item, 'STOP', 9_025_000_000)['envelope']
    assert stop is not None, 'STOP must preempt an unconfirmed ARM request'
    assert stop['command'] == dict(v=1, kind='STOP')
    assert item.outcomes[-1]['outcome'] == 'superseded_by_stop_unconfirmed'
    assert record(item, ack(old), 9_030_000_000)['reason'] == 'unmatched_ack'
    record(item, fresh_offer(), 9_032_000_000)
    assert not review(item)['session_conditions_satisfied']
    result = record(item, ack(stop, reason='software_stop'), 9_037_000_000)
    assert result['ack_correlated'] and item.arm_barrier is None
    assert item.outcomes[-1]['outcome'] == 'stop_ack_not_physical_stop'


def test_stop_revokes_completed_arm_even_if_stop_ack_is_lost():
    item, old = armed()
    assert review(item)['session_conditions_satisfied']
    begin(item, 'STOP', 9_037_000_000)
    assert item.arm_barrier is None
    record(item, ack(old), 9_038_000_000)
    assert not review(item, ns=9_040_000_000, source=window(9_039_000_000))['session_conditions_satisfied']
    item.poll(window(9_137_000_000))
    assert item.pending is None and item.outcomes[-1]['outcome'] == 'timeout_unconfirmed'
    assert item.arm_barrier is None


@pytest.mark.parametrize('when,upper', [(9_120_000_000, None), (9_119_000_000, 9_120_000_000)])
def test_deadline_exact_or_uncertain_crossing_cannot_resurrect_arm(when, upper):
    item, req = start()
    record(item, fresh_offer(), 9_032_000_000)
    result = item.observe(ack(req), received=window(when, upper), connection_id=LINK)
    assert result['reason'] == 'unmatched_ack' and item.arm_barrier is None
    assert item.outcomes[-1]['outcome'] == 'timeout_unconfirmed'


@pytest.mark.parametrize('bad_ack', [dict(accepted=False), dict(reason='software_stop'),
                                   dict(request='different'), dict(accepted=1), dict(extra=1)])
def test_bad_ack_never_establishes_permission(bad_ack):
    item, req = start()
    record(item, fresh_offer(), 9_030_000_000)
    record(item, ack(req, **bad_ack), 9_032_000_000)
    assert item.arm_barrier is None and not review(item)['session_conditions_satisfied']


def test_duplicate_ack_does_not_refresh_deadline_and_conflict_cancels():
    item, req = start()
    deadline = item.pending.deadline_ns
    record(item, ack(req), 9_030_000_000)
    assert record(item, ack(req), 9_031_000_000)['reason'] == 'duplicate_ack'
    assert item.pending.deadline_ns == deadline
    record(item, ack(req, accepted=False), 9_032_000_000)
    record(item, fresh_offer(), 9_033_000_000)
    assert item.arm_barrier is None and item.pending is None
    assert item.outcomes[-1]['outcome'] == 'conflicting_ack_unconfirmed'


@pytest.mark.parametrize('changed', [dict(generation=5), dict(owner='foreign'), dict(output_ready=False),
    dict(state='ESTOP_LATCHED', owner=None), dict(state='DISARMED', owner=None)])
def test_incompatible_new_transition_before_ack_cancels(changed):
    item, req = start()
    record(item, fresh_offer(**changed), 9_030_000_000)
    record(item, ack(req), 9_032_000_000)
    assert item.pending is None and item.arm_barrier is None


@pytest.mark.parametrize('payload', [fresh_offer(generation=5, lease=10, state='ESTOP_LATCHED', owner=None),
                                    fresh_offer(lease=10, owner='foreign'),
                                    fresh_offer(lease=10, output_ready=False)])
def test_current_permission_revoked_by_new_state_owner_or_readiness(payload):
    item, _ = armed()
    record(item, payload, 9_033_000_000)
    assert item.arm_barrier is None and not review(item)['session_conditions_satisfied']


def test_rejected_new_generation_offer_does_not_keep_old_permission():
    item, _ = armed()
    result = record(item, fresh_offer(generation=5, lease=9), 9_033_000_000)
    assert result['reason'] == 'duplicate_or_old_lease'
    assert item.arm_barrier is None


def test_foreign_connection_callback_does_not_poison_current_clock_or_pending():
    item, req = start()
    assert record(item, ack(req), 1, connection_id='old')['reason'] == 'connection_mismatch'
    assert not item.closed and item.pending.request == req['request']


@pytest.mark.parametrize('failure', ['disconnect', 'epoch', 'clock', 'boot'])
def test_context_loss_closes_without_auto_recovery(failure):
    item, req = armed()
    if failure == 'disconnect':
        item.disconnect()
    elif failure == 'epoch':
        item.poll(window(9_033_000_000, epoch='host-B'))
    elif failure == 'clock':
        item.poll(window(9_031_000_000))
    else:
        record(item, fresh_offer(boot='boot-B', lease=10), 9_033_000_000)
    assert item.closed and item.arm_barrier is None
    assert record(item, ack(req), 9_035_000_000)['reason'] == 'closed'
    assert begin(item, 'ARM', 9_036_000_000)['envelope'] is None


def test_request_ids_are_nonreused_across_instances_and_bounded():
    a, b = session(), session()
    first = begin(a, 'STOP')['envelope']
    second = begin(a, 'STOP', 9_020_000_001)['envelope']
    third = begin(b, 'STOP')['envelope']
    assert len({x['request'] for x in (first, second, third)}) == 3
    assert all(len(x['request']) <= 64 for x in (first, second, third))
    assert a.outcomes[0]['outcome'] == 'superseded_by_stop_unconfirmed'


def test_pending_and_outcomes_are_bounded_and_read_only_copies():
    item, _ = start()
    pending = item.pending
    assert begin(item, 'CLEAR', 9_021_000_000)['reason'] == 'control_pending'
    assert item.pending is pending
    with pytest.raises(FrozenInstanceError):
        pending.kind = 'STOP'
    for ns in range(9_022_000_000, 9_022_000_100):
        begin(item, 'STOP', ns)
    assert len(item.outcomes) == 32
    copy = item.outcomes
    copy[0]['outcome'] = 'changed'
    assert item.outcomes[0]['outcome'] != 'changed'


def test_clear_confirmation_does_not_arm():
    item = session()
    record(item, offer(state='ESTOP_LATCHED'), 9_010_000_000)
    req = begin(item, 'CLEAR')['envelope']
    record(item, ack(req, reason='explicit_clear'), 9_030_000_000)
    record(item, fresh_offer(state='DISARMED', owner=None), 9_032_000_000)
    assert item.pending is None and item.arm_barrier is None
    assert not review(item)['session_conditions_satisfied']
    new = begin(item, 'ARM', 9_037_000_000)['envelope']
    assert new and new['request'] != req['request'] and new['command']['generation'] == 4


def test_local_stop_latch_survives_missing_stop_ack_and_requires_clear_evidence():
    item = session(timeout=10_000_000)
    record(item, offer(), 9_010_000_000)
    begin(item, 'STOP', 9_012_000_000)
    item.poll(window(9_022_000_000))
    assert item.stop_requested and item.pending is None
    assert begin(item, 'ARM', 9_023_000_000)['reason'] == 'explicit_clear_required'
    record(item, fresh_offer(state='ESTOP_LATCHED', owner=None), 9_030_000_000)
    clear = begin(item, 'CLEAR', 9_031_000_000)['envelope']
    assert clear
    record(item, ack(clear, reason='explicit_clear'), 9_032_000_000)
    assert item.stop_requested
    record(item, fresh_offer(generation=5, lease=10, state='DISARMED', owner=None), 9_033_000_000)
    assert not item.stop_requested and item.arm_barrier is None
    assert begin(item, 'ARM', 9_034_000_000)['envelope'] is not None


def test_disarm_request_revokes_arm_immediately_and_requires_matching_transition():
    item, _ = armed()
    request = begin(item, 'DISARM', 9_033_000_000)['envelope']
    assert request and item.arm_barrier is None
    record(item, fresh_offer(generation=5, lease=10, state='DISARMED', owner=None), 9_034_000_000)
    record(item, ack(request, reason='explicit_disarm'), 9_035_000_000)
    assert item.pending is None and not review(item)['session_conditions_satisfied']


def test_invalid_source_epoch_or_future_timestamp_is_not_usable():
    item, _ = armed()
    assert not review(item, source=window(9_034_000_000, epoch='old-host'))['session_conditions_satisfied']
    assert not review(item, source=window(9_040_000_000))['session_conditions_satisfied']
    assert review(item)['session_conditions_satisfied']


def test_request_overlapping_ack_window_never_confirms():
    item = session()
    record(item, offer(), 9_010_000_000)
    request = item.begin_control('ARM', window(9_020_000_000, 9_025_000_000), fit=fixture()[1],
        max_issue_age_ns=100_000_000, forward_delay_upper_ns=5_000_000, policy_note='합성 시험')['envelope']
    assert request
    result = item.observe(ack(request), received=window(9_023_000_000, 9_026_000_000), connection_id=LINK)
    assert result['reason'] == 'ack_order_uncertain' and item.pending is None


def test_arm_barrier_uses_latest_possible_receipt_not_midpoint():
    item, req = start()
    item.observe(ack(req), received=window(9_030_000_000, 9_032_000_000), connection_id=LINK)
    item.observe(fresh_offer(), received=window(9_031_000_000, 9_034_000_000), connection_id=LINK)
    assert item.arm_barrier == window(9_031_000_000, 9_034_000_000)
    assert not review(item, source=window(9_033_000_000))['session_conditions_satisfied']


def test_callback_queue_delay_keeps_original_receipt_but_moves_arm_barrier():
    item, req = start()
    item.poll(window(9_035_000_000))
    item.observe(ack(req), received=window(9_030_000_000), processed=window(9_036_000_000), connection_id=LINK)
    item.observe(fresh_offer(), received=window(9_031_000_000), processed=window(9_040_000_000), connection_id=LINK)
    assert not item.closed and item.arm_barrier == window(9_040_000_000)
    assert item.latest('offer').received == window(9_031_000_000)
    assert not review(item, ns=9_042_000_000, source=window(9_039_000_000))['session_conditions_satisfied']
    assert review(item, ns=9_042_000_000, source=window(9_041_000_000))['session_conditions_satisfied']


def test_ack_received_before_deadline_but_processed_late_stays_unconfirmed():
    item, req = start()
    record(item, fresh_offer(), 9_030_000_000)
    result = item.observe(ack(req), received=window(9_031_000_000), processed=window(9_120_000_000), connection_id=LINK)
    assert result['reason'] == 'unmatched_ack'
    assert not item.closed and item.arm_barrier is None
    assert item.outcomes[-1]['outcome'] == 'timeout_unconfirmed'


@pytest.mark.parametrize('received,processed', [
    (window(9_035_000_000), window(9_034_000_000)),
    (window(9_030_000_000, 9_035_000_000), window(9_034_000_000, 9_036_000_000)),
    (window(9_030_000_000, epoch='old-host'), window(9_035_000_000)),
])
def test_invalid_separate_receipt_context_closes(received, processed):
    item, req = start()
    item.observe(ack(req), received=received, processed=processed, connection_id=LINK)
    assert item.closed and item.arm_barrier is None


def test_reordered_capture_timestamps_close_even_with_monotonic_processing():
    item, req = start()
    item.observe(ack(req), received=window(9_030_000_000), processed=window(9_035_000_000), connection_id=LINK)
    item.observe(fresh_offer(), received=window(9_029_000_000), processed=window(9_036_000_000), connection_id=LINK)
    assert item.closed and item.arm_barrier is None


def test_malformed_then_valid_duplicate_ack_after_completion_does_not_refresh_barrier():
    item, request = armed()
    barrier = item.arm_barrier
    record(item, ack(request, accepted='yes'), 9_033_000_000)
    record(item, ack(request), 9_034_000_000)
    assert item.arm_barrier is barrier


def test_counter_exhaustion_closes_instead_of_reusing_ids():
    item, _ = armed()
    item._sequence = 2**32-1
    assert begin(item, 'STOP', 9_033_000_000)['reason'] == 'request_sequence_exhausted'
    assert item.closed and item.arm_barrier is None


@pytest.mark.parametrize('bad', [dict(output_ready=False), dict(state='ACTIVE', owner='foreign')])
def test_arm_requires_current_disarmed_ready_offer(bad):
    item = session()
    record(item, offer(**bad), 9_010_000_000)
    assert begin(item)['envelope'] is None


def test_stale_offer_no_time_evidence_and_no_offer_rejected_but_stop_works():
    item = session()
    assert begin(item)['envelope'] is None
    record(item, offer(), 9_021_000_000)
    assert item.begin_control('ARM', window(9_022_000_000))['reason'] == 'time_evidence_invalid'
    assert begin(item, ns=9_120_000_000)['reason'] == 'time_conditions_unsatisfied'
    assert begin(item, 'STOP', 9_121_000_000)['envelope']['command'] == dict(v=1, kind='STOP')


@pytest.mark.parametrize('value', [0, -1, True, 1.5, 60_000_000_001])
def test_invalid_timeout(value):
    with pytest.raises(ValueError):
        session(value)


def test_source_watermark_does_not_repeat_or_accept_overlapping_intervals():
    item, _ = armed()
    assert review(item, source=window(9_034_000_000, 9_035_000_000))['session_conditions_satisfied']
    for source in (window(9_034_000_000), window(9_035_000_000), window(9_033_000_000, 9_035_000_001)):
        assert review(item, source=source)['reason'] == 'source_repeated_or_order_uncertain'
    assert review(item, source=window(9_035_000_001))['session_conditions_satisfied']


def test_source_time_failure_is_not_consumed_and_new_input_can_be_reviewed():
    item, _ = armed()
    assert not review(item, delay=100_000_000)['session_conditions_satisfied']
    assert review(item)['session_conditions_satisfied']
    assert review(item)['reason'] == 'source_repeated_or_order_uncertain'


def test_actual_cpp_arm_stop_and_late_ack(tmp_path):
    binary = compile_driver(tmp_path/'tx-driver')
    mcu = Mcu(binary, boot='boot-A')
    item = session()
    origin = 9_000_000_000
    fit = fit_clock([ClockExchange('boot-A', 1, origin+200_000_000, 200_000, 200_000,
                                   origin+200_000_000)], ClockAssumptions(0, 0, 0, 5_000_000, 200_000))
    options = dict(fit=fit, max_issue_age_ns=100_000_000,
                   forward_delay_upper_ns=1_000_000, policy_note='PC exact clock fixture')
    try:
        t = mcu.setup()
        record(item, mcu.last('offer'), origin+t*1000)
        request = item.begin_control('ARM', window(origin+(t+100)*1000), **options)['envelope']
        assert request
        delivered = mcu.send(t+100, request['command'], request['request'])
        assert delivered[-1]['result'][0]
        before = len(mcu.payloads)
        mcu.pump(t+100)
        mcu.pump(t+100)
        for n, payload in enumerate(mcu.payloads[before:]):
            record(item, payload, origin+(t+200)*1000+n)
        assert item.arm_barrier is not None
        stop = item.begin_control('STOP', window(origin+(t+300)*1000))['envelope']
        assert item.arm_barrier is None
        delivered = mcu.send(t+300, stop['command'], stop['request'])
        assert delivered[-1]['result'][0] and mcu.row['state'] == 'ESTOP_LATCHED'
        before = len(mcu.payloads)
        mcu.pump(t+300)
        mcu.pump(t+300)
        for n, payload in enumerate(mcu.payloads[before:]):
            record(item, payload, origin+(t+400)*1000+n)
        late = record(item, ack(request), origin+(t+500)*1000)
        assert late['reason'] == 'unmatched_ack' and item.arm_barrier is None
        assert mcu.row['target_speed_mps'] == 0
    finally:
        mcu.close()


def test_production_nodes_do_not_import_the_candidate():
    root = Path(__file__).resolve().parents[1]
    for package in ('arena_autonomy', 'arena_vehicle_interface'):
        for path in (root/'src'/package/package).glob('*.py'):
            for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
                if isinstance(node, ast.ImportFrom):
                    assert node.module != 'actuation_host_session'
                elif isinstance(node, ast.Import):
                    assert all(name.name != 'actuation_host_session' for name in node.names)
