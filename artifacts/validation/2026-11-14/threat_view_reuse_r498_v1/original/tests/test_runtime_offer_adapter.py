"""실제 writer 필드의 순수 분리 계약. ROS/serial/장치/컴파일을 사용하지 않는다."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import sys

import pytest

import runtime_offer_adapter as adapter
from actuation_host_observer import CompactHostObserver, validate_payload
from arena_vehicle_interface.actuation_wire import Decoder, encode
from arena_vehicle_interface.clock_freshness import HostTimeWindow
from arena_vehicle_interface.motor_feedback_observation import decode_motor_feedback


def compact(**changes):
    return dict(type='offer', v=1, boot='1234abcd', generation=2, state='ARMED', owner='host',
                lease=8, lease_issued_us=20000, lease_expires_us=120000, output_ready=True) | changes


def extended(**changes):
    return compact() | dict(status_now_us=20000, estop_held=False, reason='explicit_arm',
                           motor_feedback_v1=dict(valid=True, sequence=2, capture_us=19000, counts=-1)) | changes


def adapt(row):
    return adapter.adapt_runtime_payload(row, max_lease_us=100000)


@pytest.mark.parametrize('ready', [False, True])
@pytest.mark.parametrize('form', ['compact', 'status', 'motor'])
def test_explicit_forms_and_readiness_preserved(ready, form):
    row = compact(output_ready=ready) if form == 'compact' else extended(output_ready=ready)
    if form == 'status': row.pop('motor_feedback_v1')
    original = deepcopy(row)
    result = adapt(row)
    assert row == original
    assert result.compact == compact(output_ready=ready)
    validate_payload(result.compact, 100000)
    assert result.compact['output_ready'] is ready
    if form == 'compact':
        assert result.status is result.feedback is None
    else:
        with pytest.raises(ValueError): validate_payload(row, 100000)
        assert result.status['boot'] == result.compact['boot']
        assert result.status['generation'] == result.compact['generation']
        assert result.status['status_now_us'] == result.compact['lease_issued_us']
        assert result.status['output_ready'] is ready
        assert result.status['scope'] == 'observation_only_no_drive'
        if form == 'motor':
            assert result.feedback['boot'] == result.status['boot']
            assert result.feedback['generation'] == result.status['generation']
            assert result.feedback['status_now_us'] == result.status['status_now_us']
            assert result.feedback['mcu_capture_us'] == 19000 and result.feedback['motor_counts'] == -1
        else:
            assert result.feedback is None


@pytest.mark.parametrize('raw', [dict(valid=False, sequence=None, capture_us=None, counts=None),
                                dict(valid=False, sequence=2, capture_us=19000, counts=-1)])
def test_unavailable_and_stale_feedback_are_not_zero_speed(raw):
    result = adapt(extended(motor_feedback_v1=raw))
    assert result.feedback['source_valid'] is False
    assert result.feedback['motor_counts'] == raw['counts']
    assert result.feedback['mcu_capture_us'] == raw['capture_us']
    assert not result.feedback['time_contract_verified']
    assert 'speed_mps' not in result.feedback and 'estimated_capture_ns' not in result.feedback


@pytest.mark.parametrize('fault', ['unknown_type', 'runtime_observation', 'unknown_key', 'partial_status',
                                  'feedback_without_status', 'nested_identity', 'status_time', 'bool_time',
                                  'negative_time', 'estop_type', 'long_reason', 'bad_ready', 'owner_conflict'])
def test_unknown_fields_conflicts_and_bad_status_rejected(fault):
    row = extended()
    if fault == 'unknown_type': row['type'] = 'mystery'
    if fault == 'runtime_observation': row['type'] = 'runtime_drive_observation_v1'
    if fault == 'unknown_key': row['future_feature'] = True
    if fault == 'partial_status': del row['reason']
    if fault == 'feedback_without_status':
        for key in adapter.STATUS_KEYS: del row[key]
    if fault == 'nested_identity': row['motor_feedback_v1']['generation'] = row['generation']+1
    if fault == 'status_time': row['status_now_us'] += 1
    if fault == 'bool_time': row['status_now_us'] = True
    if fault == 'negative_time': row['status_now_us'] = -1
    if fault == 'estop_type': row['estop_held'] = 0
    if fault == 'long_reason': row['reason'] = 'x'*33
    if fault == 'bad_ready': row['output_ready'] = 1
    if fault == 'owner_conflict': row['owner'] = None
    with pytest.raises(ValueError): adapt(row)


@pytest.mark.parametrize('raw', [dict(valid=True, sequence=None, capture_us=None, counts=None),
    dict(valid=False, sequence=1, capture_us=None, counts=0),
    dict(valid=True, sequence=1, capture_us=20001, counts=0),
    dict(valid=True, sequence=True, capture_us=19000, counts=0),
    dict(valid=True, sequence=1, capture_us=19000, counts=True),
    dict(valid=True, sequence=1, capture_us=19000, counts=2**63),
    dict(valid=1, sequence=1, capture_us=19000, counts=0)])
def test_malformed_feedback_rejects_entire_split(raw):
    with pytest.raises(ValueError): adapt(extended(motor_feedback_v1=raw))


def test_existing_motor_decoder_agrees_on_raw_fields_and_rejects_other_identity():
    row = extended()
    parts = adapt(row)
    # helper 재사용 대조를 위한 합성 mapped 상태다. 어댑터는 이를 생성하지 않는다.
    status = dict(boot=row['boot'], generation=row['generation'], mcu_now_us=row['status_now_us'],
                  now_us=1000000, host_receipt_ns=1001000000, clock_epoch=1, time_basis='synthetic_only')
    observation = decode_motor_feedback(row, status)
    for key in ('boot', 'generation', 'source_valid', 'sequence', 'motor_counts', 'mcu_capture_us'):
        assert parts.feedback[key] == observation[key]
    for key, value in [('boot', 'other'), ('generation', 3), ('mcu_now_us', 20001)]:
        with pytest.raises(ValueError): decode_motor_feedback(row, status | {key: value})


def test_existing_wire_decoder_and_compact_observer_receive_same_offer():
    row = extended()
    wire = encode(row)
    decoder = Decoder()
    assert decoder.feed(wire[:11]) == []
    payloads = decoder.feed(wire[11:])
    assert decoder.errors == 0 and len(payloads) == 1
    parts = adapt(payloads[0])
    observer = CompactHostObserver(boot='1234abcd', host_epoch='host-1', connection_id='link', max_lease_us=100000)
    result = observer.observe(parts.compact, received=HostTimeWindow('host-1', 1000000000, 1000000000),
                              connection_id='link')
    assert result['accepted'] and not result['ready_to_drive']
    assert observer.latest('offer').payload == compact()
    assert parts.status['status_now_us'] == 20000


def test_compact_ack_state_and_diag_pass_without_extensions_or_aliases():
    rows = [dict(type='ack', request='req1', accepted=True, reason='explicit_arm'),
            dict(type='state', v=1, boot='1234abcd', generation=2, state='ARMED', reason='explicit_arm', changes=1),
            dict(type='diag', v=1, boot='1234abcd', now_us=20000, generation=2, state='ARMED',
                 reason='explicit_arm', accepted_seq=1, target_speed_umps=0, steering_urad=0,
                 deadline_us=120000, estop_held=False, feedback_mode='drive_motor_shaft', feedback_valid=False,
                 feedback_seq=None, feedback_capture_us=None, motor_counts=None, left_ticks=None, right_ticks=None,
                 mean_wheel_umps=None, output_ready=True, deadline_miss_count=0, lease_count=1,
                 tx={key: 0 for key in ('ack_dropped', 'ack_evicted', 'ack_purged', 'state_merged', 'diag_skipped',
                                       'aborted_frames', 'encode_failures')})]
    for row in rows:
        result = adapt(row)
        assert result.compact == row and result.compact is not row
        assert result.status is result.feedback is None
        with pytest.raises(ValueError): adapt(row | {'status_now_us': 20000})
    result.compact['tx']['ack_dropped'] = 9
    assert rows[-1]['tx']['ack_dropped'] == 0


def test_mutating_input_and_split_does_not_change_other_parts():
    row = extended(); parts = adapt(row)
    row['motor_feedback_v1']['counts'] = 9
    row['boot'] = 'other'
    assert parts.feedback['motor_counts'] == -1 and parts.compact['boot'] == '1234abcd'
    parts.compact['generation'] = 9
    assert parts.status['generation'] == parts.feedback['generation'] == 2


def test_full_expanded_wire_limit_is_checked_before_return():
    # 64 scalar 문자열은 compact 계약에서 허용되지만 합친 확장 프레임은 한도를 넘을 수 있다.
    row = extended(boot='𐀀'*64, owner='𐀀'*40, reason='𐀀'*32)
    core = {key: row[key] for key in adapter.OFFER_KEYS}
    validate_payload(core, 100000)
    with pytest.raises(ValueError, match='runtime payload limit'): adapt(row)
