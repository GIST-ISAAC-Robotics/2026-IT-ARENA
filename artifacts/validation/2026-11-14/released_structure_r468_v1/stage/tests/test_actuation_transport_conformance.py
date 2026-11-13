"""분할 바이트/정지/만료의 결합 계약. 실제 UART·펌웨어의 시간 성능은 측정하지 않는다."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import struct
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
spec = importlib.util.spec_from_file_location('actuation_transport_conformance',
                                            ROOT / 'scripts/actuation_transport_conformance.py')
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)


@pytest.fixture(scope='module')
def cases():
    return bench.make_cases()


def case_named(cases, name):
    return next(case for case in cases if case['name'] == name)


def test_cases_checkpoints_and_replay(cases):
    assert len(cases) == len({case['name'] for case in cases}) == 12
    for case in cases:
        assert case['checkpoints']
        assert bench.compare_case(case, bench.replay(case)) is None
        for checkpoint in case['checkpoints']:
            status = case['events'][checkpoint['event_index']]['expected']['status']
            for key, value in checkpoint['expected_status'].items():
                assert status[key] == value
        for checkpoint in case['transport_checkpoints']:
            response = case['events'][checkpoint['event_index']]['expected']
            for key, value in checkpoint['expected'].items():
                if key == 'deliveries':
                    assert [item['result'] for item in response['result']] == value
                elif key == 'buffered_bytes':
                    assert response['decoder']['buffered_bytes'] == value
                else:
                    assert key == 'min_errors' and response['decoder']['errors'] >= value


def test_deterministic_and_json_roundtrip(cases):
    assert bench.make_cases() == cases
    for case in json.loads(json.dumps(cases, allow_nan=False)):
        assert bench.compare_case(case, bench.replay(case)) is None


def test_motor_mode_and_existing_time_limits(cases):
    for case in cases:
        cfg = case['config']
        assert cfg['feedback_mode'] == 'drive_motor_shaft'
        assert (cfg['ticks_per_rev'], cfg['gear_ratio'], cfg['wheel_radius_m']) == (28, 15., .0325)
        assert (cfg['lease_us'], cfg['feedback_timeout_us'], cfg['max_control_gap_us']) == (100_000, 50_000, 50_000)
        assert all(row['expected']['status']['left_wheel_mps'] is None for row in case['events'])


def test_known_pending_length_limitation_is_not_claimed_immediate_stop(cases):
    case = case_named(cases, 'plausible_partial_length_hides_software_stop')
    feed = next(row for row in case['events'] if row['input']['op'] == 'bytes' and
                row['expected']['decoder']['buffered_bytes'] > 0)
    assert feed['expected']['result'] == []
    assert feed['input']['now_us'] == 220_000
    assert feed['expected']['status']['target_speed_mps'] == 1.
    last = case['events'][-1]
    assert last['input']['now_us'] == 320_000
    assert last['expected']['status']['reason'] == 'command_expired'
    assert last['expected']['status']['target_speed_mps'] == 0.
    assert last['expected']['decoder']['buffered_bytes'] > 0


def test_physical_stop_does_not_need_parser_to_complete(cases):
    case = case_named(cases, 'physical_stop_independent_of_pending_parser')
    pressed = next(row for row in case['events'] if row['input']['op'] == 'physical_stop')
    assert pressed['input']['now_us'] == 230_000
    assert pressed['expected']['decoder']['buffered_bytes'] > 0
    assert pressed['expected']['status']['target_speed_mps'] == 0.
    assert pressed['expected']['status']['reason'] == 'physical_stop'
    assert case['events'][-1]['expected']['status']['state'] == 'ESTOP_LATCHED'


def test_stop_intermediate_and_final_outputs_are_preserved(cases):
    case = case_named(cases, 'coalesced_stop_rejects_queued_drive')
    deliveries = case['events'][-1]['expected']['result']
    assert [row['status']['target_speed_mps'] for row in deliveries] == [.5, 0., 0.]
    assert deliveries[0]['status']['generation'] + 1 == deliveries[1]['status']['generation']


def test_reset_only_clears_parser_not_receiver(cases):
    case = case_named(cases, 'decoder_reset_drops_tail_but_does_not_stop_or_rearm')
    index = next(i for i, row in enumerate(case['events']) if row['input']['op'] == 'decoder_reset')
    before, after = case['events'][index-1]['expected'], case['events'][index]['expected']
    assert before['status'] == after['status']
    assert before['decoder']['buffered_bytes'] > 0
    assert after['decoder']['buffered_bytes'] == 0


@pytest.mark.parametrize('damage', ['drop_event', 'target', 'result', 'buffer', 'clock_type'])
def test_corrupted_observation_not_silently_accepted(cases, damage):
    case = case_named(cases, 'coalesced_stop_rejects_queued_drive')
    observed = deepcopy(bench.replay(case))
    last = observed['responses'][-1]
    if damage == 'drop_event':
        observed['responses'].pop()
    elif damage == 'target':
        last['status']['target_speed_mps'] = .5
    elif damage == 'result':
        last['result'][-1]['result'] = [True, 'drive']
    elif damage == 'buffer':
        last['decoder']['buffered_bytes'] += 1
    else:
        last['status']['feedback_capture_us'] = float(last['status']['feedback_capture_us'])
    assert bench.compare_case(case, observed)


@pytest.mark.parametrize('split', range(1, len(bench.encode(bench.STOP))))
def test_every_stop_split_is_atomic_at_complete_frame(split):
    b, _ = bench.active('split_stop')
    frame = bench.encode(bench.STOP)
    assert b.feed(frame[:split]) == []
    assert b.receiver.target_speed == 1.
    response = b.feed(frame[split:])
    assert response[0]['result'] == [True, 'software_stop']
    assert b.receiver.target_speed == 0.


@pytest.mark.parametrize('chunk_size', [1, 2, 3, 7, 16, 32, 64, 4096])
def test_chunking_does_not_change_drive_stop_order(chunk_size):
    b, _ = bench.active('chunked_stream')
    first, last = b.packet(.5), b.packet(.8)
    stream = bench.encode(first) + bench.encode(bench.STOP) + bench.encode(last)
    deliveries = []
    for index in range(0, len(stream), chunk_size):
        deliveries.extend(b.feed(stream[index:index+chunk_size]))
    assert [row['result'] for row in deliveries] == [[True, 'drive'], [True, 'software_stop'], [False, 'session']]
    assert b.receiver.target_speed == 0.


@pytest.mark.parametrize('length', [128, 256, 512, 1536])
def test_pending_legal_length_does_not_create_keepalive(length):
    b, _ = bench.active('pending_length')
    pending = bench.MAGIC + struct.pack('<H', length) + bench.encode(bench.STOP)
    assert len(pending) < length + 8
    assert b.feed(pending) == []
    b.advance(100_000)
    assert b.receiver.reason == 'command_expired'
    assert b.receiver.target_speed == 0.


@pytest.mark.parametrize('offset', [0, 1, 2, 3, 4, 10, -5, -4, -3, -2, -1])
def test_single_bit_error_does_not_deliver_new_drive(offset):
    b, _ = bench.active('bit_error')
    damaged = bytearray(bench.encode(b.packet(.7)))
    damaged[offset] ^= 1
    assert b.feed(bytes(damaged)) == []
    assert b.receiver.target_speed == 1. and b.receiver.deadline_us == 320_000
    b.advance(100_000)
    assert b.receiver.target_speed == 0. and b.receiver.reason == 'command_expired'
