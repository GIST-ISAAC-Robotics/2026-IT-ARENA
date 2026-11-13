"""출력 없는 compact 메시지 보관 후보. 합성 payload와 실제 C++ TX 프레임 대조."""
from dataclasses import FrozenInstanceError
from pathlib import Path
import shutil

import pytest

from actuation_host_observer import CompactHostObserver
from audit_actuation_offer_clock import fixture
from audit_actuation_tx_policy import compile_driver
from arena_vehicle_interface.actuation_wire import Decoder, encode
from arena_vehicle_interface.clock_freshness import HostTimeWindow
from test_actuation_tx_policy import Mcu


def observer(**kwargs):
    return CompactHostObserver(boot='boot-A', host_epoch='host-epoch-A', connection_id='connection-A',
                               max_lease_us=100_000, **kwargs)


def record(obs, payload, when=9_010_000_000, **kwargs):
    options = dict(received=HostTimeWindow('host-epoch-A', when, when), connection_id='connection-A')
    return obs.observe(payload, **(options | kwargs))


def offer(**kwargs):
    return fixture()[0] | kwargs


def state(**kwargs):
    return dict(type='state', v=1, boot='boot-A', generation=3, state='ACTIVE', reason='drive', changes=1) | kwargs


def diag(**kwargs):
    return dict(type='diag', v=1, boot='boot-A', now_us=1_000_100, generation=3, state='ACTIVE', reason='drive',
                accepted_seq=5, target_speed_umps=500_000, steering_urad=-100_000, deadline_us=1_100_100,
                estop_held=False, feedback_mode='drive_motor_shaft', feedback_valid=True, feedback_seq=10,
                feedback_capture_us=1_000_000, motor_counts=-10, left_ticks=None, right_ticks=None,
                mean_wheel_umps=510_000, output_ready=True, deadline_miss_count=0, lease_count=10,
                tx=dict(ack_dropped=0, ack_evicted=0, ack_purged=0, state_merged=0, diag_skipped=0,
                        aborted_frames=0, encode_failures=0)) | kwargs


def review(obs, now_ns=9_020_000_000):
    _, fit, _, now, options = fixture(now_ns=now_ns)
    return obs.review_offer_time(fit, now, **{k: options[k] for k in
                                ('max_issue_age_ns', 'forward_delay_upper_ns', 'policy_note')})


def test_preserves_original_receipt_and_has_no_drive_permission():
    obs = observer()
    assert record(obs, offer())['accepted']
    saved = obs.latest('offer')
    data = saved.payload
    data['lease_issued_us'] = 2_000_000
    assert saved.payload['lease_issued_us'] == 1_000_100
    with pytest.raises(FrozenInstanceError):
        saved.payload_json = '{}'
    assert review(obs)['timestamp_policy_satisfied']
    assert review(obs)['ready_to_drive'] is False
    assert not hasattr(obs, 'send') and not hasattr(obs, 'drive') and not hasattr(obs, 'arm')


def test_other_messages_do_not_rejuvenate_offer():
    obs = observer()
    record(obs, offer())
    first = obs.latest('offer')
    for index, payload in enumerate([state(), diag(), dict(type='ack', request='old-request', accepted=True, reason='ok')]):
        assert record(obs, payload, 9_015_000_000+index)['accepted']
        assert obs.latest('offer') is first
    assert review(obs, 9_120_000_000)['timestamp_policy_satisfied'] is False
    assert len(obs._latest) == 4
    assert obs.latest('ack').payload['accepted'] is True  # 서비스 성공/ARM 허가로 해석하지 않음


@pytest.mark.parametrize('duplicate', [offer(), offer(lease=7), offer(output_ready=False)])
def test_duplicate_offer_does_not_refresh_any_receipt(duplicate):
    obs = observer()
    record(obs, offer())
    saved = obs.latest('offer')
    result = record(obs, duplicate, 9_019_000_000)
    assert not result['accepted'] and result['reason'] == 'duplicate_or_old_lease'
    assert obs.latest('offer') is saved
    assert review(obs)['timestamp_policy_satisfied']


def test_new_lease_with_regressed_issue_is_not_adopted():
    obs = observer()
    record(obs, offer())
    assert record(obs, offer(lease=9, lease_issued_us=1_000_000, lease_expires_us=1_100_000), 9_011_000_000)['reason'] == 'issue_regressed'
    assert obs.latest('offer').payload['lease'] == 8
    assert record(obs, offer(lease=10, lease_issued_us=1_000_100), 9_012_000_000)['accepted']


@pytest.mark.parametrize('new', [state(generation=4, state='ESTOP_LATCHED', reason='software_stop'),
                                 diag(generation=4, state='FAULT_LATCHED', reason='command_expired')])
def test_new_generation_invalidates_old_offer_even_without_new_offer(new):
    obs = observer()
    record(obs, offer())
    assert record(obs, new, 9_011_000_000)['accepted']
    assert obs.generation == 4 and obs.latest('offer') is None
    assert review(obs)['status'] == 'no_current_offer'
    assert record(obs, offer(lease=9), 9_012_000_000)['reason'] == 'old_generation'
    assert record(obs, offer(generation=4, lease=10, state='DISARMED', owner=None, output_ready=False), 9_013_000_000)['accepted']
    result = review(obs)
    assert result['timestamp_policy_satisfied'] and result['ready_to_drive'] is False


def test_new_generation_cannot_reset_per_boot_lease_watermark():
    obs = observer()
    record(obs, offer())
    assert not record(obs, offer(generation=4, lease=8), 9_011_000_000)['accepted']
    assert obs.generation == 4 and obs.latest('offer') is None


def test_duplicate_diag_does_not_refresh_receipt():
    obs = observer()
    record(obs, diag())
    saved = obs.latest('diag')
    assert record(obs, diag(), 9_011_000_000)['reason'] == 'duplicate_or_old_diag'
    assert obs.latest('diag') is saved
    assert record(obs, diag(now_us=1_000_101), 9_012_000_000)['accepted']


@pytest.mark.parametrize('foreign', [dict(connection_id='old-connection'),
                                    dict(received=HostTimeWindow('old-host', 5, 5))])
def test_stale_external_context_cannot_replace_or_regress_current_session(foreign):
    obs = observer()
    record(obs, offer())
    saved = obs.latest('offer')
    assert record(obs, offer(lease=9), 1, **foreign)['reason'] == 'context_mismatch'
    assert not obs.closed and obs.latest('offer') is saved
    assert review(obs)['timestamp_policy_satisfied']


def test_foreign_boot_is_not_implicitly_adopted():
    obs = observer()
    record(obs, offer())
    assert record(obs, offer(boot='boot-B', generation=100), 9_011_000_000)['reason'] == 'boot_mismatch'
    assert obs.boot == 'boot-A' and obs.generation == 3


def test_local_clock_regression_closes_without_implicit_recovery():
    obs = observer()
    record(obs, offer())
    assert record(obs, diag(), 9_009_999_999)['reason'] == 'clock_regressed'
    assert obs.closed and not obs._latest
    assert record(obs, offer(lease=9), 9_020_000_000)['reason'] == 'closed'
    assert review(obs)['timestamp_policy_satisfied'] is False


def test_disconnect_invalidates_and_old_connection_data_is_rejected_on_new_instance():
    obs = observer()
    record(obs, offer())
    obs.disconnect()
    assert obs.latest('offer') is None and review(obs)['status'] == 'no_current_offer'
    new = CompactHostObserver(boot='boot-A', host_epoch='host-epoch-A', connection_id='connection-B', max_lease_us=100_000)
    assert record(new, offer())['reason'] == 'context_mismatch'
    assert new.generation is None


def test_receipt_uncertainty_is_not_replaced_by_an_exact_timestamp():
    obs = observer()
    window = HostTimeWindow('host-epoch-A', 9_000_000_000, 9_010_000_000)
    assert record(obs, offer(), received=window)['accepted']
    assert obs.latest('offer').received == window
    assert review(obs)['conditions']['receive_order_certain'] is False
    with pytest.raises(ValueError):
        review(obs, 9_009_999_999)


@pytest.mark.parametrize('payload', [None, [], dict(type=[]), dict(type='unknown'), offer(v=True),
    offer(lease=True), offer(lease=0), offer(lease_expires_us=1_100_101), offer(lease_issued_us=-1),
    offer(owner=None), offer(state='DISARMED'), offer(output_ready=1), offer(extra=0), state(changes=True),
    state(reason='x'*65), state(state=[]), diag(accepted_seq=1.), diag(lease_count=33), diag(tx=[]),
    diag(tx={'unknown': 0}), diag(feedback_valid=1), diag(motor_counts=2**63), diag(feedback_seq=-1),
    diag(feedback_mode=[]), dict(type='ack', request='', accepted=True, reason='ok'),
    dict(type='ack', request='r', accepted=1, reason='ok'), dict(type='ack', request='r', accepted=True, reason='ok', boot='b')])
def test_bad_schema_is_not_saved(payload):
    obs = observer()
    assert record(obs, payload)['reason'] == 'schema'
    assert not obs._latest and obs.generation is None


def test_history_and_latest_storage_are_bounded():
    obs = observer(history_limit=4)
    for n in range(100):
        result = record(obs, dict(type='ack', request=str(n), accepted=True, reason='ok'), 9_010_000_000+n)
        assert result['ack_correlated'] is False
    assert len(obs.history) == 4 and obs.history_dropped == 96
    assert len(obs._latest) == 1 and obs.accepted == 100
    assert obs.latest('offer') is None and obs.generation is None


def test_actual_cpp_transmit_frames_are_accepted_and_stop_generation_removes_old_offer(tmp_path):
    if not shutil.which('g++'):
        pytest.fail('C++ frame integration requires g++')
    binary = compile_driver(tmp_path/'tx-driver')
    mcu = Mcu(binary, boot='boot-A')
    obs = observer()
    processed = 0
    cursor_ns = 9_000_000_000
    kinds = set()

    def consume():
        nonlocal processed, cursor_ns
        for payload in mcu.payloads[processed:]:
            cursor_ns += 1
            result = record(obs, payload, cursor_ns)
            assert result['accepted'], (payload, result)
            kinds.add(payload['type'])
        processed = len(mcu.payloads)

    try:
        now = mcu.setup()
        consume()
        assert obs.latest('offer').payload['state'] == 'DISARMED'
        mcu.control(now, 'ARM', request='arm-A')
        mcu.pump(now)
        mcu.pump(now)
        consume()
        assert obs.latest('offer').payload['state'] == 'ARMED'
        assert obs.latest('ack').payload['accepted']
        old = obs.latest('offer')
        mcu.send(now+1, dict(v=1, kind='STOP'), request='stop-A')
        mcu.pump(now+1)
        mcu.pump(now+1)
        consume()
        assert obs.latest('offer').payload['state'] == 'ESTOP_LATCHED'
        assert obs.latest('offer').payload['generation'] > old.payload['generation']
        assert kinds == {'offer', 'ack', 'state', 'diag'}
        # 받은 ACK는 세대/boot 상관 확인이 없으므로 허가로 승격되지 않는다.
        assert obs.closed is False
    finally:
        mcu.close()


def test_existing_crc_decoder_fragmentation_keeps_same_payloads():
    payloads = [offer(), state(), diag(), dict(type='ack', request='r', accepted=True, reason='ok')]
    wire = b''.join(encode(p) for p in payloads)
    for chunk in (1, 7, 256, len(wire)):
        decoder, obs = Decoder(), observer()
        for offset in range(0, len(wire), chunk):
            for payload in decoder.feed(wire[offset:offset+chunk]):
                assert record(obs, payload)['accepted']
        assert decoder.errors == 0
        assert {kind: obs.latest(kind).payload for kind in ('offer', 'state', 'diag', 'ack')} == {p['type']: p for p in payloads}


def test_production_nodes_do_not_import_the_candidate():
    import ast
    root = Path(__file__).resolve().parents[1]
    for package in ('arena_autonomy', 'arena_vehicle_interface'):
        for path in (root/'src'/package/package).glob('*.py'):
            for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
                if isinstance(node, ast.ImportFrom):
                    assert node.module != 'actuation_host_observer'
                elif isinstance(node, ast.Import):
                    assert all(name.name != 'actuation_host_observer' for name in node.names)
