"""부분 프레임 절대 기한·연결/오류 정책 후보. C++ 링크와 독립 oracle·참조 정책 대조(보드 없음)."""
import importlib.util
import os
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/f'scripts/{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ref = load('actuation_link_reference')
bench = load('validate_actuation_link_cpp')
STOP = bench.STOP
PERMISSIVENESS = dict(ESTOP_LATCHED=0, FAULT_LATCHED=0, DISARMED=1, ARMED=2, ACTIVE=3)


@pytest.fixture(scope='module')
def link_cases():
    return bench.make_link_cases()


@pytest.fixture(scope='module')
def policy_cases():
    return ref.make_policy_cases()


@pytest.fixture(scope='module')
def cpp_result(link_cases, policy_cases):
    path = os.environ.get('ARENA_LINK_CPP_BINARY')
    if not path:
        pytest.skip('C++ link requires explicit prebuilt ARENA_LINK_CPP_BINARY')
    return bench.evaluate(Path(path), link_cases, policy_cases)


def run(timeout, ops):
    oracle = ref.LinkOracle(timeout)
    return [oracle.apply(op) for op in ops]


def test_deadline_boundary_is_half_open():
    for at, expired in [(1_099, False), (1_100, True), (1_101, True)]:
        last = run(1_000, [('C', 0), ('F', 100, b'IA\x05'), ('P', at)])[-1]
        assert last['timeouts'] == int(expired)
        assert (last['anchor'] is None) == expired


def test_trickle_and_empty_feeds_do_not_move_anchor():
    ops = [('C', 0), ('F', 0, b'IA\x00\x06')] + [('F', t, b'x') for t in range(1, 1_000)]
    ops += [('F', 999, b''), ('P', 999)]
    snapshots = run(1_000, ops)
    assert {s['anchor'] for s in snapshots[1:]} == {0}
    assert run(1_000, ops + [('P', 1_000)])[-1]['buffer'] == ''


def test_anchor_moves_only_when_all_old_bytes_are_consumed():
    stop = STOP
    kept = run(1_000, [('C', 0), ('F', 0, b'IA\x00\x06' + stop[:5]), ('F', 500, stop[5:] + stop[:3])])[-1]
    assert kept['anchor'] == 0 and kept['frames'] == []
    renewed = run(1_000, [('C', 0), ('F', 0, stop[:5]), ('F', 500, stop[5:] + stop[:3])])[-1]
    assert renewed['anchor'] == 500 and renewed['frames'] == [['L', stop[4:-4].hex()]]


def test_drain_delivers_only_frames_already_complete_and_leaves_nothing():
    last = run(10, [('C', 0), ('F', 0, b'IA\x00\x06' + STOP + STOP[:7]), ('F', 10, STOP[7:])])[-1]
    assert last['frames'] == [['X', STOP[4:-4].hex()]]
    assert last['buffer'] == '' and last['timeouts'] == 1


def test_session_and_fault_boundaries_drop_without_delivery():
    snapshots = run(10**9, [('C', 0), ('F', 1, b'IA\x00\x06' + STOP), ('D', 2), ('C', 3),
                            ('F', 4, STOP[:3]), ('C', 5), ('F', 6, STOP[:3]), ('P', 5)])
    assert snapshots[2]['frames'] == [] and snapshots[2]['dropped'] == 33
    assert snapshots[5]['breaks'] == 2 and snapshots[5]['dropped'] == 36
    assert snapshots[7]['fault'] == 'clock_regressed' and snapshots[7]['breaks'] == 3


def test_clear_fault_only_returns_to_disconnected():
    snapshots = run(10, [('C', 5), ('P', 4), ('X', 0), ('F', 1, STOP), ('C', 1), ('F', 2, STOP)])
    assert [s['result'] for s in snapshots] == ['ok', 'fault_raised', 'ok', 'not_connected', 'ok', 'ok']
    assert snapshots[2]['state'] == 'disconnected'


def test_uint64_inputs_are_bounded():
    with pytest.raises(ValueError):
        ref.LinkOracle(2**64)
    with pytest.raises(ValueError):
        ref.LinkOracle(1).apply(('P', 2**64))
    assert ref.LinkOracle(0).state == 'faulted'


def test_oracle_manual_checks_and_invariants(link_cases):
    names = [case['name'] for case in link_cases]
    assert len(names) == len(set(names))
    assert len([n for n in names if n.startswith('random_link_')]) == 240
    assert len([n for n in names if n.startswith('framing_')]) == 241
    assert sum(len(case['checks']) for case in link_cases) >= 60
    for case in link_cases:
        bench.expected_lines(case)


def test_oracle_detects_trickle_refreshing_anchor(link_cases):
    class Refreshing(ref.LinkOracle):
        def _feed(self, now, data, null, frames):
            result = super()._feed(now, data, null, frames)
            if result == 'ok' and self.pending:
                self.anchor = now  # 결함: 조각마다 기한 연장
            return result
    case = next(c for c in link_cases if c['name'] == 'trickle_cannot_extend_deadline')
    original = bench.LinkOracle
    bench.LinkOracle = Refreshing
    try:
        with pytest.raises(AssertionError):
            bench.expected_lines(case)
    finally:
        bench.LinkOracle = original


def test_policy_cases_replay_with_oracle_link(policy_cases):
    assert len(policy_cases) == len({c['name'] for c in policy_cases}) == 10
    for case in policy_cases:
        assert case['checkpoints'] and case['link_timeout'] == ref.TEST_ASSEMBLY_TIMEOUT_US
        oracle = ref.LinkOracle(case['link_timeout'])
        snapshots = [oracle.apply(op) for op in ref.link_ops(case)]
        assert ref.compare_policy_case(case, ref.replay_policy(case, snapshots)) is None
        previous = 'DISARMED'
        for event in case['events']:
            state = event['expected']['status']['state']
            if event['input']['op'] == 'link':
                # 수락된 명시 명령 프레임 없이 연결/poll/reset/drain만으로 더 허용적인 상태가 되지 않는다.
                accepted = {d.get('kind') for d in event['expected']['result']['deliveries'] if d['result'][0]}
                if not accepted & {'CLEAR', 'ARM', 'DRIVE'}:
                    assert PERMISSIVENESS[state] <= PERMISSIVENESS[previous], (case['name'], previous, state)
            previous = state


def test_policy_without_session_break_latch_is_rejected(monkeypatch):
    original = ref.apply_policy_link

    def no_latch(receiver, policy, now, snapshot):
        return original(receiver, dict(policy, breaks_seen=snapshot['breaks']), now, snapshot)
    monkeypatch.setattr(ref, 'apply_policy_link', no_latch)
    with pytest.raises(AssertionError):
        ref.make_policy_cases()


def test_hidden_stop_now_surfaces_before_command_expiry(policy_cases):
    case = next(c for c in policy_cases if c['name'] == 'hidden_stop_surfaces_at_assembly_deadline')
    final = case['events'][case['checkpoints'][-1]['event_index']]['expected']['status']
    assert final['state'] == 'ESTOP_LATCHED' and final['reason'] == 'software_stop'
    assert ref.TEST_ASSEMBLY_TIMEOUT_US < 100_000  # 시험값일 뿐 운영값이 아니다.


def test_cpp_link_matches_oracle_and_policy(cpp_result):
    report, stdin, stdout, stderr = cpp_result
    assert report['passed'], (report['link_mismatches'][:3], report['policy_replay_failures'][:3], stderr[:1000])
    assert report['binary_exit_code'] == 0 and not stderr
    assert len(stdin.splitlines()) == len(stdout.splitlines()) == report['driver_lines']


def test_cpp_result_does_not_claim_production_or_firmware(cpp_result):
    report = cpp_result[0]
    assert report['test_timeout_values_only'] is True
    for field in ('production_timeout_selected', 'ros_or_runtime_connected', 'json_validation_ported',
                  'contract_state_machine_ported', 'wcet_measured', 'firmware_verified', 'ready_to_drive'):
        assert report[field] is False
