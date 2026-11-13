"""C++ 바이트 코어와 Python 참조 비교. 보드 없는 제한 포팅 검사."""
import importlib.util
import os
from pathlib import Path
import struct
import sys
import zlib

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
spec = importlib.util.spec_from_file_location('validate_actuation_wire_cpp', ROOT/'scripts/validate_actuation_wire_cpp.py')
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)


@pytest.fixture(scope='module')
def cases():
    return bench.make_cases()


@pytest.fixture(scope='module')
def cpp_result(cases):
    path = os.environ.get('ARENA_WIRE_CPP_BINARY')
    if not path:
        pytest.skip('C++ core requires explicit prebuilt ARENA_WIRE_CPP_BINARY')
    return bench.evaluate(Path(path), cases)


def test_known_crc_standard_vector():
    assert zlib.crc32(b'123456789') == 0xcbf43926
    assert bench.expected_line(bench.FramingOracle(), ('C', b'123456789')) == 'C;3421780262'


def test_size_limit_and_actual_header():
    frame = bench.frame(b'abc')
    assert frame[:4] == b'IA\x03\x00'
    assert struct.unpack('<I', frame[-4:])[0] == zlib.crc32(b'\x03\x00abc')
    oracle = bench.FramingOracle()
    assert bench.expected_line(oracle, ('E', b'')) == 'E;REJECT'
    assert bench.expected_line(oracle, ('E', b'x'*1537)) == 'E;REJECT'


def test_crc_valid_payload_is_not_json_validation():
    raw = b'\xff\x00not-json'
    oracle = bench.FramingOracle()
    assert oracle.feed(bench.frame(raw)) == (True, [raw])
    assert bench.Decoder().feed(bench.frame(raw)) == []


def test_pending_legal_length_remains_pending_until_reset():
    oracle = bench.FramingOracle()
    pending = b'IA\x00\x06' + bench.frame(b'{"v":1,"kind":"STOP"}')
    assert oracle.feed(pending) == (True, [])
    assert oracle.feed(b'') == (True, [])
    assert oracle.pending == pending
    bench.expected_line(oracle, ('R', b''))
    assert oracle.pending == b''


def test_chunk_budget_clears_pending_with_visible_rejection():
    oracle = bench.FramingOracle()
    oracle.feed(b'I')
    assert oracle.feed(b'0'*257) == (False, [])
    assert oracle.errors == 1 and oracle.pending == b''


def test_cases_are_repeatable_and_include_state_contract_streams(cases):
    assert bench.make_cases() == cases
    names = [case['name'] for case in cases]
    assert len(names) == len(set(names))
    assert len([name for name in names if name.startswith('contract_')]) == 12
    assert len([name for name in names if name.startswith('random_stream_')]) == 160


def test_existing_python_json_boundary_parity(cases):
    comparisons, failures = bench.json_boundary_parity(cases)
    assert comparisons > 500
    assert failures == []


def test_cpp_core_matches_all_frozen_operations(cpp_result):
    report, stdin, stdout, stderr = cpp_result
    assert report['passed'], (report['framing_mismatches'][:3], stderr[:1000])
    assert report['binary_exit_code'] == 0 and not stderr
    assert len(stdin.splitlines()) == len(stdout.splitlines()) == report['operations']
    assert not report['framing_mismatches'] and not report['json_boundary_mismatches']


def test_cpp_result_does_not_claim_full_mcu_port(cpp_result):
    report = cpp_result[0]
    for field in ('json_validation_ported', 'contract_state_machine_ported',
                  'partial_frame_timeout_implemented', 'bounded_parser_time_measured',
                  'firmware_verified', 'ready_to_drive', 'runtime_connected'):
        assert report[field] is False
