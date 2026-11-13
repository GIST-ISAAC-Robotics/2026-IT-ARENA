"""C++ 명령 parser(엄격 JSON·명령 필드 표현)와 독립 기준·실제 receive 대조. 보드/상태기계 이관 아님."""
from copy import deepcopy
import importlib.util
import os
from pathlib import Path
import struct
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


ref = load('actuation_command_reference')
bench = load('validate_actuation_command_cpp')


@pytest.fixture(scope='module')
def plan():
    return bench.build_plan()


@pytest.fixture(scope='module')
def cpp_result(plan):
    path = os.environ.get('ARENA_COMMAND_CPP_BINARY')
    if not path:
        pytest.skip('C++ parser requires explicit prebuilt ARENA_COMMAND_CPP_BINARY')
    return bench.evaluate(Path(path), plan)


def test_reference_status_matches_production_decoder(plan):
    for row in plan['rows'] + plan['receive']:
        status, obj = ref.parse_reference(row['payload'])
        decoded = ref.production_decode(row['payload'])
        assert (status == 'object') == (decoded is not None), row['name']
        if decoded is not None:
            assert decoded == obj, row['name']


def test_manual_payload_expectations_hold_in_reference(plan):
    checked = 0
    for row in plan['rows']:
        if row['expect']:
            checked += 1
            assert bench.check_expect(row['expect'], ref.reference_line(row['payload'], row['layout'])) == {}, row['name']
    assert checked >= 150


def test_error_order_follows_python(plan):
    names = {row['name']: row for row in plan['rows']}
    # 상수는 만나는 즉시, 중복 키는 객체가 닫힐 때 거절된다.
    assert ref.parse_reference(names['constant_before_dup_close']['payload'])[0] == 'nonfinite_constant'
    assert ref.parse_reference(names['duplicate_inner_before_constant']['payload'])[0] == 'duplicate_key'
    assert ref.parse_reference(names['dup_never_closed_is_syntax']['payload'])[0] == 'syntax'


def test_manual_receive_expectations_hold_without_cpp(plan):
    rows = {row['name']: row for row in plan['receive']}
    assert set(bench.MANUAL_RECEIVE) <= set(rows)
    for name, manual in bench.MANUAL_RECEIVE.items():
        row = rows[name]
        s = plan['states'][row['state']]
        receiver = deepcopy(s['receiver'])
        result = ref.deliver(receiver, row['layout'], ref.production_decode(row['payload']), s['now'])
        status = receiver.status()
        fields = bench.line_fields(ref.reference_line(row['payload'], row['layout']))
        observed = dict(fields, result=result, state=status['state'], reason=status['reason'],
                        target=status['target_speed_mps'])
        assert {k: observed.get(k) for k in manual} == manual, name


def test_invalid_drive_is_represented_not_dropped(plan):
    row = next(r for r in plan['receive'] if r['name'] == 'active_DRIVE_speed_mps_palette_5')
    fields = bench.line_fields(ref.reference_line(row['payload'], 0))
    assert fields == dict(status='object', cls='drive', session=1, drive=0)


def test_reconstruction_preserves_receive_outcome_for_reference_lines(plan):
    for row in plan['receive']:
        s = plan['states'][row['state']]
        left, right = deepcopy(s['receiver']), deepcopy(s['receiver'])
        a = ref.deliver(left, row['layout'], ref.production_decode(row['payload']), s['now'])
        b = ref.deliver(right, row['layout'], ref.reconstruct(ref.reference_line(row['payload'], row['layout'])), s['now'])
        assert bench.compare_values(dict(r=a, s=left.status()), dict(r=b, s=right.status())) is None, row['name']


def test_float_tokens_use_python_float_bits():
    for token, expected in [('1e-400', 0.0), ('-1e-400', -0.0), ('5e-324', 5e-324), ('2.5000000000000002', 2.5)]:
        line = ref.reference_line(('{"v":1,"kind":"DRIVE","boot":"b","generation":0,"lease":1,"client":"c","seq":0,'
                                   f'"speed_mps":{token},"steering_rad":0,"source_age_us":0,"ttl_us":1}}').encode(), 0)
        speed = line.split(';')[16]
        assert speed == 'D' + struct.pack('>d', expected).hex()
    assert ref.reference_line(b'{"a":1e400}', 0).startswith('P;0')


def test_case_inventory_is_repeatable(plan):
    names = [row['name'] for row in plan['rows'] + plan['receive']]
    assert len(names) == len(set(names))
    assert plan['ros_samples'] is True
    again = bench.build_plan()
    assert [r['payload'] for r in again['rows']] == [r['payload'] for r in plan['rows']]
    assert len([n for n in names if n.startswith('fuzz_')]) == 6000
    assert len(plan['streams']) == 12


def test_cpp_matches_reference_and_receive(cpp_result):
    report, stdin, stdout, stderr = cpp_result
    assert report['passed'], (report['representation_mismatches'][:3], report['manual_failures'][:3],
                              report['receive_equivalence_failures'][:3], stderr[:1000])
    assert report['binary_exit_code'] == 0 and not stderr
    assert len(stdout.splitlines()) == report['driver_lines']


def test_cpp_report_does_not_claim_state_machine_or_firmware(cpp_result):
    report = cpp_result[0]
    assert report['json_command_representation_ported'] is True
    for field in ('contract_state_machine_ported', 'ros_or_runtime_connected', 'wcet_measured',
                  'mcu_toolchain_verified', 'firmware_verified', 'ready_to_drive'):
        assert report[field] is False
