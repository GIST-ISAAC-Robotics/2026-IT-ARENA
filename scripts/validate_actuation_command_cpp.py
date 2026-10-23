#!/usr/bin/env python3
"""C++ 명령 parser(엄격 JSON·명령 필드 표현)의 PC 대조. 상태기계·펌웨어·운영 연결 검증 아님."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import random
import subprocess
import zipfile

from actuation_conformance import ROOT, Builder, compare_values, digest, make_cases as conformance_cases
from actuation_command_reference import (
    CLASSES, DEFAULT_LIMITS, STATUS, bits, deliver, parse_reference, production_decode, reconstruct,
    reference_line,
)
from actuation_transport_conformance import make_cases as transport_cases
import validate_actuation_wire_cpp as wire
from arena_vehicle_interface.actuation_wire import Decoder, MAX_PAYLOAD

FILES = [
    'scripts/validate_actuation_command_cpp.py', 'scripts/actuation_command_reference.py',
    'scripts/actuation_command_cpp_driver.cpp', 'tests/test_actuation_command_cpp.py',
    'src/arena_vehicle_interface/portable/actuation_command.hpp',
    'src/arena_vehicle_interface/portable/actuation_link.hpp',
    'src/arena_vehicle_interface/portable/actuation_wire.hpp',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_wire.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_contract.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_ros.py',
    'scripts/actuation_conformance.py', 'scripts/actuation_transport_conformance.py',
    'scripts/validate_actuation_wire_cpp.py', 'scripts/audit_actuation_link_budget.py',
]
STOP = b'{"v":1,"kind":"STOP"}'


class Raw:
    """JSON 직렬화 결과에 그대로 넣을 토큰(예: 1e400, -0, 2.5000000000000002)."""
    def __init__(self, text):
        self.text = text

    def __repr__(self):
        return f'Raw({self.text!r})'


def escape_all(text):
    return '"' + ''.join(f'\\u{ord(ch):04x}' if ord(ch) < 0x10000 else
                         ''.join(f'\\u{unit:04x}' for unit in
                                 [0xD800 + ((ord(ch) - 0x10000) >> 10), 0xDC00 + ((ord(ch) - 0x10000) & 0x3FF)])
                         for ch in text) + '"'


def dump(value, *, ascii=True, sep=(',', ':'), escape_keys=False, order=None):
    if isinstance(value, Raw):
        return value.text
    if type(value) is dict:
        keys = list(value) if order is None else order(list(value))
        items = [(escape_all(k) if escape_keys else json.dumps(k, ensure_ascii=ascii)) + sep[1] +
                 dump(value[k], ascii=ascii, sep=sep, escape_keys=escape_keys, order=order) for k in keys]
        return '{' + sep[0].join(items) + '}'
    if type(value) is list:
        return '[' + sep[0].join(dump(v, ascii=ascii, sep=sep, escape_keys=escape_keys, order=order)
                                 for v in value) + ']'
    return json.dumps(value, ensure_ascii=ascii)


def enc(value, **options):
    return dump(value, **options).encode('utf-8', 'surrogatepass')


# ---------------------------------------------------------------- 문법·경계 corpus

def corpus():
    """(이름, layout, payload, 수동 기대값). 수동 기대값은 구현과 별개로 적은 판정이다."""
    rows = []

    def add(name, payload, layout=0, **expect):
        rows.append(dict(name=name, layout=layout, payload=payload if type(payload) is bytes else
                         payload.encode('utf-8', 'surrogatepass'), expect=expect))

    add('stop_exact', STOP, status='object', cls='stop')
    add('stop_whitespace', ' \t\n\r{ "v" : 1 ,\n"kind":"STOP" }\r\n', status='object', cls='stop')
    add('stop_key_order', '{"kind":"STOP","v":1}', cls='stop')
    add('stop_escaped_kind', '{"v":1,"kind":"\\u0053TOP"}', cls='stop')
    add('stop_escaped_keys', '{"\\u0076":1,"\\u006b\\u0069nd":"STOP"}', cls='stop')
    add('stop_extra_field_is_schema', '{"v":1,"kind":"STOP","x":0}', cls='schema')
    add('stop_lowercase_is_schema', '{"v":1,"kind":"stop"}', cls='schema')
    add('kind_not_string_is_schema', '{"v":1,"kind":["STOP"]}', cls='schema')
    add('empty_object_invalid_message', '{}', cls='invalid_message')
    add('v_float_invalid', '{"v":1.0,"kind":"STOP"}', cls='invalid_message')
    add('v_exponent_invalid', '{"v":1e0,"kind":"STOP"}', cls='invalid_message')
    add('v_bool_invalid', '{"v":true,"kind":"STOP"}', cls='invalid_message')
    add('v_string_invalid', '{"v":"1","kind":"STOP"}', cls='invalid_message')
    add('v_negative_zero_int', '{"v":-0,"kind":"STOP"}', cls='invalid_message')
    add('v_bigint_invalid', '{"v":18446744073709551617,"kind":"STOP"}', cls='invalid_message')
    for name, text in [('empty', ''), ('open', '{'), ('close', '}'), ('trailing_comma', '{"v":1,}'),
                       ('missing_comma', '{"v":1 "kind":2}'), ('missing_colon', '{"v"1}'),
                       ('bare_key', '{v:1}'), ('single_quote', "{'v':1}"), ('leading_zero', '{"v":01}'),
                       ('dot_end', '{"v":1.}'), ('dot_start', '{"v":.5}'), ('exp_empty', '{"v":1e}'),
                       ('exp_sign_only', '{"v":1e+}'), ('plus', '{"v":+1}'), ('double_minus', '{"v":--1}'),
                       ('minus_only', '{"v":-}'), ('hex', '{"v":0x10}'), ('two_dots', '{"v":1.5.2}'),
                       ('tru', '{"v":tru}'), ('True', '{"v":True}'), ('nul', '{"v":nul}'),
                       ('unterminated', '{"v":"abc}'), ('bad_escape', '{"v":"\\x"}'), ('short_u', '{"v":"\\u12"}'),
                       ('bad_u', '{"v":"\\u12G4"}'), ('raw_newline', '{"v":"a\nb"}'), ('raw_nul', '{"v":"\x00"}'),
                       ('raw_1f', '{"v":"\x1f"}'), ('garbage_after', '{"v":1} x'), ('two_objects', '{"v":1}{}'),
                       ('array_trailing', '{"a":[1,]}'), ('array_leading', '{"a":[,1]}'),
                       ('array_unclosed', '{"a":[}'), ('array_bad_close', '{"a":]}'), ('obj_bad_close', '{"a":{]}'),
                       ('extra_close', '{"a":1}}'), ('mismatch_close', '{"a":1]'), ('double_colon', '{"a"::1}'),
                       ('nonascii_digit_after_dot', '{"a":1.١}'), ('dotless_i', '[1.ı]'),
                       ('neg_inf_partial', '{"a":-Inf}'), ('inf_partial', '{"a":Inf}'), ('nan_lower', '{"a":nan}'),
                       ('surrogate_then_bad_u', '{"a":"\\ud800\\uZZZZ"}'), ('ws_only', ' \n')]:
        add('syntax_' + name, text, status='invalid_size' if text == '' else 'syntax')
    add('raw_del_is_valid', '{"v":"\x7f"}', status='object')
    add('trailing_ws_valid', '{"v":1}\n\t', status='object')
    add('empty_key_valid', '{"":1}', status='object')
    add('bom_rejected', b'\xef\xbb\xbf{}', status='syntax')
    for name, text in [('nan', '{"a":NaN}'), ('inf', '{"a":Infinity}'), ('neg_inf', '{"a":-Infinity}'),
                       ('nan_suffix', '{"a":NaNx}'), ('nested', '{"a":[1,{"b":Infinity}]}'),
                       ('before_dup_close', '{"a":1,"a":2,"b":NaN}')]:
        add('constant_' + name, text, status='nonfinite_constant')
    for name, text in [('plain', '{"v":1,"v":1}'), ('escaped', '{"v":1,"\\u0076":1}'),
                       ('pair_vs_utf8', '{"\\ud83d\\ude00":1,"\U0001F600":2}'),
                       ('lone_surrogate', '{"\\ud800":1,"\\ud800":2}'),
                       ('lone_then_ascii', '{"\\ud800\\u0041":1,"\\ud800A":2}'),
                       ('solidus', '{"a\\/b":1,"a/b":2}'), ('nested', '{"x":{"a":1,"a":2}}'),
                       ('inner_before_constant', '{"a":{"b":1,"b":2},"c":NaN}'),
                       ('in_array_then_garbage', '[{"a":1,"a":2}] x'),
                       ('envelope_command', '{"command":{},"command":{}}')]:
        add('duplicate_' + name, text, layout=1 if 'envelope' in name else 0, status='duplicate_key')
    add('dup_never_closed_is_syntax', '{"a":1,"a":2', status='syntax')
    add('case_differs_not_dup', '{"A":1,"a":2}', status='object')
    add('same_key_in_sibling_objects', '{"x":[{"a":1},{"a":1}]}', status='object')
    add('inner_key_reused_by_outer', '{"x":{"a":1},"a":2}', status='object')
    for name, data in [('ff', b'{"a":"\xff"}'), ('overlong_2', b'{"a":"\xc0\xaf"}'),
                       ('overlong_3', b'{"a":"\xe0\x80\xaf"}'), ('surrogate', b'{"a":"\xed\xa0\x80"}'),
                       ('above_max', b'{"a":"\xf4\x90\x80\x80"}'), ('truncated', b'{"a":"\xe2\x82"}'),
                       ('lone_continuation', b'{"a":"\x80"}'), ('outside_string', b'{"a":1}\xc3'),
                       ('f5', b'{"a":"\xf5\x80\x80\x80"}')]:
        add('utf8_' + name, data, status='invalid_utf8')
    for name, data in [('two_byte', '{"a":"é"}'), ('three_byte', '{"a":"€"}'),
                       ('four_byte', '{"a":"\U0001F600"}'), ('max', '{"a":"\U0010FFFF"}'),
                       ('fffe', '{"a":"￾"}'), ('lone_surrogate_escape', '{"a":"\\ud800"}'),
                       ('low_surrogate_escape', '{"a":"\\udc00\\ud800"}')]:
        add('utf8_valid_' + name, data, status='object')
    for name, text in [('int', '1'), ('string', '"x"'), ('array', '[]'), ('null', 'null'), ('true', 'true'),
                       ('array_of_object', '[{"v":1,"kind":"STOP"}]')]:
        add('not_object_' + name, text, status='not_object')
    depth = (MAX_PAYLOAD - len('{"a":}')) // 2
    add('max_nesting', '{"a":' + '[' * depth + ']' * depth + '}', status='object')
    add('deep_unclosed', '{"a":' + '[' * (MAX_PAYLOAD - 6) + ']', status='syntax')
    add('deep_objects', '{"a":' + '{"a":' * 250 + '1' + '}' * 251, status='object')
    pad = MAX_PAYLOAD - len('{"v":1,"kind":"STOP","p":""}')
    add('size_1536_schema', '{"v":1,"kind":"STOP","p":"' + 'x' * pad + '"}', status='object', cls='schema')
    add('size_1537_rejected', '{"v":1,"kind":"STOP","p":"' + 'x' * (pad + 1) + '"}', status='invalid_size')
    add('long_number_string', '{"a":0.' + '0' * 1400 + '1}', status='object')
    add('long_bigint', '{"a":1' + '0' * 1400 + '}', status='object')
    add('huge_exponent', '{"a":1e-9999999999999999999999,"b":1e9999999999999999999}', status='object')
    # envelope 배치
    add('env_stop', '{"type":"command","request":"","command":{"v":1,"kind":"STOP"}}', layout=1, cls='stop')
    add('env_type_offer_ignored', '{"type":"offer","command":{"v":1,"kind":"STOP"}}', layout=1,
        cls='envelope_ignored')
    add('env_type_escaped', '{"type":"\\u0063ommand","command":{"v":1,"kind":"STOP"}}', layout=1, cls='stop')
    add('env_command_array_ignored', '{"type":"command","command":[]}', layout=1, cls='envelope_ignored')
    add('env_command_missing_ignored', '{"type":"command","v":1,"kind":"STOP"}', layout=1, cls='envelope_ignored')
    add('env_nested_command_not_used', '{"type":"command","x":{"command":{"v":1,"kind":"STOP"}},"command":{}}',
        layout=1, cls='invalid_message')
    add('env_extra_keys_allowed', '{"z":0,"type":"command","command":{"v":1,"kind":"STOP"},"y":[1]}', layout=1,
        cls='stop')
    add('env_request_types', '{"type":"command","request":5,"command":{"v":1,"kind":"STOP"}}', layout=1, cls='stop')
    add('env_request_64', '{"type":"command","request":"' + 'r' * 64 + '","command":{}}', layout=1)
    add('env_request_65', '{"type":"command","request":"' + 'r' * 65 + '","command":{}}', layout=1)
    arm = '{"v":1,"kind":"ARM","boot":"b","generation":0,"lease":1,"seq":0,"client":%s}'
    add('client_escaped_pair', arm % '"\\ud83d\\ude00"', cls='arm', session=1)
    add('client_raw_astral', arm % '"\U0001F600"', cls='arm', session=1)
    add('client_lone_high_then_ascii', arm % '"\\ud800\\u0041"', cls='arm', session=1)
    add('client_low_then_high', arm % '"\\udc00\\ud800"', cls='arm', session=1)
    add('client_64_with_pair', arm % ('"' + 'a' * 63 + '\\ud83d\\ude00"'), cls='arm', session=1)
    add('client_65_with_pair', arm % ('"' + 'a' * 64 + '\\ud83d\\ude00"'), cls='arm', session=0)
    add('client_64_raw_astral', arm % ('"' + '\U0001F600' * 64 + '"'), cls='arm', session=1)
    add('client_empty', arm % '""', cls='arm', session=0)
    add('boot_escaped_pair', '{"v":1,"kind":"ARM","boot":"\\ud83d\\ude00","generation":0,"lease":1,"seq":0,"client":"c"}',
        cls='arm', session=1)
    add('bare_layout_ignores_envelope', '{"type":"command","command":{"v":1,"kind":"STOP"}}', cls='invalid_message')
    for token in ['1e400', '-1e400', '1e-400', '-1e-400', '5e-324', '3e-324', '2e-324', '1e-320',
                  '2.2250738585072011e-308', '1.7976931348623157e308', '1.7976931348623159e308',
                  '2.5000000000000002', '2.5000000000000003', '0.37000000000000005', '-0.0', '0e999999',
                  '1E+2', '0.1e1', '123456789012345678901234567890e-29', '9007199254740993',
                  '9223372036854775807', '9223372036854775808', '-9223372036854775808', '-9223372036854775809',
                  '18446744073709551616', '-0', '0', '100000', '4294967295', '4294967296']:
        add('number_' + token, '{"a":' + token + '}', status='object')
    return rows


# ---------------------------------------------------------------- 실제 producer/ROS/기존 벡터

def producer_cases():
    rows = []
    for case in conformance_cases():
        for index, row in enumerate(case['events']):
            if row['input']['op'] == 'command':
                payload = bytes.fromhex(row['input']['args']['frame_hex'])[4:-4]
                rows.append(dict(name=f"conformance_{case['name']}_{index}", layout=0, payload=payload, expect={}))
    b = Builder('host_controls').prepare(False)
    offer = b.event('offer')
    for kind in ('ARM', 'CLEAR', 'DISARM'):
        command = b.producer.control(offer, kind)
        envelope = {'type': 'command', 'request': 'f' * 32, 'command': command}
        rows.append(dict(name=f'host_control_{kind}', layout=1, payload=enc(envelope), expect={}))
    for label, request in [('service', 'e' * 32), ('watch', '')]:
        rows.append(dict(name=f'host_stop_{label}', layout=1, expect={'cls': 'stop'},
                         payload=enc({'type': 'command', 'request': request, 'command': {'v': 1, 'kind': 'STOP'}})))
    try:
        from audit_actuation_link_budget import sample_frames
    except ImportError:  # ROS 메시지 패키지가 없으면 명시적으로 빠진다.
        samples = None
    else:
        samples = sample_frames()
    for sample in samples or []:
        for key, payload in sample['payloads'].items():
            rows.append(dict(name=f"ros_{sample['name']}_{key}", layout=1, payload=enc(payload),
                             expect={'cls': 'drive'} if key == 'drive' else
                             {'cls': 'stop'} if key == 'service_stop_shape' else {'cls': 'envelope_ignored'}))
    return rows, samples is not None


def wire_streams():
    """기존 전송 계약 12사례의 바이트·재연결 순서를 FrameLink+parser로 공급한다."""
    streams = []
    for case in transport_cases():
        ops = []
        for row in case['events']:
            event = row['input']
            if event['op'] == 'bytes':
                ops.append(('W', bytes.fromhex(event['args']['hex'])))
            elif event['op'] in ('decoder_reset', 'reboot'):
                ops.append(('R', b''))
        streams.append(dict(name='transport_' + case['name'], ops=ops))
    return streams


# ---------------------------------------------------------------- receive 동등성

def receive_states():
    states = {}
    b = Builder('active').prepare(); b.drive(1.); states['active'] = b
    b = Builder('armed').prepare(); states['armed'] = b
    b = Builder('disarmed').prepare(False); states['disarmed'] = b
    b = Builder('estop').prepare(); b.send({'v': 1, 'kind': 'STOP'}); b.advance(220_000); states['estop'] = b
    out = {}
    for name, b in states.items():
        offer = b.event('offer')
        out[name] = dict(receiver=deepcopy(b.receiver), now=b.now, boot=offer['boot'],
                         generation=offer['generation'], lease=offer['lease'], client=b.producer.client,
                         seq=max(b.receiver.sequence, b.producer.sequence) + 1)
    return out


PALETTE = [None, True, False, [], {}, 'x', '', 0, 1, -1, 1.0, 2**63 - 1, 2**63, -(2**63), -(2**63) - 1, 2**64,
           Raw('-0'), Raw('-0.0'), Raw('1e400'), Raw('-1e400'), Raw('1e-400'), Raw('5e-324'), Raw('1E2'),
           Raw('0.5e0'), [1], {'a': 1}]


def variants(base, s):
    yield 'canonical', base, {}
    yield 'pretty', base, dict(sep=(', ', ': '))
    yield 'reversed', base, dict(order=lambda keys: keys[::-1])
    yield 'sorted', base, dict(order=sorted)
    yield 'escaped_keys', base, dict(escape_keys=True)
    yield 'non_ascii', base, dict(ascii=False)
    for field in list(base):
        removed = dict(base); del removed[field]
        yield f'remove_{field}', removed, {}
        for index, value in enumerate(PALETTE):
            yield f'{field}_palette_{index}', dict(base, **{field: value}), {}
    yield 'extra_key', dict(base, extra=1), {}
    specific = {
        'speed_mps': [2.5, Raw('2.5000000000000002'), Raw('2.5000000000000003'), 2.4999999999999996, Raw('2.50'),
                      3, 2, Raw('0.' + '0' * 300 + '1'), Raw('1' + '0' * 300), .5, Raw('5e-324')],
        'steering_rad': [.37, -.37, Raw('0.37000000000000005'), Raw('-0.37000000000000005'),
                         Raw('0.37000000000000004'), Raw('-0'), -0.0],
        'ttl_us': [100_000, 100_001, 1, 0, Raw('100000.0')],
        'source_age_us': [99_999, 100_000, Raw('-0')],
        'seq': [s['seq'] - 1, s['seq'], 2**32 - 1, 2**32, Raw('1e0')],
        'generation': [s['generation'] + 1, -1, Raw(str(s['generation']) + '.0')],
        'lease': [s['lease'] + 1, 0, 2**63 - 1],
        'client': ['', 'a' * 64, 'a' * 65, s['client'] + '\ud800', Raw(escape_all(s['client']))],
        'boot': [s['boot'] + 'x', 'x' * 65, Raw(escape_all(s['boot']))],
        'kind': ['stop', 'STOP', Raw('"\\u0044RIVE"'), 1],
    }
    for field, values in specific.items():
        if field in base:
            for index, value in enumerate(values):
                yield f'{field}_specific_{index}', dict(base, **{field: value}), {}


def receive_cases():
    states = receive_states()
    plan = {'active': ['DRIVE', 'DISARM', 'STOP', 'ARM', 'CLEAR'], 'armed': ['DRIVE', 'DISARM'],
            'disarmed': ['ARM', 'DRIVE', 'CLEAR'], 'estop': ['CLEAR', 'ARM']}
    rows = []
    for state, kinds in plan.items():
        s = states[state]
        for kind in kinds:
            if kind == 'STOP':
                base = {'v': 1, 'kind': 'STOP'}
            else:
                base = dict(v=1, kind=kind, boot=s['boot'], generation=s['generation'], lease=s['lease'],
                            client=s['client'], seq=s['seq'])
                if kind == 'DRIVE':
                    base.update(speed_mps=.5, steering_rad=.1, source_age_us=0, ttl_us=100_000)
            for label, message, options in variants(base, s):
                payload = enc(message, **options)
                rows.append(dict(name=f'{state}_{kind}_{label}', state=state, layout=0, payload=payload))
                if (state, kind) in (('active', 'DRIVE'), ('disarmed', 'ARM'), ('estop', 'CLEAR')):
                    envelope = enc({'type': 'command', 'request': 'q' * 32, 'command': Raw(payload.decode(
                        'utf-8', 'surrogatepass'))})
                    rows.append(dict(name=f'{state}_{kind}_{label}_envelope', state=state, layout=1,
                                     payload=envelope))
    return states, rows


# 구현과 별개로 적은 상태/결과 기대값(기존 receive 의미). 표현이 이 의미를 보존하는지 본다.
MANUAL_RECEIVE = {
    'active_DRIVE_canonical': dict(cls='drive', session=1, drive=1, result=[True, 'drive'], state='ACTIVE'),
    'active_DRIVE_speed_mps_palette_5': dict(cls='drive', session=1, drive=0, result=[False, 'invalid_drive'],
                                             state='FAULT_LATCHED', reason='invalid_drive'),
    'active_DRIVE_speed_mps_palette_1': dict(drive=0, result=[False, 'invalid_drive'], state='FAULT_LATCHED'),
    'active_DRIVE_speed_mps_palette_15': dict(drive=0, result=[False, 'invalid_drive'], state='FAULT_LATCHED'),
    'active_DRIVE_speed_mps_palette_0': dict(drive=0, result=[False, 'invalid_drive'], state='FAULT_LATCHED'),
    'active_DRIVE_speed_mps_specific_1': dict(drive=1, result=[True, 'drive'], target=2.5),
    'active_DRIVE_speed_mps_specific_2': dict(drive=0, result=[False, 'invalid_drive'], state='FAULT_LATCHED'),
    'active_DRIVE_speed_mps_palette_20': dict(drive=1, result=[True, 'zero_target'], state='ARMED'),
    'active_DRIVE_speed_mps_palette_21': dict(drive=1, result=[True, 'drive'], state='ACTIVE'),
    'active_DRIVE_speed_mps_palette_17': dict(drive=1, result=[True, 'zero_target'], state='ARMED'),
    'active_DRIVE_seq_specific_3': dict(session=0, result=[False, 'session'], state='ACTIVE'),
    'active_DRIVE_generation_specific_0': dict(session=1, result=[False, 'session'], state='ACTIVE'),
    'active_DRIVE_client_specific_2': dict(session=0, result=[False, 'session'], state='ACTIVE'),
    'active_DRIVE_client_specific_1': dict(session=1, result=[False, 'not_owner_or_armed'], state='ACTIVE'),
    'active_DRIVE_client_specific_4': dict(result=[True, 'drive']),
    'active_DRIVE_boot_specific_2': dict(result=[True, 'drive']),
    'active_DRIVE_lease_specific_0': dict(result=[False, 'expired_lease'], state='ACTIVE'),
    'active_DRIVE_ttl_us_specific_1': dict(drive=0, result=[False, 'invalid_drive'], state='FAULT_LATCHED'),
    'active_DRIVE_source_age_us_specific_1': dict(drive=0, result=[False, 'invalid_drive'], state='FAULT_LATCHED'),
    'active_DRIVE_kind_specific_2': dict(cls='drive', result=[True, 'drive']),
    'active_DRIVE_kind_specific_1': dict(cls='schema', result=[False, 'schema'], state='ACTIVE'),
    'active_DRIVE_extra_key': dict(cls='schema', result=[False, 'schema'], state='ACTIVE'),
    'active_DRIVE_v_palette_10': dict(cls='invalid_message', result=[False, 'invalid_message'], state='ACTIVE'),
    'active_DRIVE_v_palette_1': dict(cls='invalid_message', result=[False, 'invalid_message']),
    'active_DRIVE_escaped_keys': dict(cls='drive', result=[True, 'drive']),
    'active_DRIVE_canonical_envelope': dict(cls='drive', result=[True, 'drive']),
    'active_STOP_extra_key': dict(cls='schema', result=[False, 'schema'], state='ACTIVE'),
    'active_STOP_canonical': dict(cls='stop', result=[True, 'software_stop'], state='ESTOP_LATCHED'),
    'disarmed_ARM_canonical': dict(cls='arm', result=[True, 'explicit_arm'], state='ARMED'),
    'disarmed_DRIVE_canonical': dict(result=[False, 'not_owner_or_armed'], state='DISARMED'),
    'estop_CLEAR_canonical': dict(cls='clear', result=[True, 'explicit_clear'], state='DISARMED'),
    'estop_ARM_canonical': dict(result=[False, 'not_disarmed'], state='ESTOP_LATCHED'),
    'armed_DRIVE_speed_mps_palette_5': dict(drive=0, result=[False, 'invalid_drive'], state='FAULT_LATCHED'),
}


# ---------------------------------------------------------------- 무작위 변이

def fuzz_cases(seeds, count=6000):
    rng = random.Random(0x20261022)
    tokens = [b'"', b'\\', b'{', b'}', b'[', b']', b',', b':', b'u', b'0', b'-', b'e', b'.', b' ', b'\x80',
              b'\xff', b'N', b'\\u0000', b'\\ud800', b'1e400', b'"v"', b'"kind"', b'true', b'null']
    rows = []
    for index in range(count):
        seed = rng.choice(seeds)
        data = bytearray(seed['payload'])
        for _ in range(rng.randrange(1, 4)):
            choice = rng.random()
            at = rng.randrange(len(data) + 1)
            if choice < .3 and data:
                data[min(at, len(data) - 1)] ^= 1 << rng.randrange(8)
            elif choice < .5 and data:
                del data[min(at, len(data) - 1)]
            elif choice < .8:
                data[at:at] = rng.choice(tokens)
            elif data:
                end = min(len(data), at + rng.randrange(1, 12))
                data[at:at] = data[at:end]
        rows.append(dict(name=f'fuzz_{index}', layout=seed['layout'], payload=bytes(data[:MAX_PAYLOAD]), expect={}))
    return rows


# ---------------------------------------------------------------- 평가

def line_fields(line):
    parts = line.split(';')
    status = STATUS[int(parts[1])]
    if status != 'object':
        return dict(status=status)
    return dict(status=status, cls=CLASSES[int(parts[22])], session=int(parts[23]), drive=int(parts[24]))


def check_expect(expect, line):
    fields = line_fields(line)
    return {k: (v, fields.get(k)) for k, v in expect.items()
            if k in ('status', 'cls', 'session', 'drive') and fields.get(k) != v}


def build_plan():
    rows = corpus()
    producer, ros_samples = producer_cases()
    rows += producer
    states, receive = receive_cases()
    seeds = [r for r in rows + receive if parse_reference(r['payload'])[0] == 'object']
    rows += fuzz_cases(seeds)
    return dict(rows=rows, receive=receive, states=states, streams=wire_streams(), ros_samples=ros_samples)


def evaluate(binary, plan):
    commands, expected, owners = [], [], []
    alt_limits = (1.0, .5, 50_000)
    limit_line = lambda limits: f'L {bits(limits[0])} {bits(limits[1])} {limits[2]}'
    commands.append('S'); expected.append(None); owners.append('sizes')  # 값은 보고만 한다.
    commands.append(limit_line(DEFAULT_LIMITS)); expected.append('L;1'); owners.append('limits_default')
    for row in plan['rows'] + plan['receive']:
        commands.append(f"P {row['layout']} {row['payload'].hex()}")
        expected.append(reference_line(row['payload'], row['layout']))
        owners.append(row['name'])
    # 한도는 호출자 입력이다. 다른 지원 한도와 미지원 한도(2^63)를 구분한다.
    commands.append(limit_line(alt_limits)); expected.append('L;1'); owners.append('limits_alt')
    alt_rows = [r for r in plan['receive'] if r['name'].startswith('active_DRIVE_')]
    for row in alt_rows:
        commands.append(f"P {row['layout']} {row['payload'].hex()}")
        expected.append(reference_line(row['payload'], row['layout'], alt_limits))
        owners.append('alt_' + row['name'])
    commands.append(f'L {bits(2.0**63)} {bits(.37)} 100000'); expected.append('L;0'); owners.append('limits_unsupported')
    commands.append(limit_line(DEFAULT_LIMITS)); expected.append('L;1'); owners.append('limits_restore')
    oracle = wire.FramingOracle()
    production = Decoder()
    production_stream_mismatch = []
    for stream in plan['streams']:
        for op, data in stream['ops']:
            if op == 'R':
                commands.append('R'); expected.append('R'); owners.append(stream['name'])
                oracle.pending = b''; production.reset()
                continue
            commands.append(f'W 0 {data.hex()}')
            payloads = []
            for start in range(0, len(data), wire.CHUNK):
                chunk = data[start:start + wire.CHUNK]
                raw = oracle.feed(chunk)[1]
                payloads += raw
                # 같은 청크에서 운영 Decoder가 전달한 dict == 기준 판정이 객체인 payload들.
                if production.feed(chunk) != [obj for status, obj in map(parse_reference, raw) if status == 'object']:
                    production_stream_mismatch.append(stream['name'])
            for payload in payloads:
                expected.append(reference_line(payload, 0)); owners.append(stream['name'])
            expected.append(f'W;{len(payloads)}'); owners.append(stream['name'])
    stdin = '\n'.join(commands) + '\n'
    run = subprocess.run([str(binary)], input=stdin, capture_output=True, text=True, timeout=600)
    actual = run.stdout.splitlines()
    mismatches = [dict(index=i, case=owners[i], expected=e[:300], actual=actual[i][:300] if i < len(actual) else None)
                  for i, e in enumerate(expected)
                  if i >= len(actual) or (actual[i] != e if e is not None else not actual[i].startswith('S;'))]
    sizes = dict(zip(('parser_bytes', 'command_bytes', 'frame_link_bytes'),
                     map(int, actual[0].split(';')[1:]))) if actual and actual[0].startswith('S;') else None
    if len(actual) > len(expected):
        mismatches.append(dict(error='unexpected extra output', lines=len(actual) - len(expected)))
    by_name = {}
    for owner, line in zip(owners, actual):
        by_name.setdefault(owner, line)
    # 실제 운영 Decoder와 기준 상태 판정의 일치(전달 여부와 dict 값).
    production_mismatch = []
    for row in plan['rows'] + plan['receive']:
        status, obj = parse_reference(row['payload'])
        decoded = production_decode(row['payload'])
        if (status == 'object') != (decoded is not None) or (decoded is not None and decoded != obj):
            production_mismatch.append(row['name'])
    # 수동 기대값(corpus·producer)
    manual_failures = []
    manual_count = 0
    for row in plan['rows']:
        if row['expect']:
            manual_count += 1
            ref = reference_line(row['payload'], row['layout'])
            for label, line in (('reference', ref), ('cpp', by_name.get(row['name'], ''))):
                diff = check_expect(row['expect'], line) if line else {'missing': True}
                if diff:
                    manual_failures.append(dict(name=row['name'], source=label, diff=str(diff)))
    # receive 동등성: 원래 JSON(운영 Decoder) vs C++ 표현에서 되살린 dict.
    receive_failures, receive_count = [], 0
    states = plan['states']
    outcome_counts = {}
    for row in plan['receive']:
        s = states[row['state']]
        line = by_name.get(row['name'])
        if line is None:
            receive_failures.append(dict(name=row['name'], error='no cpp line'))
            continue
        original, rebuilt = deepcopy(s['receiver']), deepcopy(s['receiver'])
        message = production_decode(row['payload'])
        left = deliver(original, row['layout'], message, s['now'])
        right = deliver(rebuilt, row['layout'], reconstruct(line), s['now'])
        receive_count += 1
        key = str(left if isinstance(left, str) else left[1])
        outcome_counts[key] = outcome_counts.get(key, 0) + 1
        error = compare_values(dict(result=left, status=original.status()),
                               dict(result=right, status=rebuilt.status()))
        if error:
            receive_failures.append(dict(name=row['name'], error=error))
        manual = MANUAL_RECEIVE.get(row['name'])
        if manual:
            fields = line_fields(line)
            status = original.status()
            observed = dict(fields, result=left, state=status['state'], reason=status['reason'],
                            target=status['target_speed_mps'])
            diff = {k: (v, observed.get(k)) for k, v in manual.items() if observed.get(k) != v}
            if diff:
                manual_failures.append(dict(name=row['name'], source='receive', diff=str(diff)))
    missing_manual = sorted(set(MANUAL_RECEIVE) - {r['name'] for r in plan['receive']})
    report = dict(
        payload_cases=len(plan['rows']), receive_cases=len(plan['receive']), wire_streams=len(plan['streams']),
        driver_lines=len(expected), binary_exit_code=run.returncode, representation_mismatches=mismatches,
        production_decoder_mismatches=production_mismatch + production_stream_mismatch,
        manual_checks=manual_count + len(MANUAL_RECEIVE), manual_failures=manual_failures,
        missing_manual_cases=missing_manual, receive_equivalence_checks=receive_count,
        receive_equivalence_failures=receive_failures, receive_outcomes=outcome_counts,
        ros_samples_included=plan['ros_samples'], host_object_sizes=sizes,
        status_counts={s: sum(1 for r in plan['rows'] if parse_reference(r['payload'])[0] == s) for s in STATUS},
        passed=(run.returncode == 0 and not mismatches and not production_mismatch and
                not production_stream_mismatch and not manual_failures and not missing_manual and
                not receive_failures and plan['ros_samples'] and sizes is not None),
        json_command_representation_ported=True, contract_state_machine_ported=False,
        ros_or_runtime_connected=False, wcet_measured=False, mcu_toolchain_verified=False,
        firmware_verified=False, ready_to_drive=False)
    return report, stdin, run.stdout, run.stderr


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    hashes = {p: digest(ROOT/p) for p in FILES}
    report, stdin, stdout, stderr = evaluate(args.binary.resolve(), build_plan())
    report.update(source_sha256=hashes, binary_sha256=digest(args.binary),
                  source_unchanged=all(digest(ROOT/p) == v for p, v in hashes.items()))
    for name, value in [('stdin.txt', stdin), ('stdout.txt', stdout), ('stderr.txt', stderr)]:
        (args.output/name).write_text(value, encoding='utf-8', newline='\n')
    with (args.output/'report.json').open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in FILES:
            archive.write(ROOT/path, path)
    summary = {k: v for k, v in report.items() if not isinstance(v, (list, dict)) or k in ('status_counts',
                                                                                          'receive_outcomes')}
    summary.update({k + '_count': len(report[k]) for k in ('representation_mismatches', 'production_decoder_mismatches',
                                                           'manual_failures', 'receive_equivalence_failures')})
    print(json.dumps({k: v for k, v in summary.items() if k != 'source_sha256'}))
    return 0 if report['passed'] and report['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
