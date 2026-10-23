#!/usr/bin/env python3
"""C++ 명령 parser의 독립 Python 기준. CPython json(실제 Decoder와 같은 설정)을 정답으로 쓴다.

- reference_line: payload 하나의 기대 표현(C++ driver 출력 형식). C++ 구현을 읽지 않는다.
- reconstruct: C++ 출력 줄에서 receive가 볼 dict를 다시 만든다. 이 dict와 원래 JSON을 실제
  ActuationReceiver에 넣어 결과·상태가 같으면 표현이 receive에 필요한 정보를 잃지 않은 것이다.
상태기계 포팅 검증이 아니다. 런타임/ROS에 연결하지 않는다.
"""
import json
import struct
import zlib

from arena_vehicle_interface.actuation_contract import integer, number
from arena_vehicle_interface.actuation_wire import Decoder, MAGIC, MAX_PAYLOAD

FIELDS = ('v', 'kind', 'boot', 'generation', 'lease', 'client', 'seq',
          'speed_mps', 'steering_rad', 'source_age_us', 'ttl_us')
KINDS = {'STOP': 2, 'ARM': 3, 'CLEAR': 4, 'DISARM': 5, 'DRIVE': 6}
CLASSES = ('not_delivered', 'envelope_ignored', 'invalid_message', 'stop', 'schema',
           'arm', 'clear', 'disarm', 'drive')
STATUS = ('object', 'invalid_size', 'invalid_utf8', 'syntax', 'nonfinite_constant', 'duplicate_key', 'not_object')
INT64 = (-2**63, 2**63 - 1)
MAX_STORED = 64
DEFAULT_LIMITS = (2.5, .37, 100_000)  # ContractConfig 기본 max_speed/max_steering/lease와 같다.


class _Duplicate(ValueError):
    pass


class _Constant(ValueError):
    pass


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise _Duplicate(key)
        result[key] = value
    return result


def _constant(value):
    raise _Constant(value)


def frame(payload):
    body = struct.pack('<H', len(payload)) + payload
    return MAGIC + body + struct.pack('<I', zlib.crc32(body))


def parse_reference(payload):
    """(status, obj). 실제 Decoder와 같은 json.loads 설정에 오류 종류만 구분한다."""
    if not 1 <= len(payload) <= MAX_PAYLOAD:
        return 'invalid_size', None
    try:
        text = payload.decode('utf-8')
    except UnicodeDecodeError:
        return 'invalid_utf8', None
    try:
        obj = json.loads(text, object_pairs_hook=_pairs, parse_constant=_constant)
    except _Duplicate:
        return 'duplicate_key', None
    except _Constant:
        return 'nonfinite_constant', None
    except ValueError:
        return 'syntax', None
    if type(obj) is not dict:
        return 'not_object', None
    return 'object', obj


def production_decode(payload):
    """실제 운영 Decoder 결과(전달된 dict 또는 None)."""
    if not 1 <= len(payload) <= MAX_PAYLOAD:
        return None
    messages = Decoder().feed(frame(payload))
    return messages[0] if messages else None


def bits(value):
    return struct.pack('>d', value).hex()


def value_text(present, value):
    if not present:
        return 'A'
    if value is None:
        return 'N'
    if type(value) is bool:
        return 'T' if value else 'F'
    if type(value) is int:
        return f'I{value}' if INT64[0] <= value <= INT64[1] else ('B-' if value < 0 else 'B+')
    if type(value) is float:
        return 'D' + bits(value)
    if type(value) is str:
        return f'S{len(value)}'
    if type(value) is list:
        return 'R'
    if type(value) is dict:
        return 'O'
    raise TypeError(type(value))


def stored_text(value):
    if type(value) is not str:
        return '-'
    if len(value) > MAX_STORED:
        return f'{len(value)}:*'
    return f'{len(value)}:' + ''.join(f'{ord(ch):06x}' for ch in value)


def command_class(command, envelope_ok=True):
    """receive의 상태 비의존 내용 분기(_check 이후 순서와 같음)."""
    if not envelope_ok:
        return 'envelope_ignored'
    if type(command.get('v')) is not int or command['v'] != 1:
        return 'invalid_message'
    kind = command.get('kind')
    if kind == 'STOP' and set(command) == {'v', 'kind'}:
        return 'stop'
    common = {'v', 'kind', 'boot', 'generation', 'lease', 'client', 'seq'}
    expected = common | ({'speed_mps', 'steering_rad', 'source_age_us', 'ttl_us'} if kind == 'DRIVE' else set())
    if kind not in ('ARM', 'CLEAR', 'DISARM', 'DRIVE') or set(command) != expected:
        return 'schema'
    return kind.lower()


def session_shape_ok(command):
    client = command.get('client')
    return (integer(command.get('generation')) and integer(command.get('lease'), 1) and
            type(client) is str and 1 <= len(client) <= 64 and integer(command.get('seq'), 0, 2**32 - 1))


def drive_values_ok(command, limits):
    """receive의 invalid_drive 조건식을 그대로 부정한다(기존 number/integer 사용)."""
    max_speed, max_steering, lease_us = limits
    speed, angle = command.get('speed_mps'), command.get('steering_rad')
    age, ttl = command.get('source_age_us'), command.get('ttl_us')
    return not (not number(speed) or not 0 <= speed <= max_speed or
                not number(angle) or abs(angle) > max_steering or
                not integer(age) or not integer(ttl, 1, lease_us) or age >= ttl)


def reference_line(payload, layout, limits=DEFAULT_LIMITS):
    status, obj = parse_reference(payload)
    if status != 'object':
        return f'P;{STATUS.index(status)}'
    envelope_ok = True
    request = '-'
    type_command = command_object = 0
    if layout == 1:
        type_command = int(obj.get('type') == 'command' and type(obj.get('type')) is str)
        command_object = int(type(obj.get('command')) is dict)
        request = stored_text(obj.get('request')) if 'request' in obj else '-'
        envelope_ok = bool(type_command and command_object)
        command = obj['command'] if command_object else {}
    else:
        command = obj
    present = sum(1 << i for i, name in enumerate(FIELDS) if name in command)
    unknown = int(any(key not in FIELDS for key in command))
    kind = command.get('kind')
    kind_code = KINDS.get(kind, 1) if type(kind) is str else 0
    values = [value_text(name in command, command.get(name)) for name in FIELDS]
    drive = drive_values_ok(command, limits)
    tail = (f"D{bits(float(command['speed_mps']))};D{bits(float(command['steering_rad']))}"
            if drive else '-;-')
    return ';'.join(['P', '0', str(layout), str(type_command), str(command_object), request,
                     str(present), str(unknown), str(kind_code), *values,
                     stored_text(command.get('boot')), stored_text(command.get('client')),
                     str(CLASSES.index(command_class(command, envelope_ok))),
                     str(int(session_shape_ok(command))), str(int(drive)), tail])


def _stored(text):
    if text == '-':
        return None
    length, _, body = text.partition(':')
    if body == '*':
        return 'x' * int(length)  # 65 코드포인트 이상: 길이만 의미가 있다(boot id·client 한도 64 초과).
    return ''.join(chr(int(body[i:i+6], 16)) for i in range(0, len(body), 6))


def _value(text, stored=None, kind_code=None):
    tag = text[0]
    if tag == 'N':
        return None
    if tag in 'TF':
        return tag == 'T'
    if tag == 'I':
        return int(text[1:])
    if tag == 'B':
        return -(2**64) if text[1] == '-' else 2**64  # int64 밖: 모든 범위 검사에서 같은 결론
    if tag == 'D':
        return struct.unpack('>d', bytes.fromhex(text[1:]))[0]
    if tag == 'S':
        if stored is not None:
            return stored
        if kind_code is not None:
            names = {v: k for k, v in KINDS.items()}
            return names.get(kind_code, '\0' * int(text[1:]))
        return 'x' * int(text[1:])
    if tag == 'R':
        return []
    if tag == 'O':
        return {}
    raise ValueError(text)


def reconstruct(line):
    """C++ 출력 → (전달 여부, envelope dict 또는 command dict)."""
    parts = line.split(';')
    if parts[0] != 'P':
        raise ValueError(line[:60])
    if parts[1] != '0':
        return None
    layout = int(parts[2])
    present, unknown, kind_code = int(parts[6]), parts[7] == '1', int(parts[8])
    values = parts[9:20]
    boot, client = _stored(parts[20]), _stored(parts[21])
    command = {}
    for index, (name, text) in enumerate(zip(FIELDS, values)):
        if not present >> index & 1:
            continue
        stored = boot if name == 'boot' else client if name == 'client' else None
        command[name] = _value(text, stored, kind_code if name == 'kind' else None)
    if unknown:
        command['\0unknown'] = 0
    if layout == 0:
        return command
    envelope = {'type': 'command' if parts[3] == '1' else '\0other'}
    if parts[4] == '1':
        envelope['command'] = command
    request = _stored(parts[5])
    if request is not None:
        envelope['request'] = request
    return envelope


def deliver(receiver, layout, message, now, output_ready=True):
    """bare: receive 직접. envelope: 현행 VirtualMCU.on_tx의 무시/ARM 준비 분기 뒤 receive."""
    if message is None:
        return 'not_delivered'
    if layout == 1:
        if message.get('type') != 'command' or type(message.get('command')) is not dict:
            return 'envelope_ignored'
        if message['command'].get('kind') == 'ARM' and not output_ready:
            return [False, 'output_not_ready']
        message = message['command']
    return list(receiver.receive(message, now))
