#!/usr/bin/env python3
"""명시 실장 값의 빌드 입력 생성/무결성 확인. 장치 I/O, flash, 출력 허가 없음."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))
from arena_vehicle_interface.steering_calibration import cpp_zero_profile, read_profile

BOARD_TYPES = {
    'encoder_direction': 'int', 'motor_direction_confirmed': 'bool', 'forward_dir_high': 'bool',
    'maximum_duty': 'double', 'kp': 'double', 'ki': 'double', 'maximum_speed_mps': 'double',
    'feedback_period_us': 'int64',
}
SERVO_TYPES = {
    'id': 'unsigned', 'goal_speed': 'unsigned', 'goal_acceleration': 'unsigned',
    'position_period_us': 'int64', 'observation_max_age_us': 'int64',
    'command_limit': 'double', 'wheelbase': 'double', 'track': 'double', 'wheel_limit': 'double',
}
PROFILE_FILES = ('runtime_board.json', 'steering_zero.json',
                 'mounted_steering_zero.hpp', 'runtime_installed_profile.hpp')


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def read_json(path):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('duplicate JSON key: '+key)
            result[key] = value
        return result
    return json.loads(Path(path).read_text(encoding='utf-8'), object_pairs_hook=unique,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError('nonfinite JSON: '+value)))


def literal(value, kind):
    # C++ 표현 가능성만 확인한다. PI/동역학/기구 허용선은 기존 RuntimeBoardProfile.valid가 소유한다.
    if kind == 'bool':
        if type(value) is not bool:
            raise ValueError('explicit boolean required')
        return 'true' if value else 'false'
    if kind == 'double':
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError('finite numeric field required')
        return repr(float(value))
    low, high = {'int': (-(2**31), 2**31-1), 'int64': (-(2**63), 2**63-1),
                 'unsigned': (0, 2**32-1)}[kind]
    if type(value) is not int or not low <= value <= high:
        raise ValueError('integer field outside C++ representation')
    if kind == 'int64' and value == -(2**63):
        return '(-9223372036854775807LL-1)'
    return str(value)+('LL' if kind == 'int64' else 'u' if kind == 'unsigned' else '')


def headers(board, zero, assembly_id):
    required = {'schema', 'assembly_id', 'servo', *BOARD_TYPES}
    if type(board) is not dict or not required <= set(board) <= required | {'max_wheel_mps'}:
        raise ValueError('exact runtime_installed_board_v1 fields required')
    if board['schema'] != 'runtime_installed_board_v1':
        raise ValueError('unknown installed board schema')
    if not isinstance(assembly_id, str) or not assembly_id.strip() or board['assembly_id'] != assembly_id:
        raise ValueError('board assembly identity changed')
    if type(board['servo']) is not dict or set(board['servo']) != set(SERVO_TYPES):
        raise ValueError('exact explicit RuntimeServoConfig fields required')
    if 'max_wheel_mps' in board:
        # 기존 receiver의 양수/명령 상한 관계를 생성 전에 확인한다. 새 보호 판정은 아니다.
        literal(board['max_wheel_mps'], 'double')
        literal(board['maximum_speed_mps'], 'double')
        if board['max_wheel_mps'] <= 0 or board['max_wheel_mps'] < board['maximum_speed_mps']:
            raise ValueError('max_wheel_mps must be positive and >= maximum_speed_mps')
    zero_header, zero_digest = cpp_zero_profile(zero, assembly_id=assembly_id,
        **{key: board['servo'][key] for key in ('command_limit', 'wheelbase', 'track', 'wheel_limit')})
    lines = ['#pragma once', '#include "mounted_steering_zero.hpp"',
             '// Explicit installed values; build only, no automatic ARM.',
             'namespace arena_runtime_mount {', 'inline RuntimeBoardProfile Profile() {',
             '  RuntimeBoardProfile p;']
    for prefix, values, types in (('p.', board, BOARD_TYPES), ('p.servo.', board['servo'], SERVO_TYPES)):
        lines.extend('  '+prefix+key+'='+literal(values[key], kind)+';' for key, kind in types.items())
    if 'max_wheel_mps' in board:
        lines.append('  p.max_wheel_mps='+literal(board['max_wheel_mps'], 'double')+';')
    lines.extend(['  p.servo.zero=arena_steering_mount::Profile();', '  return p;', '}',
                  '}  // namespace arena_runtime_mount', ''])
    return zero_header.encode('ascii'), '\n'.join(lines).encode('ascii'), zero_digest


def source_hashes():
    paths = ('scripts/runtime_installed_profile.py',
             'src/arena_vehicle_interface/arena_vehicle_interface/steering_calibration.py',
             'src/arena_vehicle_interface/arena_vehicle_interface/steering_cad_table.py')
    return {name: digest((ROOT/name).read_bytes()) for name in paths}


def prepare(board_path, zero_path, assembly_id, output):
    board_raw, zero_raw = Path(board_path).read_bytes(), Path(zero_path).read_bytes()
    zero_header, board_header, zero_digest = headers(read_json(board_path), read_profile(zero_path), assembly_id)
    # 원본은 실제 생성에 사용한 동일 bytes여야 한다. 생성 중 외부 입력 수정은 거절한다.
    if board_raw != Path(board_path).read_bytes() or zero_raw != Path(zero_path).read_bytes():
        raise ValueError('input changed during profile preparation')
    files = dict(zip(PROFILE_FILES, (board_raw, zero_raw, zero_header, board_header)))
    manifest = dict(schema='runtime_installed_profile_bundle_v1', assembly_id=assembly_id,
                    files={name: digest(raw) for name, raw in files.items()}, sources=source_hashes(),
                    canonical_zero_sha256=zero_digest, output_enabled=False, firmware_uploaded=False,
                    cpp_valid_checked=False)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    for name, raw in files.items():
        (output/name).write_bytes(raw)
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    return manifest


def verify(folder):
    folder = Path(folder)
    manifest = read_json(folder/'manifest.json')
    if (type(manifest) is not dict or manifest.get('schema') != 'runtime_installed_profile_bundle_v1'
            or set(manifest) != {'schema', 'assembly_id', 'files', 'sources', 'canonical_zero_sha256',
                                 'output_enabled', 'firmware_uploaded', 'cpp_valid_checked'}
            or any(manifest[key] is not False for key in ('output_enabled', 'firmware_uploaded', 'cpp_valid_checked'))
            or manifest.get('sources') != source_hashes()
            or type(manifest.get('files')) is not dict or set(manifest['files']) != set(PROFILE_FILES)):
        raise ValueError('profile bundle schema or source mismatch')
    for name, expected in manifest['files'].items():
        if digest((folder/name).read_bytes()) != expected:
            raise ValueError('profile file changed: '+name)
    zero_header, board_header, zero_digest = headers(read_json(folder/'runtime_board.json'),
        read_profile(folder/'steering_zero.json'), manifest.get('assembly_id'))
    if (zero_header != (folder/'mounted_steering_zero.hpp').read_bytes()
            or board_header != (folder/'runtime_installed_profile.hpp').read_bytes()
            or zero_digest != manifest.get('canonical_zero_sha256')):
        raise ValueError('generated profile differs from preserved input')
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    make = commands.add_parser('prepare')
    make.add_argument('--board-profile', type=Path, required=True)
    make.add_argument('--steering-profile', type=Path, required=True)
    make.add_argument('--assembly-id', required=True)
    make.add_argument('--output', type=Path, required=True)
    check = commands.add_parser('verify')
    check.add_argument('--profile-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        value = (prepare(args.board_profile, args.steering_profile, args.assembly_id, args.output)
                 if args.command == 'prepare' else verify(args.profile_dir))
        print(json.dumps(dict(assembly_id=value['assembly_id'], files=value['files'],
                             output_enabled=False, firmware_uploaded=False, cpp_valid_checked=False)))
        return 0
    except (ValueError, OSError, OverflowError) as exc:
        print(json.dumps(dict(error=str(exc), output_enabled=False, firmware_uploaded=False)))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
