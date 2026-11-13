"""실장 입력 전달의 합성 회귀. 테스트 값은 운영 교정값이 아니며 장치를 실행하지 않는다."""
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))
import runtime_installed_profile as subject
from arena_vehicle_interface.steering_calibration import record_zero, complete_mount_profile, cpp_zero_profile


def synthetic_inputs(tmp_path, direction=1):
    # 합성 fixture 전용. 실장 프로필 예제/자동 기본값으로 배포하지 않는다.
    board = dict(schema='runtime_installed_board_v1', assembly_id='synthetic-not-installed',
        encoder_direction=-1, motor_direction_confirmed=True, forward_dir_high=True,
        maximum_duty=.4, kp=.6, ki=1.2, maximum_speed_mps=2., feedback_period_us=20000,
        servo=dict(id=1, goal_speed=120, goal_acceleration=10, position_period_us=100000,
            observation_max_age_us=200000, command_limit=.4, wheelbase=.128, track=.1239,
            wheel_limit=.5235987755982988))
    zero = complete_mount_profile(record_zero(2000, assembly_id=board['assembly_id'],
        straight_confirmed=True, evidence_note='SYNTHETIC TEST ONLY'), direction=direction,
        minimum=1000, maximum=3000, confirmed=True)
    bp, zp = tmp_path/'board.json', tmp_path/'zero.json'
    bp.write_text(json.dumps(board), encoding='utf-8')
    zp.write_text(json.dumps(zero), encoding='utf-8')
    return board, zero, bp, zp


def prepared(tmp_path, direction=1):
    board, zero, bp, zp = synthetic_inputs(tmp_path, direction)
    folder = tmp_path/'prepared'
    result = subject.prepare(bp, zp, board['assembly_id'], folder)
    return board, zero, folder, result


def test_preserved_inputs_existing_zero_export_and_no_overwrite(tmp_path):
    board, zero, folder, result = prepared(tmp_path)
    assert subject.verify(folder) == result
    expected, digest = cpp_zero_profile(zero, assembly_id=board['assembly_id'])
    assert (folder/'mounted_steering_zero.hpp').read_bytes() == expected.encode('ascii')
    assert result['canonical_zero_sha256'] == digest
    assert (folder/'runtime_board.json').read_bytes() == (tmp_path/'board.json').read_bytes()
    assert (folder/'steering_zero.json').read_bytes() == (tmp_path/'zero.json').read_bytes()
    assert result['output_enabled'] is result['firmware_uploaded'] is result['cpp_valid_checked'] is False
    with pytest.raises(FileExistsError):
        subject.prepare(tmp_path/'board.json', tmp_path/'zero.json', board['assembly_id'], folder)


@pytest.mark.parametrize('fault', ['missing', 'nan', 'bool-number', 'missing-servo', 'wrong-board-assembly',
                                  'wrong-zero-assembly', 'pending-zero'])
def test_bad_explicit_inputs_rejected_without_output(tmp_path, fault):
    board, zero, bp, zp = synthetic_inputs(tmp_path)
    if fault == 'missing': del board['kp']
    if fault == 'nan': board['kp'] = float('nan')
    if fault == 'bool-number': board['kp'] = True
    if fault == 'missing-servo': del board['servo']['track']
    if fault == 'wrong-board-assembly': board['assembly_id'] = 'different'
    if fault == 'wrong-zero-assembly': zero['assembly_id'] = 'different'
    if fault == 'pending-zero': zero['status'] = 'zero_recorded_pending_mount_checks'
    bp.write_text(json.dumps(board), encoding='utf-8'); zp.write_text(json.dumps(zero), encoding='utf-8')
    with pytest.raises(ValueError):
        subject.prepare(bp, zp, 'synthetic-not-installed', tmp_path/'no-output')
    assert not (tmp_path/'no-output').exists()


@pytest.mark.parametrize('name', ['runtime_board.json', 'mounted_steering_zero.hpp', 'runtime_installed_profile.hpp'])
def test_tampered_bundle_rejected(tmp_path, name):
    _, _, folder, result = prepared(tmp_path)
    with (folder/name).open('ab') as stream: stream.write(b' ')
    with pytest.raises(ValueError, match='changed'):
        subject.verify(folder)
    if name.endswith('.hpp'):
        # 해시만 함께 고쳐도 원 JSON에서 생성된 header와 달라 수락하지 않는다.
        result['files'][name] = subject.digest((folder/name).read_bytes())
        (folder/'manifest.json').write_text(json.dumps(result), encoding='utf-8')
        with pytest.raises(ValueError, match='differs'):
            subject.verify(folder)


def test_duplicate_keys_and_cli_do_not_start_device_code(tmp_path):
    path = tmp_path/'duplicate.json'
    path.write_text('{"schema":1,"schema":2}', encoding='utf-8')
    with pytest.raises(ValueError, match='duplicate'): subject.read_json(path)
    board, _, bp, zp = synthetic_inputs(tmp_path)
    result = subprocess.run([sys.executable, str(ROOT/'scripts/runtime_installed_profile.py'), 'prepare',
        '--board-profile', str(bp), '--steering-profile', str(zp), '--assembly-id', board['assembly_id'],
        '--output', str(tmp_path/'cli')], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)['firmware_uploaded'] is False


def compile_run(tmp_path, body, profile=None, enabled=True):
    compiler = shutil.which('g++')
    assert compiler, 'native g++ is required, no skip/pass substitution'
    source = tmp_path/'check.cpp'; binary = tmp_path/'check'
    source.write_text('#include "board_profile.hpp"\n#include "actuation_motor_endpoint.hpp"\n'
        '#include <cassert>\nint main(){\nconst arena_steering::CadMapping map(arena_steering::kCadSamples,'
        'sizeof(arena_steering::kCadSamples)/sizeof(arena_steering::kCadSamples[0]));\n'
        'auto p=installed_board_profile();\n'+body+'\n}\n', encoding='ascii')
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic', '-ffp-contract=off',
        '-I'+str(ROOT/'src/arena_vehicle_interface/portable'),
        '-I'+str(ROOT/'src/arena_vehicle_interface/firmware_runtime_drive/main')]
    if profile:
        flags += ['-I'+str(profile)]
        if enabled: flags += ['-DARENA_RUNTIME_INSTALLED_PROFILE=1']
    result = subprocess.run([compiler, *flags, str(source), '-o', str(binary)], capture_output=True, text=True, timeout=90)
    assert result.returncode == 0, result.stderr
    result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, (result.stdout, result.stderr)


BOOT_INACTIVE = '''
const std::uint32_t boot[]={1}; arena_actuation::TxConfig tc;
arena_actuation::ActuationMotorEndpoint endpoint(p.endpoint_receiver_config(map),boot,1,30000,
  arena_actuation::PayloadLayout::kEnvelope,tc,p.kp,p.ki,.05,0.,p.maximum_duty);
endpoint.connect(0); assert(!endpoint.receiver().armed());
arena_servo::RuntimeServo servo(map,p.servo); assert(!servo.engaged());
'''


def test_default_profile_inactive_even_when_header_is_on_include_path(tmp_path):
    _, _, folder, _ = prepared(tmp_path)
    # 단순 파일 존재로 활성화하지 않는다. macro를 빼고 include 경로만 넣는다.
    compile_run(tmp_path, 'assert(!p.valid(map)); assert(p.encoder_direction==0);\n'+BOOT_INACTIVE,
                folder, enabled=False)


@pytest.mark.parametrize('direction', [-1, 1])
def test_every_field_exact_and_existing_valid_is_authority(tmp_path, direction):
    board, zero, folder, _ = prepared(tmp_path, direction)
    expected = []
    for prefix, values, types in (('p.', board, subject.BOARD_TYPES), ('p.servo.', board['servo'], subject.SERVO_TYPES)):
        expected += ['assert('+prefix+key+'=='+subject.literal(values[key], kind)+');' for key, kind in types.items()]
    expected += ['assert(p.servo.zero.zero==2000 && p.servo.zero.direction=='+str(direction)+');',
                 'assert(p.servo.zero.minimum==1000 && p.servo.zero.maximum==3000);',
                 'assert(p.valid(map));', BOOT_INACTIVE,
                 'p.kp=0; assert(!p.valid(map)); p.kp=.6;',
                 'p.motor_direction_confirmed=false; assert(!p.valid(map));']
    compile_run(tmp_path, '\n'.join(expected), folder)


def test_builder_drive_only_option_and_shell_syntax(tmp_path):
    for name in ('build_runtime_drive.sh', 'build_runtime_preview.sh'):
        result = subprocess.run(['bash', '-n', str(ROOT/'scripts'/name)], capture_output=True, text=True, timeout=10)
        assert result.returncode == 0, result.stderr
    result = subprocess.run(['bash', str(ROOT/'scripts/build_runtime_preview.sh'), str(tmp_path/'build'),
        '/unused-st', str(tmp_path/'evidence'), 'firmware_runtime_preview', '--installed-profile', '/unused'],
        capture_output=True, text=True, timeout=10)
    assert result.returncode == 2 and 'drive-only' in result.stderr
    assert not (tmp_path/'build').exists() and not (tmp_path/'evidence').exists()
