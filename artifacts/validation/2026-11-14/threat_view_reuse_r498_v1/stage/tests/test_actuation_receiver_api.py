"""C++ 코어 생성/복구 응답/API 숫자 경계. 일반 빌드와 시험 빌드를 별도로 확인한다."""
import os
import subprocess

import pytest

CASES = (
    'invalid_feedback_mode', 'invalid_config_constructor', 'null_boot_constructor',
    'empty_boot_constructor', 'long_boot_constructor', 'invalid_boot_codepoint',
    'freshness_invalid_time', 'clear_at_counter_exhaustion', 'valid_motor_arm_drive_stop',
)


@pytest.mark.parametrize('case', CASES)
def test_core_api_boundary(case):
    binary = os.environ.get('ARENA_RECEIVER_API_BINARY')
    if not binary:
        pytest.skip('C++ API checks require explicit ARENA_RECEIVER_API_BINARY')
    result = subprocess.run([binary, case], capture_output=True, text=True, timeout=20)
    assert result.returncode == 0 and not result.stderr, (case, result.stdout, result.stderr)
    assert result.stdout.strip() == case + ':PASS'


def test_normal_build_hides_counter_injection():
    binary = os.environ.get('ARENA_RECEIVER_PRODUCTION_API_BINARY')
    if not binary:
        pytest.skip('normal C++ build check requires explicit ARENA_RECEIVER_PRODUCTION_API_BINARY')
    result = subprocess.run([binary], capture_output=True, text=True, timeout=20)
    assert result.returncode == 0 and not result.stderr, (result.stdout, result.stderr)
