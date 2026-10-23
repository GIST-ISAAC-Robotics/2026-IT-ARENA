"""수신기/PI의 같은 주기 결합. 실제 HAL/출력 감시/차체 정지 검증이 아니다."""
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
CASES = [
    'normal', 'zero_and_resume', 'software_stop_rearm', 'physical_stop',
    'output_loss_latches', 'default_ready_false', 'command_expiry', 'feedback_timeout',
    'control_gap', 'duplicate_cycle', 'regressed_clock', 'pi_arithmetic',
    'invalid_config', 'arm_drive_batch', 'attach_while_active', 'generation_reset',
    'legacy_rejected', 'poll_does_not_tick', 'existing_fault_preserved', 'invalid_clock',
    'motor_feedback_used', 'normal_deceleration', 'external_tick_cannot_mask_gap',
    'new_arm_batch_resets', 'clear_without_fresh_feedback',
]


@pytest.fixture(scope='module', params=['sanitizers', 'size_optimized'])
def cycle_binary(request, tmp_path_factory):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('g++ required')
    binary = tmp_path_factory.mktemp('motor-cycle') / 'driver'
    includes = [ROOT / 'src/arena_vehicle_interface/portable',
                ROOT / 'src/arena_vehicle_interface/firmware_smoke/main']
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic', '-fno-exceptions',
             '-fno-rtti', '-ffp-contract=off']
    flags += (['-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer']
              if request.param == 'sanitizers' else ['-Os'])
    built = subprocess.run([compiler, *flags, *(f'-I{p}' for p in includes),
                            str(ROOT / 'scripts/motor_cycle_cpp_driver.cpp'), '-o', str(binary)],
                           capture_output=True, text=True, timeout=90)
    assert built.returncode == 0, (built.stdout, built.stderr)
    return binary


@pytest.mark.parametrize('case', CASES)
def test_motor_cycle_boundaries(cycle_binary, case):
    result = subprocess.run([str(cycle_binary), case], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0 and not result.stderr, (result.stdout, result.stderr)
    assert result.stdout.strip() == case + ':PASS'
