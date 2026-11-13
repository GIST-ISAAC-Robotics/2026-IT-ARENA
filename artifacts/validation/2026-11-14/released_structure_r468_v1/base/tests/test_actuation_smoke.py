"""MCU 검사 앱과 같은 합성 사례를 호스트에서만 실행한다(보드 실행으로 세지 않음)."""
import json
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('mode', ['sanitizers', 'size_optimized'])
def test_firmware_smoke_shared_cases_on_host(tmp_path, mode):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('g++ is required for the host-only firmware smoke cases')
    binary = tmp_path / 'smoke_host'
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic', '-fno-exceptions', '-fno-rtti',
             '-ffp-contract=off']
    flags += (['-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer']
              if mode == 'sanitizers' else ['-Os'])
    includes = [ROOT / 'src/arena_vehicle_interface/portable',
                ROOT / 'src/arena_vehicle_interface/firmware_smoke/main']
    build = subprocess.run([compiler, *flags, *(f'-I{path}' for path in includes),
                            str(ROOT / 'scripts/actuation_smoke_host.cpp'), '-o', str(binary)],
                           capture_output=True, text=True, timeout=90)
    assert build.returncode == 0, (build.stdout, build.stderr)
    run = subprocess.run([str(binary)], capture_output=True, text=True, timeout=15)
    assert run.returncode == 0 and not run.stderr, (run.stdout, run.stderr)
    assert json.loads(run.stdout) == {
        'checks': 20, 'failures': 0, 'failure_mask': '0000000000000000', 'hardware_executed': False,
        'ingress_checks': 12, 'ingress_failures': 0, 'ingress_failure_mask': '0000000000000000',
        'tx_checks': 9, 'tx_failures': 0, 'tx_failure_mask': '0000000000000000',
        'bench_checks': 15, 'bench_failures': 0, 'bench_failure_mask': '0000000000000000',
        'pi_checks': 12, 'pi_failures': 0, 'pi_failure_mask': '0000000000000000',
        'cycle_checks': 25, 'cycle_failures': 0, 'cycle_failure_mask': '0000000000000000',
    }
