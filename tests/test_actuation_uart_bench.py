"""출력 비활성 UART 주기 어댑터. 가짜 I/O·합성 시각만 쓰며 실제 UART/보드 실행이 아니다."""
import json
from pathlib import Path
import re
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
PORTABLE = ROOT / 'src/arena_vehicle_interface/portable'
BENCH = ROOT / 'src/arena_vehicle_interface/firmware_bench'
ADAPTER = PORTABLE / 'actuation_uart_bench.hpp'


@pytest.mark.parametrize('mode', ['sanitizers', 'size_optimized'])
def test_uart_bench_shared_cases_on_host(tmp_path, mode):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('g++ is required for the host-only UART bench cases')
    binary = tmp_path / 'uart_bench_host'
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic', '-fno-exceptions', '-fno-rtti',
             '-ffp-contract=off']
    flags += (['-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer']
              if mode == 'sanitizers' else ['-Os'])
    includes = [PORTABLE, ROOT / 'src/arena_vehicle_interface/firmware_smoke/main', BENCH / 'main']
    build = subprocess.run([compiler, *flags, *(f'-I{path}' for path in includes),
                            str(ROOT / 'scripts/actuation_uart_bench_host.cpp'), '-o', str(binary)],
                           capture_output=True, text=True, timeout=120)
    assert build.returncode == 0, (build.stdout, build.stderr)
    run = subprocess.run([str(binary)], capture_output=True, text=True, timeout=30)
    assert run.returncode == 0 and not run.stderr, (run.stdout, run.stderr)
    assert json.loads(run.stdout) == {
        'uart_checks': 33, 'uart_failures': 0, 'uart_failure_mask': '0000000000000000', 'failed': [],
        'hardware_executed': False, 'output_ready': False,
    }


def _code(path):
    # 주석을 제거한 코드만 검사한다(주석의 금지 API 언급은 허용).
    text = path.read_text(encoding='utf-8')
    return re.sub(r'//[^\n]*', '', text)


def test_portable_adapter_has_no_sdk_or_output_path():
    code = _code(ADAPTER)
    assert not re.search(r'#include\s+["<](driver/|freertos/|esp_|hal/|soc/)', code)
    for forbidden in ['set_output_ready', 'update_motor_encoder', 'set_estop', 'receive(', 'offer(',
                      'clear_link_fault', 'MotorControlCandidate', 'SpeedPI']:
        assert forbidden not in code, forbidden
    # 제어 tick은 시간 기한 경로와 시계 고장 기록 경로 두 곳에서만 호출한다(통신 경로에서 호출하지 않음).
    assert code.count('endpoint_.control_tick(') == 2
    assert 'bool enabled = false;' in code and 'bool wiring_confirmed = false;' in code
    assert 'std::uint32_t baud = 0;' in code


def test_target_bench_keeps_external_io_disabled_and_nonblocking():
    sources = {path.name: _code(path) for path in (BENCH / 'main').iterdir() if path.suffix in {'.cpp', '.hpp'}}
    joined = '\n'.join(sources.values())
    for forbidden in [r'\buart_write_bytes\s*\(', r'\bgpio_set_level\s*\(', r'\bgpio_config\s*\(', r'\bledc_',
                      r'\bmcpwm', r'\bpcnt_', r'\besp_rom_gpio', r'MotorControlCandidate', r'esp_restart\s*\(']:
        assert not re.search(forbidden, joined), forbidden
    # 공유 사례 파일은 API 부재를 static_assert로 검사하므로 이름이 등장한다. 실행 코드에는 없어야 한다.
    runtime = sources['main.cpp'] + sources['esp_uart_io.hpp']
    for forbidden in ['set_output_ready', 'update_motor_encoder', 'set_estop']:
        assert forbidden not in runtime, forbidden
    io = sources['esp_uart_io.hpp']
    assert re.search(r'uart_driver_install\(port_, rx_ring_bytes, 0,', io)  # TX ring 0: uart_tx_chars 비차단 전제
    assert re.search(r'uart_read_bytes\([^;]*, 0\);', io)
    assert 'check_uart_plan(plan) != aa::UartPlanStatus::kReady' in io
    for name, text in sources.items():
        if name != 'esp_uart_io.hpp':
            assert 'uart_driver_install' not in text and 'uart_set_pin' not in text, name
    main = sources['main.cpp']
    assert re.search(r'^aa::UartPortPlan arena_bench_protocol_plan\{\};$', main, re.M)
    # 합성 자기 검사 실패 시 계획이 준비돼 있어도 외부 I/O를 시작하지 않는다.
    assert 'self_check_ok = cases.checks == arena_smoke::kUartCases && cases.failures == 0;' in main
    assert 'started = self_check_ok && status == aa::UartPlanStatus::kReady && start_owner(plan)' in main
    assert '"output_ready\\":false' in main and '"ready_to_drive\\":false' in main
    assert 'xTaskDelayUntil' in main and 'last = xTaskGetTickCount();' in main


def test_bench_build_script_is_build_only():
    script = (ROOT / 'scripts/build_actuation_bench.sh').read_text(encoding='utf-8')
    actions = re.findall(r'idf\.py"[^\n]*\b(build|size|flash|monitor|erase[-_]flash|app-flash)\b', script)
    assert actions and set(actions) <= {'build', 'size'}
    assert 'esptool' not in script and '/dev/tty' not in script
    assert 'test ! -e "$task_stage"' in script
    assert 'large_stack_frames.txt' in script  # 큰 정적 프레임 거절(실제 stack 증거 아님)


def test_bench_sdkconfig_is_check_value_only():
    defaults = (BENCH / 'sdkconfig.defaults').read_text(encoding='utf-8').splitlines()
    assert 'CONFIG_IDF_TARGET="esp32s3"' in defaults
    assert not any(line.startswith('CONFIG_ESP_CONSOLE') for line in defaults)  # 콘솔 위치를 바꾸지 않는다
    assert '# CONFIG_COMPILER_CXX_EXCEPTIONS is not set' in defaults


def test_review_defect_repro_passes_on_current_adapter(tmp_path):
    # 최종 검토 결함 7건의 재현기. 이전 판본 실패는 증거 폴더에 보존하고, 현재 헤더에서는 모두 통과해야 한다.
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('g++ is required for the host-only UART bench cases')
    binary = tmp_path / 'uart_bench_repro'
    includes = [PORTABLE, ROOT / 'src/arena_vehicle_interface/firmware_smoke/main', BENCH / 'main']
    build = subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic', '-fno-exceptions',
                            '-fno-rtti', '-ffp-contract=off', '-O1', '-fsanitize=address,undefined',
                            *(f'-I{path}' for path in includes), str(ROOT / 'scripts/uart_bench_defect_repro.cpp'),
                            '-o', str(binary)], capture_output=True, text=True, timeout=120)
    assert build.returncode == 0, (build.stdout, build.stderr)
    run = subprocess.run([str(binary)], capture_output=True, text=True, timeout=30)
    assert run.returncode == 0 and not run.stderr, (run.stdout, run.stderr)
    assert json.loads(run.stdout) == {'repro_checks': 7, 'failed': [], 'repro_failures': 0, 'hardware_executed': False}
