#!/usr/bin/env python3
"""출력 비활성 UART 주기 어댑터의 제한 변이 검사. 원본이 아닌 보존 사본에서 빌드한다."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
TARGET = 'actuation_uart_bench.hpp'
MUTATIONS = {
    # 통신 주기마다 제어 tick(통신이 watchdog을 갱신)
    'tick_every_cycle': ('if (!has_tick_ || now >= next_tick_) control_tick(now, r);', 'control_tick(now, r);'),
    # 늦은 tick 뒤 지난 기한을 이어서 따라잡기
    'catch_up_ticks': ('next_tick_ = has_tick_ && now - next_tick_ < config_.control_period_us ? next_tick_ + config_.control_period_us',
                       'next_tick_ = has_tick_ ? next_tick_ + config_.control_period_us'),
    'io_when_disabled': ('    if (!io_enabled_) return finish(r);\n', '\n'),
    'rx_byte_budget_x4': ('while (r.rx_bytes + r.rx_discarded < config_.max_rx_bytes_per_cycle)',
                          'while (r.rx_bytes + r.rx_discarded < 4 * config_.max_rx_bytes_per_cycle)'),
    'no_rx_time_budget': ('if (!first_read && now - r.start_us >= config_.cycle_budget_us)',
                          'if (false && now - r.start_us >= 0)'),
    'no_first_read_guarantee': ('if (!first_read && now - r.start_us', 'if (now - r.start_us'),
    # 사건 예산을 주기 전체가 아니라 확인 호출마다 다시 채움 / 예산 4배
    'per_call_event_budget': ('    bool error = false;\n    while (left != 0) {',
                              '    left = config_.max_events_per_cycle;\n    bool error = false;\n    while (left != 0) {'),
    'event_budget_x4': ('std::uint32_t events_left = config_.max_events_per_cycle;',
                        'std::uint32_t events_left = 4u * config_.max_events_per_cycle;'),
    # 진입 때 이미 가득 찬 큐를 보지 않음 / 꺼낸 뒤 다시 가득 찬 큐를 보지 않음
    'no_entry_saturation_check': ('    if (io_.pending_events() >= config_.event_queue_capacity) {',
                                  '    if (false) {'),
    'no_exit_saturation_check': ('    if (pending >= config_.event_queue_capacity) {', '    if (false) {'),
    'error_events_ignored': ('if (event != UartIoEvent::kData) { error = true;', 'if (event != UartIoEvent::kData) { error = false;'),
    # 읽은 뒤 사건 확인 생략 / 확인 못 한 조각을 넣음 / 미확인 상태에서 읽기 / 읽기 전 예약분 무시
    'feed_before_event_check': ('      const auto after = check_events(r, events_left);',
                                '      const auto after = EventCheck::kClean;'),
    'unconfirmed_fed': ('if (after != EventCheck::kClean) {', 'if (after == EventCheck::kError) {'),
    'read_when_unconfirmed': ('if (before == EventCheck::kUnconfirmed || events_left < config_.post_read_event_reserve) {',
                              'if (false) {'),
    'no_post_read_reserve': (' || events_left < config_.post_read_event_reserve) {', ') {'),
    # 같은 주기의 두 번째 오류를 다음 주기로 넘기지 않고 잊음
    'deferred_break_dropped': ('    break_pending_ = true;\n    r.break_deferred = true;', '    r.break_deferred = true;'),
    # 빈 큐 응답 뒤 마지막 조회에서 실제로 보인 사건을 Clean으로 처리(이전 판본 동작)
    'clean_after_empty_drain': ('return pending == 0 ? EventCheck::kClean : EventCheck::kUnconfirmed;',
                                'return (left != 0 || pending == 0) ? EventCheck::kClean : EventCheck::kUnconfirmed;'),
    # 마지막 작업 뒤 종료 시각을 다시 읽지 않음
    'end_clock_not_read': ('    if (r.start_us >= 0 && fault_ != UartBenchFault::kClockRegressed) {', '    if (false) {'),
    'stamp_cycle_start': ('endpoint_.feed(buffered, rx_.data(), n)', 'endpoint_.feed(r.start_us, rx_.data(), n)'),
    'partial_write_as_complete': ('      return static_cast<std::size_t>(written);', '      return size;'),
    'write_error_ignored': ('{ write_error = true; return 0; }', '{ return 0; }'),
    'no_clock_regress_check': ('(has_now_ && now < last_now_)', '(false && now < last_now_)'),
    'no_input_discard': ('if (!io_.discard_input())', 'if (false)'),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    compiler = shutil.which('g++')
    if not compiler:
        raise SystemExit('g++ required')
    original = (ROOT / 'src/arena_vehicle_interface/portable' / TARGET).read_text(encoding='utf-8')
    results = []
    for name, mutation in [('control', None), *MUTATIONS.items()]:
        folder = args.output / name
        folder.mkdir()
        shutil.copytree(ROOT / 'src/arena_vehicle_interface/portable', folder / 'portable')
        shutil.copytree(ROOT / 'src/arena_vehicle_interface/firmware_smoke/main', folder / 'smoke')
        shutil.copytree(ROOT / 'src/arena_vehicle_interface/firmware_bench/main', folder / 'bench')
        shutil.copy2(ROOT / 'scripts/actuation_uart_bench_host.cpp', folder / 'driver.cpp')
        if mutation:
            before, after = mutation
            if original.count(before) != 1:
                raise RuntimeError(f'{name}: mutation anchor count is {original.count(before)}')
            (folder / 'portable' / TARGET).write_bytes(original.replace(before, after).encode('utf-8'))
        binary = folder / 'driver'
        compiled = subprocess.run([compiler, '-std=c++17', '-O1', '-Wall', '-Wextra', '-pedantic',
                                   '-ffp-contract=off', '-fno-exceptions', '-fno-rtti',
                                   f'-I{folder / "portable"}', f'-I{folder / "smoke"}', f'-I{folder / "bench"}',
                                   str(folder / 'driver.cpp'), '-o', str(binary)],
                                  capture_output=True, text=True, timeout=120)
        (folder / 'compiler.log').write_text(compiled.stdout + compiled.stderr)
        row = dict(name=name, compile_exit=compiled.returncode, detected=False)
        if compiled.returncode == 0:
            try:
                run = subprocess.run([str(binary)], capture_output=True, text=True, timeout=20)
            except subprocess.TimeoutExpired:
                row['timeout'] = True  # 무한 반복은 사례 판단 차이로 세지 않는다
                results.append(row)
                continue
            (folder / 'run.log').write_text(run.stdout + run.stderr)
            row['run_exit'] = run.returncode
            try:
                stats = json.loads(run.stdout)
                row['report'] = stats
                # 컴파일/충돌/시간 초과가 아니라 공유 사례 자체의 판단 차이만 검출로 센다.
                row['detected'] = (run.returncode == 1 and not run.stderr and stats['uart_checks'] == 33 and
                                   stats['uart_failures'] > 0)
                row['control_ok'] = (run.returncode == 0 and not run.stderr and stats['uart_checks'] == 33 and
                                     stats['uart_failures'] == 0)
            except (ValueError, KeyError):
                row['report_error'] = True
        results.append(row)
    passed = results[0].get('control_ok', False) and all(row['detected'] for row in results[1:])
    summary = dict(control_passed=results[0].get('control_ok', False),
                   detected=sum(row['detected'] for row in results[1:]), total=len(MUTATIONS),
                   passed=passed, hardware_executed=False, results=results)
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({k: v for k, v in summary.items() if k != 'results'}))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
