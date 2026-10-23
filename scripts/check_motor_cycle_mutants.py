#!/usr/bin/env python3
"""제어 주기 후보의 제한 변이 검사. 원본이 아닌 보존 사본에서 빌드한다."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
MUTATIONS = {
    'no_output_readiness': ('if (!output_ready)', 'if (false && !output_ready)'),
    'no_receiver_tick': ('receiver_.tick(now);', '(void)now;'),
    'no_fault_latch': ('receiver_.report_control_fault(reason, now);', '(void)reason;'),
    'no_generation_reset': ('if (generation_ != receiver_.generation())',
                            'if (false && generation_ != receiver_.generation())'),
    'fixed_dt': ('const double dt = first ? 0.0 : static_cast<double>(now - last_cycle_) / 1000000.0;',
                 'const double dt = first ? 0.0 : 0.005;'),
    'ignore_feedback': ('receiver_.target_speed(), receiver_.mean_mps(), dt, true',
                        'receiver_.target_speed(), 0.0, dt, true'),
    'no_integral_reset': ('void reset() { pi_.reset(); }', 'void reset() {}'),
    'no_zero_target_reset': ('      pi_.reset();\n      auto out', '      // mutation\n      auto out'),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    compiler = shutil.which('g++')
    if not compiler:
        raise SystemExit('g++ required')
    original = (ROOT / 'src/arena_vehicle_interface/portable/motor_control_candidate.hpp').read_text()
    results = []
    for name, mutation in [('control', None), *MUTATIONS.items()]:
        folder = args.output / name
        folder.mkdir()
        shutil.copytree(ROOT / 'src/arena_vehicle_interface/portable', folder / 'portable')
        shutil.copytree(ROOT / 'src/arena_vehicle_interface/firmware_smoke/main', folder / 'cases')
        shutil.copy2(ROOT / 'scripts/actuation_smoke_host.cpp', folder / 'driver.cpp')
        if mutation:
            before, after = mutation
            if original.count(before) != 1:
                raise RuntimeError(f'{name}: mutation anchor count is {original.count(before)}')
            (folder / 'portable/motor_control_candidate.hpp').write_text(original.replace(before, after))
        binary = folder / 'driver'
        compiled = subprocess.run([compiler, '-std=c++17', '-O1', '-Wall', '-Wextra', '-Werror',
                                   '-pedantic', '-ffp-contract=off', '-fno-exceptions', '-fno-rtti',
                                   f'-I{folder / "portable"}', f'-I{folder / "cases"}',
                                   str(folder / 'driver.cpp'), '-o', str(binary)],
                                  capture_output=True, text=True, timeout=90)
        (folder / 'compiler.log').write_text(compiled.stdout + compiled.stderr)
        row = dict(name=name, compile_exit=compiled.returncode, detected=False)
        if compiled.returncode == 0:
            run = subprocess.run([str(binary)], capture_output=True, text=True, timeout=15)
            (folder / 'run.log').write_text(run.stdout + run.stderr)
            row['run_exit'] = run.returncode
            try:
                stats = json.loads(run.stdout)
                row['report'] = stats
                # 컴파일/충돌/파싱 실패가 아니라 결합 사례 자체의 판단 차이만 검출로 센다.
                unaffected = all(stats[f'{part}failures'] == 0 for part in ['', 'ingress_', 'tx_', 'bench_', 'pi_'])
                row['detected'] = (run.returncode == 1 and not run.stderr and unaffected and
                                   stats['cycle_checks'] == 25 and stats['cycle_failures'] > 0)
                row['control_ok'] = run.returncode == 0 and not run.stderr and unaffected and stats['cycle_failures'] == 0
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
