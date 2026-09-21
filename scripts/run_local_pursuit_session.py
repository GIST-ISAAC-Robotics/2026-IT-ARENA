#!/usr/bin/env python3
"""지역 Pure Pursuit의 정해진 단계만 순차 실행한다. 실패 시 자동 튜닝하지 않는다."""
import argparse
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--label', default='v1')
    parser.add_argument('--wall-budget-s', type=float, default=1800)
    parser.add_argument('--record', action='store_true', help='검증 통과 뒤에만 추가 녹화')
    args = parser.parse_args()
    if not args.label.replace('_', '').isalnum():
        raise ValueError('label must be alphanumeric')
    began = time.monotonic()
    base = ROOT/'artifacts/validation/2026-09-21/local_pursuit'
    label = args.label
    suffix = 2
    while (base/f'smoke_{label}').exists():
        label = f'{args.label}_retry{suffix}'
        suffix += 1
    common = [sys.executable, 'scripts/validate_basic_autonomy.py', '--autonomy-mode', 'local_pursuit',
              '--lidar-acquisition', 'sequential', '--lidar-compensation', 'both', '--control-rate-hz', '50',
              '--render-backend', 'wsl_nvidia', '--collision-detector', 'configured',
              '--shutdown-mode', 'service',
              '--sensor-wall-timeout-s', '30', '--disable-speed-bump', '--red-duration-s', '1.0',
              '--wall-timeout-s', '1200', '--stop-wall-timeout-s', '120']
    stages = [('smoke', ['--speed-profile', 'exploratory', '--target-progress-m', '12', '--max-sim-seconds', '60']),
              ('lap', ['--speed-profile', 'local_fast', '--laps', '1', '--max-sim-seconds', '130'])]
    if args.record:
        stages.append(('video', ['--speed-profile', 'local_fast', '--laps', '1', '--max-sim-seconds', '130', '--video']))
    for name, options in stages:
        remaining = args.wall_budget_s - (time.monotonic() - began)
        if remaining < 120:
            print('시간 예산 부족: 후속 단계 생략', flush=True)
            return 2
        output = base/f'{name}_{label}'
        command = common + options + ['--output', str(output)]
        print(f'RUN {name}: {output}', flush=True)
        process = subprocess.Popen(command, cwd=ROOT)
        try:
            code = process.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            import signal
            process.send_signal(signal.SIGINT)  # 평가기가 자기 자식들을 정리한다.
            process.wait(timeout=45)
            return 2
        if code:
            print(f'STOP {name}: return={code}; 다음 단계 미실행', flush=True)
            return code
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
