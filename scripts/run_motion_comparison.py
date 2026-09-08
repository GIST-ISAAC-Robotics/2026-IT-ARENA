#!/usr/bin/env python3
"""동일 제어기·센서 입력 부하에서 순차 LiDAR 보정 네 모드 비교."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys

from validate_lidar_control_lab import CASES, REPO


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cases', nargs='+', choices=CASES, default=list(CASES))
    parser.add_argument('--modes', nargs='+', choices=['snapshot', 'none', 'deskew', 'shift', 'both'],
                        default=['snapshot', 'none', 'deskew', 'shift', 'both'])
    parser.add_argument('--render-backend', default='software', choices=['software', 'system', 'wsl_nvidia'])
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(REPO / 'artifacts'):
        raise ValueError('output must be in artifacts')
    output.mkdir(parents=True, exist_ok=False)
    summary = {'started_at_utc': datetime.now(timezone.utc).isoformat(), 'results': [],
               'cases': args.cases, 'modes': args.modes, 'render_backend': args.render_backend}

    def save():
        (output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')

    save()
    for case in args.cases:
        for mode in args.modes:
            name = case + '_' + mode
            directory = output / name
            print('START ' + name, flush=True)
            with (output / (name + '.log')).open('w', encoding='utf-8') as log:
                run = subprocess.run([sys.executable, str(REPO / 'scripts/validate_lidar_control_lab.py'),
                    '--case', case, '--rate', '10', '--source-rate', '500',
                    '--acquisition', 'snapshot_matched' if mode == 'snapshot' else 'sequential',
                    '--compensation', 'none' if mode == 'snapshot' else mode,
                    '--render-backend', args.render_backend, '--output', str(directory)],
                    cwd=REPO, stdout=log, stderr=subprocess.STDOUT)
            path = directory / 'report.json'
            report = json.loads(path.read_text()) if path.exists() else {}
            summary['results'].append({'name': name, 'return_code': run.returncode,
                                      'report': str(path.relative_to(output)), **report})
            save()
            print(f"DONE {name} pass={report.get('passed')} motion={report.get('motion_verified')} "
                  f"shutdown={report.get('shutdown_clean')}", flush=True)
            if not all(report.get(k) for k in ('measurement_completed', 'acquisition_verified', 'shutdown_clean')):
                print('STOP: 측정/종료 실패. 제어 성능 실패와 구분하십시오.', flush=True)
                return 1
    summary['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
    save()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
