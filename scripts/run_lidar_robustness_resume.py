#!/usr/bin/env python3
"""9/22 미실행 조건만 순차 실행. 기존 산출물을 재사용/삭제하지 않는다."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'artifacts/validation/2026-09-22/lidar_robustness'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--batch-name', required=True)
    parser.add_argument('--remaining-only', action='store_true', help='초기 실행기 오류 후 clean/delay300만 새 폴더에서 재검증')
    args = parser.parse_args()
    if not args.batch_name.replace('_', '').isalnum():
        raise ValueError('batch name must contain only alphanumeric characters and underscores')
    batch = BASE/args.batch_name
    batch.mkdir(parents=True, exist_ok=False)
    state = {'started_utc': datetime.now(timezone.utc).isoformat(), 'completed': False,
             'phase': 'regression', 'results': []}

    def checkpoint():
        temporary = batch/'progress.tmp'
        temporary.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding='utf-8')
        temporary.replace(batch/'progress.json')
        print(json.dumps(state, ensure_ascii=False), flush=True)

    def execute(arguments, log):
        with log.open('w', encoding='utf-8') as stream:
            result = subprocess.run([sys.executable, *arguments], cwd=ROOT,
                                    stdout=stream, stderr=subprocess.STDOUT)
        return result.returncode

    checkpoint()
    rc = execute(['-m', 'pytest', 'tests', 'src/arena_vehicle_interface/test', '-q',
                  '--junitxml', str(batch/'pytest.xml')], batch/'pytest.log')
    state['regression_returncode'] = rc
    if rc:
        state.update(completed=True, passed=False, error='regression failed')
        checkpoint()
        return 1
    cases = [('bias_plus30', 'lidar_bias_plus30.json'), ('bias_minus30', 'lidar_bias_minus30.json'),
             ('delay80', 'lidar_delay80.json'), ('clean', None), ('delay300_stop', 'lidar_delay300_stop.json')]
    if args.remaining_only:
        cases = cases[-2:]
    for label, profile in cases:
        run = BASE/f'{args.batch_name}_{label}'
        if run.exists():
            raise FileExistsError(run)
        state.update(phase=label, current_run=str(run))
        checkpoint()
        command = ['scripts/validate_basic_autonomy.py', '--autonomy-mode', 'local_pursuit',
                   '--lidar-acquisition', 'sequential', '--lidar-compensation', 'both',
                   '--control-rate-hz', '50', '--render-backend', 'wsl_nvidia',
                   '--collision-detector', 'configured', '--sensor-wall-timeout-s', '30',
                   '--disable-speed-bump', '--red-duration-s', '1', '--speed-profile', 'local_fast',
                   '--laps', '1', '--grid-slot', '3', '--shutdown-mode', 'service',
                   '--target-progress-m', '12', '--max-sim-seconds', '60',
                   '--wall-timeout-s', '900', '--stop-wall-timeout-s', '120', '--output', str(run)]
        if profile:
            command += ['--lidar-test-profile', str(ROOT/'config/tests'/profile)]
        rc = execute(command, batch/f'{label}.log')
        if not (run/'report.json').exists():
            state.update(completed=True, passed=False, error=f'{label}: missing report', returncode=rc)
            checkpoint()
            return 1
        road_rc = execute(['scripts/audit_local_pursuit_road.py', str(run)], batch/f'{label}_road.log')
        summary_rc = execute(['scripts/summarize_lidar_robustness.py', str(run)], batch/f'{label}_summary.log')
        summary = json.loads((run/'robustness_summary.json').read_text()) if summary_rc == 0 else {}
        road = summary.get('road', {})
        road_ok = (road_rc == 0 and road.get('driving_samples', 0) > 0 and
                   road.get('outside_samples_over_1cm2') == 0 and road.get('stop_outside_samples_over_1cm2') == 0)
        expected_stop = label == 'delay300_stop'
        accepted = bool(road_ok and summary_rc == 0 and
            ((rc == 1 and (summary.get('expected_stale_rejection') or {}).get('protection_passed'))
             if expected_stop else (rc == 0 and summary.get('passed') and road.get('stop_samples', 0) > 0)))
        state['results'].append({'case': label, 'run': str(run), 'accepted_for_case': accepted,
                                 'validator_returncode': rc, 'road_returncode': road_rc,
                                 'summary_returncode': summary_rc, 'expected_startup_refusal': expected_stop})
        if not accepted:
            state.update(completed=True, passed=False, error=f'{label}: inspect preserved evidence')
            checkpoint()
            return 1
        checkpoint()
    state.update(completed=True, passed=True, phase='done')
    checkpoint()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
