#!/usr/bin/env python3
"""고정 v8·정해진 오류 조건 순차 실행. 실패 뒤 자동 재튜닝/기준 완화 금지."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'artifacts/validation/2026-09-22/motion_robustness'
CASES = ['clean', 'gyro_plus1', 'gyro_minus1', 'wheels_plus5', 'wheels_minus5',
         'delay15', 'clock_opposed5', 'imu_drop', 'wheels_drop', 'imu_delay_burst', 'combined']


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--batch', required=True)
    parser.add_argument('--cases', nargs='+', choices=CASES, default=CASES)
    args = parser.parse_args()
    if not args.batch.replace('_', '').isalnum():
        raise ValueError('invalid batch name')
    batch = BASE/args.batch
    batch.mkdir(parents=True, exist_ok=False)
    state = {'started_utc': datetime.now(timezone.utc).isoformat(), 'completed': False,
             'phase': 'regression', 'results': []}

    def save():
        tmp = batch/'progress.tmp'
        tmp.write_text(json.dumps(state, ensure_ascii=False, indent=2))
        tmp.replace(batch/'progress.json')
        # 원문은 파일에 보존하고 실행 콘솔에는 진행 요약만 출력한다.
        print(json.dumps({k: v for k, v in state.items() if k != 'results'} |
                         {'finished_cases': [{'case': r['case'], 'accepted': r['accepted']}
                                             for r in state['results']]}, ensure_ascii=False), flush=True)

    def execute(command, label):
        with (batch/f'{label}.log').open('w') as log:
            return subprocess.run([sys.executable, *command], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT).returncode

    save()
    rc = execute(['-m', 'pytest', 'tests', 'src/arena_vehicle_interface/test', '-q',
                  '--junitxml', str(batch/'pytest.xml')], 'pytest')
    state['regression_rc'] = rc
    if rc:
        state.update(completed=True, passed=False, error='regression failed')
        save()
        return 1
    for case in args.cases:
        output = BASE/f'{args.batch}_{case}'
        if output.exists():
            raise FileExistsError(output)
        state.update(phase=case, current_run=str(output))
        save()
        command = ['scripts/validate_basic_autonomy.py', '--autonomy-mode', 'local_pursuit',
                   '--lidar-acquisition', 'sequential', '--lidar-compensation', 'both',
                   '--control-rate-hz', '50', '--render-backend', 'wsl_nvidia',
                   '--collision-detector', 'configured', '--sensor-wall-timeout-s', '30',
                   '--disable-speed-bump', '--red-duration-s', '1', '--speed-profile', 'local_fast',
                   '--laps', '1', '--grid-slot', '3', '--shutdown-mode', 'service',
                   '--max-sim-seconds', '100', '--wall-timeout-s', '1200', '--stop-wall-timeout-s', '120',
                   '--motion-test-profile', str(ROOT/f'config/tests/motion_{case}.json'), '--output', str(output)]
        if case != 'combined':
            command += ['--target-progress-m', '12']
        else:
            command += ['--lidar-test-profile', str(ROOT/'config/tests/lidar_uniform30.json')]
        rc = execute(command, case)
        road_rc = execute(['scripts/audit_local_pursuit_road.py', str(output)], case+'_road')
        summary_rc = execute(['scripts/summarize_motion_robustness.py', str(output)], case+'_summary')
        summary_path = output/'motion_summary.json'
        summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
        accepted = rc == road_rc == summary_rc == 0 and summary.get('accepted', False)
        state['results'].append({'case': case, 'accepted': accepted, 'validator_rc': rc,
                                 'road_rc': road_rc, 'summary_rc': summary_rc, 'summary': summary})
        if not accepted:
            state.update(completed=True, passed=False, error=f'{case}: inspect preserved failure')
            save()
            return 1
        save()
    state.update(completed=True, passed=True, phase='done')
    save()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
