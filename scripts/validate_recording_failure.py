#!/usr/bin/env python3
"""입력 없는 기록의 시간 제한/미완료 보존과 재생 거절을 검사한다. 차량 실행 없음."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

REPO = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(REPO/'artifacts'):
        raise ValueError('output must be under artifacts')
    output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID='169', ROS_LOCALHOST_ONLY='1', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    recording = output/'sensor_recording'
    with (output/'record.log').open('w') as log:
        record = subprocess.run([sys.executable, str(REPO/'scripts/record_sensor_bag.py'),
            '--output', str(recording), '--max-wall-seconds', '1'], env=env,
            stdout=log, stderr=log, timeout=60)
    manifest = json.loads((recording/'manifest.json').read_text())
    with (output/'replay_rejected.log').open('w') as log:
        replay = subprocess.run([sys.executable, str(REPO/'scripts/replay_sensor_bag.py'),
            str(recording), '--output', str(output/'rejected_replay')], env=env,
            stdout=log, stderr=log, timeout=60)
    checks = {
        'partial_record_exit_nonzero': record.returncode == 1,
        'manifest_explicitly_incomplete': manifest['complete'] is False,
        'wall_limit_reason_retained': any('wall duration limit' in e for e in manifest['error']),
        'mcap_closed_with_metadata': (recording/'bag/metadata.yaml').is_file(),
        'replay_rejected': replay.returncode != 0,
        'replay_reason_retained': 'Incomplete or unsupported recording' in (output/'replay_rejected.log').read_text(),
        'no_controller_logs_created': not list((output/'rejected_replay').glob('*_parameters.yaml')),
    }
    report = dict(passed=all(checks.values()), checks=checks)
    (output/'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
