#!/usr/bin/env python3
"""격리 ROS 합성 입력으로 MCAP 정리 오류를 확인한다. 제어/실물/Gazebo 없음."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil

from validate_sensor_probe import trial

ROOT = Path(__file__).resolve().parents[1]
SOURCES = (
    'scripts/record_sensor_bag.py', 'scripts/recording_guard.py',
    'scripts/validate_sensor_probe.py', 'scripts/validate_sensor_probe_writer.py',
    'scripts/audit_sensor_probe.py', 'scripts/audit_sensor_bag.py',
    'scripts/sensor_recording_profile.py', 'scripts/sensor_probe_delivery.py',
    'scripts/rosbag_writer_compat.py', 'tests/fixtures/sensor_probe_fault_recorder.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/bag_contract.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/drive_feedback.py',
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts') or output == ROOT/'artifacts':
        raise ValueError('new artifact directory required')
    output.mkdir(parents=True, exist_ok=False)
    before = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in SOURCES}
    for p in SOURCES:
        target = output/'sources'/p
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open('xb') as dest, (ROOT/p).open('rb') as source:
            shutil.copyfileobj(source, dest)
    rows = []
    for case in ('normal', 'close', 'write_and_close'):
        events = output/(case+'_events.json')
        os.environ.update(ARENA_WRITER_FAULT=case, ARENA_WRITER_EVENTS=str(events))
        expected = None if case == 'normal' else 'injected_close_failure' if case == 'close' else 'injected_write_failure'
        result = trial(output/case, case, recorder_entrypoint=ROOT/'tests/fixtures/sensor_probe_fault_recorder.py',
                       expected_recorder_error=expected)
        observed = json.loads(events.read_text())
        manifest = json.loads((output/case/'sensor_recording/manifest.json').read_text())
        checks = dict(result['checks'])
        checks.update(close_called_once=len(observed['close_calls']) == 1,
                      no_uncaught_thread_error=not observed['uncaught_thread_errors'])
        if case != 'normal':
            checks['close_error_preserved'] = any('injected_close_failure' in e for e in manifest['error'])
        rows.append(dict(case=case, passed=all(checks.values()), checks=checks,
                         error=manifest['error'], events=observed, exit_code=result['exit_code']))
    after = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in SOURCES}
    report = dict(passed=all(row['passed'] for row in rows) and before == after,
                  cases=rows, source_unchanged=before == after, source_sha256=before,
                  injection='actual MCAP writer wrapped with exceptions after successful physical close',
                  physical_disk_failure_reproduced=False, hardware_verified=False, ready_to_drive=False)
    with (output/'report.json').open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps({k:v for k,v in report.items() if k != 'source_sha256'}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
