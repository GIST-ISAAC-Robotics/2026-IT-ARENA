#!/usr/bin/env python3
"""DDS 비교의 실제 설정·입력 신선도·운영 판정. 원본 결과를 덮어쓰지 않는다."""
import argparse
import json
from pathlib import Path
from analyze_wheel_delivery import analyze
from replay_transport import validate_active
from summarize_jetson_trials import trial_summary


def summarize(trial):
    replay = trial/'replay'
    trial_report = json.loads((trial/'trial.json').read_text())
    report = json.loads((replay/'report.json').read_text())
    wheel = analyze(replay)
    profile = report['dds_transport']
    audits = {}
    names = ['sensor_replay', 'sensor_replay_observer', 'local_pursuit', 'lidar_safety']
    if report.get('imu_probe_enabled'):
        names.append('minimal_imu_probe')
    for name in names:
        data = json.loads((replay/f'dds_active_{name}.json').read_text())
        validate_active(data, profile['mode'])
        audits[name] = data
    resource = trial_summary(trial)['resources']
    return dict(run=trial.name, transport=profile, active_participants=audits,
        imu_probe_enabled=report.get('imu_probe_enabled', False),
        imu_probe_result=report.get('imu_probe_result'),
        passed=report['passed'], operational_passed=report['operational_passed'],
        functional_passed=report['functional_passed'], schedule_passed=report['schedule_passed'],
        measurement_passed=report['measurement_passed'],
        continuity=report['active_window_continuity'],
        input_missing={key: value['published']-value['callbacks']
                       for key, value in report['input_callback_delivery'].items()},
        eof_zero=report['eof_zero_command'], exits=report['exit_codes'],
        errors=report['error'], wheel=wheel['nodes'], resources=resource,
        trial_returncode=trial_report['returncode'], trial_errors=trial_report['errors'],
        remaining_pids=trial_report['remaining_pids'],
        control_intervals_ms=report['timing']['local_pursuit/control']['wall_interval_ms'],
        schedule_lateness_ms=report['schedule_lateness_wall_ms'],
        scope='Live participant settings were checked, not packet-level transport events; '
              'short open-loop replay, no real sensor/MCU/driving/thermal validation')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    results = [summarize(path) for path in args.trials]
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(results, stream, indent=2)
    print(json.dumps([dict(run=r['run'], passed=r['passed'], operational=r['operational_passed'],
        zero=r['continuity']['zero_commands'], missing=r['input_missing'],
        wheel_max_ms={name: node['source_age_ms']['maximum'] for name, node in r['wheel'].items()})
        for r in results]))


if __name__ == '__main__':
    main()
