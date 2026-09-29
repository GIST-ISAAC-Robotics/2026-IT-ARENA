#!/usr/bin/env python3
"""A2 재생 결과의 처리 비용·실제 입력 전달·주행 명령 유지 여부를 비교한다."""
import argparse
import json
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO/'src/arena_vehicle_interface'))
from arena_vehicle_interface.bag_contract import quantiles


def summarize(path):
    report = json.loads((path/'report.json').read_text())
    original = json.loads((path/'original_audit.json').read_text())
    moving = [r[0] for r in original['commands']['/drive/safe'] if r[1] > .1]
    start, end = min(moving), max(moving)
    commands = [r for r in report['commands']['/drive/safe'] if start <= r[0] <= end]
    durations = {}
    for name in ('local_pursuit/control', 'local_pursuit/image', 'lidar_safety/control'):
        durations[name] = report['timing'].get(name, {}).get('duration_ms')
    # 진단 수집 행과 내부 callback snapshot 수가 일치할 때에만
    # 콜백 단위의 초과 횟수를 전체 완료 수와 직접 대조할 수 있다.
    late = total = 0
    with (path/'timing.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            if row['node'] == 'local_pursuit' and row['callback'] == 'control':
                total += 1
                late += row['duration_ms'] > 20.
    return dict(run=path.name, passed=report['passed'], functional=report['functional_passed'],
        schedule=report['schedule_passed'], eof_stop=report['eof_zero_command'], exits=report['exit_codes'],
        schedule_lateness_sim_ms=report['schedule_lateness_sim_ms'],
        delivery=report['input_callback_delivery'], durations_ms=durations,
        stages_ms=report.get('stages', {}), states=report['states'],
        control_over_20ms=dict(count=late, sampled=total),
        reference_active_window_s=[start/1e9, end/1e9],
        active_window_commands=len(commands),
        active_window_zero_commands=sum(abs(r[1]) < 1e-8 for r in commands),
        active_window_no_zero_commands=bool(commands) and all(abs(r[1]) >= 1e-8 for r in commands),
        active_window_speed_mps=quantiles([r[1] for r in commands]),
        scope='Command response during original active interval, not a closed-loop trajectory or real vehicle speed')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = [summarize(p) for p in args.runs]
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps([dict(run=r['run'], passed=r['passed'],
                          zero=r['active_window_zero_commands'], samples=r['active_window_commands']) for r in result]))
