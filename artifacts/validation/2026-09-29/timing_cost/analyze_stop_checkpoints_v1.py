"""안전 0명령 시점의 마지막 완료 콜백을 복원한다. DDS 도착 시각과 혼동하지 않는다."""
import bisect
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    results = []
    for name in ('buffered_a', 'buffered_b'):
        replay = HERE/'jetson_results'/('timing_cost_v1_'+name)/'replay'
        report = read(replay/'report.json')
        assert report['measurement_passed']
        with (replay/'timing.jsonl').open(encoding='utf-8') as stream:
            rows = [json.loads(line) for line in stream]
        groups = {}
        for row in rows:
            groups.setdefault((row['node'], row['callback']), []).append(row)
        for group in groups.values():
            group.sort(key=lambda row: row['begin_monotonic_ns'])

        def last_completed(node, callback, began):
            group = groups[(node, callback)]
            index = bisect.bisect_right([row['end_monotonic_ns'] for row in group], began)-1
            return group[index] if index >= 0 else None

        def checkpoint(node, stamp):
            matches = [r for r in groups[(node, 'control')] if r['receive_ros_ns'] == stamp]
            assert len(matches) == 1, (node, stamp, len(matches))
            tick = matches[0]
            inputs = {}
            for callback in ('wheels', 'imu'):
                latest = last_completed(node, callback, tick['begin_monotonic_ns'])
                assert latest is not None
                inputs[callback] = dict(source_ns=latest['source_ns'],
                    age_at_control_ms=(stamp-latest['source_ns'])/1e6,
                    callback_end_monotonic_ns=latest['end_monotonic_ns'])
            return dict(control=tick, inputs=inputs)

        window = report['active_window_continuity']
        start, end = window['reference_start_ns'], window['reference_end_ns']
        safe_commands = report['commands']['/drive/safe']
        raw_commands = {r[0]: r for r in report['commands']['/drive']}
        events = []
        for index, command in enumerate(safe_commands):
            stamp, speed, _ = command
            if not start <= stamp <= end or speed != 0:
                continue
            safety = checkpoint('lidar_safety', stamp)
            received_command = last_completed('lidar_safety', 'command', safety['control']['begin_monotonic_ns'])
            assert received_command is not None
            raw_stamp = received_command['source_ns']
            raw = raw_commands[raw_stamp]
            local = checkpoint('local_pursuit', raw_stamp)
            next_nonzero = next((r for r in safe_commands[index+1:] if r[1] > 0), None)
            events.append(dict(safe_command=command, local_input_command=raw,
                safety=safety, local=local,
                next_observed_positive_command=next_nonzero,
                time_until_next_positive_ms=(next_nonzero[0]-stamp)/1e6 if next_nonzero else None))
        motion_stops = [r for r in report['status_records']['/autonomy/status']
                        if r.get('state') == 'MOTION_STOP' and start/1e9 <= r['sim_time_s'] <= end/1e9]
        results.append(dict(run=name, events=events, active_motion_stop_status=motion_stops))
    output = dict(runs=results, analysis_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        caveat='Single executor callback completion reconstruction; exact control/header stamp match. '
               'Ages are ROS-time values, not DDS arrival times. Positive command recovery is not physical braking/restart proof.')
    with (HERE/'stop_evidence_v1.json').open('x', encoding='utf-8') as stream:
        json.dump(output, stream, indent=2)
    print(json.dumps([dict(run=r['run'], events=[dict(stamp_s=e['safe_command'][0]/1e9,
        local_speed=e['local_input_command'][1],
        local_ages={k:v['age_at_control_ms'] for k,v in e['local']['inputs'].items()},
        safety_ages={k:v['age_at_control_ms'] for k,v in e['safety']['inputs'].items()},
        next_positive_ms=e['time_until_next_positive_ms']) for e in r['events']]) for r in results]))


if __name__ == '__main__':
    main()
