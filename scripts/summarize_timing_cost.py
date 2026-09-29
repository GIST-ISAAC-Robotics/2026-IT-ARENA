"""현재 ABBA 비교의 활성 구간·계측 무결성 집계. 파일은 새 출력만 만든다."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

from analyze_wheel_delivery import distribution
from replay_timing import load_export
from summarize_dds_trials import summarize


def age_windows(rows, start, end, window_ns=10_000_000_000):
    """주행 구간을 고정 길이로 나눠 입력 나이가 뒤로 갈수록 쌓이는지 살핀다.

    빈 구간은 0 ms로 채우지 않으며 통계의 단조 증감을 자동 안전 판정으로 쓰지 않는다.
    """
    if window_ns <= 0 or end < start:
        raise ValueError('Invalid active window')
    windows = []
    left = start
    while left <= end:
        right = min(left+window_ns, end+1)
        selected = [row for row in rows if left <= row['receive_ros_ns'] < right]
        summary = {}
        for name in ('local_pursuit', 'lidar_safety'):
            for callback in ('imu', 'wheels'):
                summary[name+'/'+callback] = distribution([
                    row['source_age_ms'] for row in selected if row['node'] == name
                    and row['callback'] == callback and row['source_age_ms'] is not None])
        windows.append(dict(start_s=left/1e9, end_exclusive_s=right/1e9, age_ms=summary))
        left = right
    return windows


def summarize_phases(trial):
    result = summarize(trial)
    replay = trial/'replay'
    report = json.loads((replay/'report.json').read_text(encoding='utf-8'))
    result['timing_mode'] = report['timing_mode']
    with (replay/'timing.jsonl').open(encoding='utf-8') as stream:
        rows = [json.loads(line) for line in stream]
    window = report['active_window_continuity']
    start, end = window['reference_start_ns'], window['reference_end_ns']
    result['reference_window_s'] = [start/1e9, end/1e9]
    result['active_age_windows'] = age_windows(rows, start, end)
    commands = sorted(row[0] for row in report['commands']['/drive/safe'] if start <= row[0] <= end)
    result['active_safe_command_header_gap_ms'] = distribution([(b-a)/1e6 for a,b in zip(commands, commands[1:])])
    result['active'] = {}
    for node in ('local_pursuit', 'lidar_safety'):
        result['active'][node] = {}
        for callback in ('control', 'imu', 'wheels', 'image'):
            selected = sorted((r for r in rows if r['node'] == node and r['callback'] == callback
                and start <= r['receive_ros_ns'] <= end), key=lambda r: r['begin_monotonic_ns'])
            result['active'][node][callback] = dict(count=len(selected),
                age_ms=distribution([r['source_age_ms'] for r in selected if r['source_age_ms'] is not None]),
                duration_ms=distribution([r['duration_ms'] for r in selected]),
                start_gap_ms=distribution([(b['begin_monotonic_ns']-a['begin_monotonic_ns'])/1e6
                    for a, b in zip(selected, selected[1:])]),
                source_gap_ms=distribution([(b['source_ns']-a['source_ns'])/1e6
                    for a, b in zip(selected, selected[1:]) if a['source_ns'] is not None and b['source_ns'] is not None]))
    hashes = report['source_sha256']
    with zipfile.ZipFile(replay/'source_snapshot.zip') as archive:
        assert set(archive.namelist()) == set(hashes)
        assert all(hashlib.sha256(archive.read(p)).hexdigest() == h for p, h in hashes.items())
    result['source_sha256'] = hashes
    result['source_archive_verified'] = True
    if result['timing_mode'] == 'buffered':
        for node, metadata in report['timing_exports'].items():
            # 회수한 파일을 검증하므로 원격 경로 대신 동일 이름의 로컬 경로를 명시한다.
            path = replay/(node+'_timing_buffer.jsonl')
            local_meta = dict(metadata, path=str(path.resolve()))
            prefix = node+'/'
            completed = {k[len(prefix):]: v for k, v in report['completed_callback_counts'].items()
                         if k.startswith(prefix)}
            exported = load_export(dict(node=node, completed=completed, timing_export=local_meta), path, node)
            assert exported == [r for r in rows if r['node'] == node]
        result['export_files_verified'] = True
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials', type=Path, nargs=4)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    runs = [summarize_phases(path) for path in args.trials]
    assert [r['timing_mode'] for r in runs] == ['topic', 'buffered', 'buffered', 'topic']
    assert all(r['source_sha256'] == runs[0]['source_sha256'] for r in runs)
    parameter_sets = []
    # JSON-compatible ROS scalar/list mappings; YAML parsing is analysis-only.
    import yaml
    for trial in args.trials:
        values = {}
        for name in ('local_pursuit', 'lidar_safety'):
            params = yaml.safe_load((trial/'replay'/(name+'_parameters.yaml')).read_text())['/**']['ros__parameters']
            values[name] = {k: v for k,v in params.items()
                            if k not in ('timing_probe_mode', 'timing_probe_buffer_path')}
        parameter_sets.append(values)
    assert all(values == parameter_sets[0] for values in parameter_sets)
    result = dict(runs=runs, identical_source=True,
        identical_parameters_except_timing_mode_and_path=True,
        scope='Four independent open-loop trials; storage/publication changes affect the observer too. '
              'No production default switch, no lossless or hard realtime proof.')
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps([dict(run=r['run'], mode=r['timing_mode'], operational=r['operational_passed'],
        zero=r['continuity']['zero_commands'], missing=r['input_missing'],
        control=r['active']['local_pursuit']['control'], imu=r['active']['local_pursuit']['imu']) for r in runs]))


if __name__ == '__main__':
    main()
