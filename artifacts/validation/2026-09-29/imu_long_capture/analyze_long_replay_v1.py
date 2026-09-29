"""이 시험의 입력/소스 보존과 출발 전후 지연을 읽기 전용으로 대조한다."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'scripts'))
from analyze_wheel_delivery import distribution


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    recording = HERE / 'capture_v3/sensor_recording'
    source = {'/imu/data': [], '/wheel_states': []}
    with (recording / 'receipt.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if row['topic'] in source:
                source[row['topic']].append(row['source_ns'])
    results, hashes = [], []
    for name in ('long_capture_v3_a', 'long_capture_v3_b'):
        replay = HERE / 'jetson_results' / name / 'replay'
        report = read(replay / 'report.json')
        assert report['measurement_passed']
        current = report['source_sha256']
        hashes.append(current)
        with zipfile.ZipFile(replay / 'source_snapshot.zip') as archive:
            assert set(archive.namelist()) == set(current)
            assert all(hashlib.sha256(archive.read(p)).hexdigest() == h for p, h in current.items())
        window = report['active_window_continuity']
        start, end = window['reference_start_ns'], window['reference_end_ns']
        with (replay / 'timing.jsonl').open(encoding='utf-8') as stream:
            rows = [json.loads(line) for line in stream]
        phases = {}
        for phase, inside in (
            ('before_active', lambda t: t < start),
            ('active', lambda t: start <= t <= end),
            ('after_active', lambda t: t > end),
        ):
            groups = {}
            for callback in ('control', 'imu', 'wheels', 'image'):
                selected = sorted((r for r in rows if r['node'] == 'local_pursuit'
                    and r['callback'] == callback and inside(r['receive_ros_ns'])),
                    key=lambda r: r['begin_monotonic_ns'])
                groups[callback] = dict(count=len(selected),
                    source_age_ms=distribution([r['source_age_ms'] for r in selected if r['source_age_ms'] is not None]),
                    callback_duration_ms=distribution([r['duration_ms'] for r in selected]),
                    start_gap_ms=distribution([(b['begin_monotonic_ns']-a['begin_monotonic_ns'])/1e6
                        for a, b in zip(selected, selected[1:])]),
                    source_gap_ms=distribution([(b['source_ns']-a['source_ns'])/1e6
                        for a, b in zip(selected, selected[1:]) if a['source_ns'] is not None and b['source_ns'] is not None]))
            phases[phase] = groups
        publication = {}
        for filename, topic in (('imu_publication.jsonl', '/imu/data'), ('wheel_publication.jsonl', '/wheel_states')):
            with (replay / filename).open(encoding='utf-8') as stream:
                stamps = [json.loads(line)['source_ns'] for line in stream]
            assert stamps == source[topic]
            publication[topic] = dict(count=len(stamps), original_stamp_order_equal=True)
        worst_wheel = max((r for r in rows if r['node'] == 'local_pursuit' and r['callback'] == 'wheels'),
                          key=lambda r: r['source_age_ms'])
        results.append(dict(run=name, reference_window_s=[start/1e9, end/1e9],
            phases=phases, worst_wheel=worst_wheel, publication=publication,
            source_snapshot_hashes_verified=True))
    assert hashes[0] == hashes[1]
    changed_local = [p for p, h in hashes[0].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest() != h]
    result = dict(runs=results, common_source_files=len(hashes[0]), common_source_hashes_equal=True,
        source_different_from_current_workspace=changed_local,
        analysis_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scope='Phase membership uses callback receive ROS time and original positive command window. '
              'Within-phase gaps omit crossing boundaries; not continuous real-time/physical driving proof.')
    with (HERE/'phase_and_provenance_v1.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(dict(common_source_files=result['common_source_files'], local_differences=changed_local,
        runs=[dict(run=r['run'], active=r['phases']['active'], worst_wheel=r['worst_wheel']) for r in results])))


if __name__ == '__main__':
    main()
