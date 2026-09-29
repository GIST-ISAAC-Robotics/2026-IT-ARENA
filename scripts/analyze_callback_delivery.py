#!/usr/bin/env python3
"""완전한 진단의 IMU 누락을 영상 콜백 점유 구간과 대조한다(인과 확정 아님)."""
import argparse
from collections import Counter
import json
from pathlib import Path


def analyze(recording, replay):
    report = json.loads((replay/'report.json').read_text())
    rows = [json.loads(line) for line in (replay/'timing.jsonl').read_text().splitlines()]
    coverage = report.get('diagnostic_sequence_coverage', {})
    if not coverage.get('local_pursuit/imu', {}).get('complete') or not coverage.get('lidar_safety/imu', {}).get('complete'):
        raise ValueError('Complete diagnostic sequences required')
    source = Counter()
    with (recording/'receipt.jsonl').open() as stream:
        for line in stream:
            item = json.loads(line)
            if item['topic'] == '/imu/data':
                source[item['source_ns']] += 1
    local = Counter(r['source_ns'] for r in rows if r['node'] == 'local_pursuit' and r['callback'] == 'imu')
    missing = source-local
    safety = {r['source_ns']: r for r in rows if r['node'] == 'lidar_safety' and r['callback'] == 'imu'}
    images = [r for r in rows if r['node'] == 'local_pursuit' and r['callback'] == 'image' and r['stages']]
    matched = exact = expanded = 0
    for stamp, count in missing.items():
        other = safety.get(stamp)
        if other is None:
            continue
        matched += count
        point = other['begin_monotonic_ns']
        exact += count*any(r['begin_monotonic_ns'] <= point <= r['end_monotonic_ns'] for r in images)
        expanded += count*any(r['begin_monotonic_ns']-5_000_000 <= point <= r['end_monotonic_ns']+5_000_000 for r in images)
    return dict(run=replay.parent.name, missing=sum(missing.values()), matched_safety_callbacks=matched,
        safety_callback_during_local_image=exact, within_image_plus_minus_5ms=expanded,
        missing_source_s=[s/1e9 for s in sorted(missing)],
        image_processed=len(images), image_over_20ms=sum(r['duration_ms'] > 20 for r in images),
        scope='Same-host monotonic clocks; safety callback time is not packet arrival time. Correlation only, not isolated causation.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('replays', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = [analyze(args.recording, path) for path in args.replays]
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps([{k: v for k, v in r.items() if k != 'missing_source_s'} for r in result]))


if __name__ == '__main__':
    main()
