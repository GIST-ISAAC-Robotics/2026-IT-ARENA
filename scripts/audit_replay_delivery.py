#!/usr/bin/env python3
"""기록/재생 헤더 시각을 대조한다. 진단 로그가 온전할 때만 콜백 누락 구간을 집계한다."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('replay', type=Path)
    args = parser.parse_args()
    report = json.loads((args.replay/'report.json').read_text())
    original = defaultdict(Counter)
    with (args.recording/'receipt.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            if row['source_ns'] is not None:
                original[row['topic']][row['source_ns']] += 1
    observed = defaultdict(Counter)
    with (args.replay/'timing.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            if row['source_ns'] is not None:
                observed[row['node']+'/'+row['callback']][row['source_ns']] += 1
    topics = {'scan': '/scan', 'imu': '/imu/data', 'wheels': '/wheel_states',
              'image': '/camera/color/image_raw'}
    result = {}
    for name, value in report['input_callback_delivery'].items():
        rows = observed[name]
        # 콜백 내부 완료 횟수와 계측 행이 다르면 유실 위치는 특정하지 않는다.
        if value['callbacks'] != sum(rows.values()):
            result[name] = {'localized': False, 'reason': 'diagnostic rows incomplete'}
            continue
        source = original[topics[name.split('/')[1]]]
        missing, extra = source-rows, rows-source
        bins = Counter()
        for stamp, count in missing.items():
            bins[str(stamp//1_000_000_000)] += count
        result[name] = dict(localized=True, missing=sum(missing.values()), unexpected=sum(extra.values()),
            first_missing_source_s=min(missing)/1e9 if missing else None,
            last_missing_source_s=max(missing)/1e9 if missing else None,
            missing_by_absolute_source_second=dict(sorted(bins.items(), key=lambda v: int(v[0]))))
    output = args.replay/'delivery_audit.json'
    if output.exists():
        raise ValueError('Existing audit is not overwritten')
    output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
