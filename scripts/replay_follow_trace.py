#!/usr/bin/env python3
"""G1 판단 단계 기록을 순서대로 재생. 전체 ROS/보호층/물리 재생이 아니다."""
import argparse
import hashlib
import json
from pathlib import Path

from arena_autonomy.follow_trace import replay_events


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    before = hashlib.sha256(args.input.read_bytes()).hexdigest()
    try:
        with args.input.open(encoding='utf-8') as stream:
            report = replay_events(json.loads(line) for line in stream)
    except (ValueError, KeyError, TypeError) as error:
        report = {'passed': False, 'capture_valid': False, 'error': str(error)}
    report.update(input_sha256=before, input_unchanged=before == hashlib.sha256(args.input.read_bytes()).hexdigest())
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k not in ('outcomes', 'mismatches')}, ensure_ascii=False))
    return 0 if report['passed'] and report['input_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
