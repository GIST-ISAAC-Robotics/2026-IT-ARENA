"""완료 주행의 재관측 해제와 정체를 표본에서 집계한다. 원본 판정은 보존한다."""
from collections import Counter
import argparse
import hashlib
import json
from pathlib import Path


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    source = args.run / 'trajectory.jsonl'
    before = digest(source)
    counts = {side: Counter() for side in ('controller', 'safety')}
    releases, seen, first_latches = [], set(), {}
    start = previous = None
    spans = []
    rows = 0

    def close_span():
        nonlocal start, previous
        if start is not None:
            spans.append(dict(first_s=start[0], last_s=previous[0],
                duration_s=previous[0]-start[0], start_progress_m=start[1],
                end_progress_m=previous[1]))
        start = previous = None

    with source.open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            rows += 1
            t = row['t']
            elapsed = row.get('elapsed_since_green_s')
            if elapsed is None or elapsed < 0:
                continue
            partial = row.get('partial_observation_trial') or {}
            for side in counts:
                info = partial.get(side) or {}
                counts[side][str(info.get('reason'))] += 1
                if info.get('reason') == 'known_obstacle_latched' and side not in first_latches:
                    first_latches[side] = dict(t=t, progress_m=row['progress_m'],
                        latch_time_s=info.get('latch_time_s'),
                        reason=info.get('latch_reason'))
                for field in ('anchor_diagnostic', 'clearance_diagnostic'):
                    detail = info.get(field) or {}
                    if not detail.get('released'):
                        continue
                    key = (side, field, info.get('latch_time_s'),
                           json.dumps(detail.get('threat_set'), sort_keys=True))
                    if key in seen:
                        continue
                    seen.add(key)
                    # 상세는 실제 반환된 필드만 기록하며 없는 확인/해제는 추정하지 않는다.
                    releases.append(dict(side=side, field=field, first_sample_s=t,
                        progress_m=row['progress_m'], safe_speed_mps=row['safe_speed_mps'],
                        detail=detail))
            stopped = row['safe_speed_mps'] == 0 and abs(row['ground_speed_mps']) < .01
            if stopped:
                if previous is not None and t-previous[0] > .15:
                    close_span()
                if start is None:
                    start = (t, row['progress_m'])
                previous = (t, row['progress_m'])
            else:
                close_span()
    close_span()
    result = dict(scope='sampled trajectory diagnostics; no original verdict change',
        input_sha256=before, script_sha256=digest(Path(__file__)), samples=rows,
        reason_samples={side: dict(values) for side, values in counts.items()},
        first_latches=first_latches, observed_releases=releases,
        longest_stationary_spans=sorted(spans, key=lambda x: x['duration_s'], reverse=True)[:10],
        stationary_definition='after green; safe command zero; abs(body speed)<0.01 m/s; gaps<=0.15 s; includes intended lead stop',
        input_unchanged=digest(source) == before)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(dict(samples=rows, releases=len(releases),
        longest_stationary_spans=result['longest_stationary_spans'][:3],
        input_unchanged=result['input_unchanged']), ensure_ascii=False))


if __name__ == '__main__':
    main()
