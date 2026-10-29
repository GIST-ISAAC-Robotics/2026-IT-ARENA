"""완료된 run2의 후반 정지 원인과 재개에 필요한 작은 표본만 보존한다."""
from collections import Counter
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent


def main():
    source = BASE / 'run2_course/trajectory.jsonl'
    output = BASE / 'run2_hold_audit.json'
    if output.exists():
        raise FileExistsError(output)
    raw = source.read_bytes()
    rows = [json.loads(line) for line in raw.decode('utf-8').splitlines()]
    result = dict(scope='completed trajectory diagnostics; sampled times, no control truth input',
        trajectory_sha256=hashlib.sha256(raw).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    counts = Counter()
    transitions = []
    last_key = None
    first = None
    start = previous = None
    longest = (0., None, None)
    for row in rows:
        if row['progress_m'] < 42.5:
            continue
        info = (row.get('partial_observation_trial') or {}).get('controller') or {}
        if info.get('reason') == 'known_obstacle_latched':
            anchor, clearance = (info.get(name) or {} for name in ('anchor_diagnostic', 'clearance_diagnostic'))
            key = (anchor.get('reason'), clearance.get('reason'))
            counts[str(key)] += 1
            if key != last_key:
                transitions.append(dict(t=row['t'], anchor=anchor, clearance=clearance))
                last_key = key
            if first is None:
                evidence = info.get('latch_evidence') or {}
                first = dict(t=row['t'], progress_m=row['progress_m'],
                    safety_reason=row.get('safety_reason'), latch_time_s=info.get('latch_time_s'),
                    unknown_samples=info.get('unknown_samples'), max_gap_deg=info.get('max_gap_deg'),
                    latch_reason=info.get('latch_reason'), witnesses=evidence.get('witnesses'),
                    surface_correspondence=info.get('surface_correspondence'))
        stopped = row.get('drive_speed_mps') == 0 and abs(row['ground_speed_mps']) < .01
        if stopped:
            if start is None or previous is None or row['t'] - previous > .15:
                start = row['t']
            candidate = (row['t'] - start, start, row['t'])
            if candidate[0] > longest[0]:
                longest = candidate
            previous = row['t']
        else:
            start = previous = None
    result.update(first_latch_sample=first, reobservation_reason_counts=dict(counts),
        reobservation_transitions=transitions,
        longest_sampled_stationary_span=dict(duration_s=longest[0], first_s=longest[1], last_s=longest[2],
            criteria='progress>42.5m; command0; abs(sampled body speed)<0.01m/s; sample gaps<=0.15s'),
        source_unchanged=source.read_bytes() == raw)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(first_latch_time_s=first['latch_time_s'],
        longest_sampled_stationary_span=result['longest_sampled_stationary_span'],
        reobservation_reason_counts=dict(counts), source_unchanged=result['source_unchanged']), ensure_ascii=False))


if __name__ == '__main__':
    main()
