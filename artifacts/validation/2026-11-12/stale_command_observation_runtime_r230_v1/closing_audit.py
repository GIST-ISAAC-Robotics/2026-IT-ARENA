"""종료 후 원 평가를 변경하지 않는 취득 누계·저속 구간 요약."""
from collections import Counter
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
D = A / 'two_actor_run1'
R = D / 'run1'
read = lambda p: json.loads(p.read_bytes())
report = read(R / 'report.json')
records = read(R / 'acquisition.json')
green, end = (report['case_window'][k] for k in ('green_sim_s', 'end_sim_s'))
trace = [x for x in read(R / 'trace.json') if green <= x['t'] <= end]
start = next(x['t'] for x in trace if x['ground_speed_mps'] > .1)
segments = []
active = None
for a, b in zip(trace, trace[1:]):
    if a['t'] < start:
        continue
    if a['ground_speed_mps'] < .05:
        if active is None:
            active = {'start_s': a['t'], 'end_s': b['t']}
        active['end_s'] = b['t']
    elif active is not None:
        segments.append(active)
        active = None
if active is not None:
    segments.append(active)
safety = [json.loads(x) for x in (D / 'full_safety.jsonl').read_text().splitlines()]
for segment in segments:
    segment['duration_s'] = segment['end_s'] - segment['start_s']
    rows = [x for x in safety if segment['start_s'] <= x['reference_s'] <= segment['end_s']]
    segment['guard_reasons'] = dict(Counter(x['reason'] for x in rows))
    segment['scope'] = 'temporal overlap, not proof of sole cause'
segments.sort(key=lambda x: x['duration_s'], reverse=True)
counters = [r['discarded_source_gaps'] for r in records]
result = dict(
    original_passed=report['passed'],
    source_sha256={str(p.relative_to(A)): hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in (R/'report.json', R/'acquisition.json', R/'trace.json', D/'full_safety.jsonl')},
    acquisition=dict(completed_records=len(records), first_ray_s=records[0]['first_ray_stamp_s'],
        first_publish_s=records[0]['source_stamp_at_publish_s'], first_gap_counter=counters[0],
        last_gap_counter=counters[-1], counter_increases_after_first=sum(b>a for a,b in zip(counters,counters[1:])),
        first_to_last_intervals_s=dict(minimum=min(b['first_ray_stamp_s']-a['first_ray_stamp_s'] for a,b in zip(records,records[1:])),
            maximum=max(b['first_ray_stamp_s']-a['first_ray_stamp_s'] for a,b in zip(records,records[1:]))),
        completed_payload=report['native_acquisition']['completed_payload'],
        limitation='counter was already nonzero at first completed scan; exact missing source and cause not established'),
    longest_low_segments=segments[:5], stop=report['stop'],
    shutdown_clean=report['shutdown_clean'], remaining_test_pids=report['remaining_test_pids'],
    process_exit_codes=report['process_exit_codes'],
    scope='read-only audit; original pass/fail and acquisition failure preserved')
with (A/'closing_audit.json').open('x', encoding='utf8') as f:
    json.dump(result, f, ensure_ascii=False, indent=2)
    f.write('\n')
print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'}, ensure_ascii=False))
