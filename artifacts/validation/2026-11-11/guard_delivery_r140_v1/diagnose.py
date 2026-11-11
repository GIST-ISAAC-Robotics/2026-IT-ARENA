"""완료한 주행에서 GUARD 판단과 명령 나이를 분리한다. 원본은 읽기 전용이다."""
from bisect import bisect_right
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys


def read_rows(path):
    with path.open(encoding='utf8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def stats(values):
    values = sorted(values)
    if not values:
        return {'count': 0}
    return dict(count=len(values), minimum=values[0], p50=values[len(values)//2],
                p95=values[min(len(values)-1, int(.95*len(values)))], maximum=values[-1])


def analyze(run):
    files = ['report.json', 'follow_decisions.jsonl', 'trajectory.jsonl', 'acquisition.json']
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    hashes = {name: sha(run/name) for name in files}
    report = json.loads((run/'report.json').read_bytes())
    events = read_rows(run/'follow_decisions.jsonl')
    assert events[-1]['kind'] == 'footer', 'unfinished trace'
    assert all(row['seq'] == i for i, row in enumerate(events))
    assert all(events[-1]['data'][key] == 0 for key in ('dropped','failed','pending','capture_errors'))
    begin, end = (report['case_window'][key] for key in ('green_sim_s', 'end_sim_s'))
    calls = [row for row in events if row['kind'] == 'call']
    delivered = []
    positive_guard = []
    transitions = []
    last_state = None
    reasons = Counter()
    rechecks = Counter()
    for row in calls:
        data = row['data']; detail = data['output'][2]
        normal = detail.get('normal') or {}
        ref = data['input']['reference_s']
        mode, choice = normal.get('selector_mode'), normal.get('selector_choice')
        state = (mode, tuple((name, tuple(sorted(side.items())))
                             for name, side in sorted((normal.get('selector_sides') or {}).items())))
        if state != last_state:
            # clear 카운터의 매 증가 대신 존재/통과/소실 상태 변경만 별도로 보존한다.
            compact = {name: {key: side.get(key) for key in ('seen','present','passed','vanish')}
                       for name, side in (normal.get('selector_sides') or {}).items()}
            if not transitions or transitions[-1]['sides'] != compact or transitions[-1]['mode'] != mode:
                transitions.append(dict(seq=row['seq'], reference_s=ref, mode=mode, sides=compact,
                                        evidence=normal.get('selector_evidence')))
            last_state = state
        if begin <= ref <= end:
            reasons[f"{mode}/{choice}/{normal.get('selector_reason')}"] += 1
            retry = normal.get('selector_guard_speed_recheck') or {}
            if retry:
                rechecks[str(retry.get('applied'))] += 1
        delivery = data.get('delivery') or {}
        if delivery.get('kind') != 'candidate':
            continue
        first = data['input']['scan']['measurements']['acquired_at_s'][0]
        arrived = first + delivery['source_age_s']
        current = (data.get('current_rebase') or {}).get('output')
        command_ref = current['reference_s'] if current else ref
        target = current['speed_mps'] if current else data['output'][0]
        item = dict(seq=row['seq'], arrived_s=arrived, command_reference_s=command_ref,
                    source_reference_s=ref, mode=mode, choice=choice, target_mps=target,
                    current_rebased=current is not None, remaining_life_s=.2-(arrived-command_ref))
        delivered.append(item)
        if begin <= ref <= end and mode == 'GUARD' and choice == 'guard_baseline' and target > 0:
            positive_guard.append(item)
    assert all(b['arrived_s'] >= a['arrived_s'] for a,b in zip(delivered, delivered[1:]))
    arrivals = [d['arrived_s'] for d in delivered]
    acquired = json.loads((run/'acquisition.json').read_bytes())
    publication = [a['source_stamp_at_publish_s'] for a in acquired]
    assert publication == sorted(publication)
    stale = Counter(); examples = []
    for sample in read_rows(run/'trajectory.jsonl'):
        t = sample['t']
        if not begin <= t <= end or sample.get('safety_reason') != 'stale_input':
            continue
        i,j = bisect_right(arrivals,t)-1,bisect_right(publication,t)-1
        if min(i,j)<0:
            stale['before_first_delivery_or_scan'] += 1
            continue
        item = delivered[i]
        command_age = t-item['command_reference_s']
        scan_age = t-acquired[j]['first_ray_stamp_s']
        category = ('command_and_scan_age' if command_age>.2 and scan_age>.3 else
                    'command_age_only' if command_age>.2 else 'scan_age_only' if scan_age>.3 else
                    'not_explained_by_saved_ages')
        key = f"{category}/{item['mode']}/{item['choice']}/{'positive' if item['target_mps']>0 else 'zero'}"
        stale[key] += 1
        if category == 'command_age_only' and item['choice']=='guard_baseline' and len(examples)<8:
            examples.append(dict(t=t, command_age_s=command_age, latest_published_scan_age_s=scan_age,
                                 drive_speed_mps=sample.get('drive_speed_mps'),
                                 safe_speed_mps=sample.get('safe_speed_mps'), **item))
    return dict(source_sha256=hashes, source_unchanged=all(sha(run/k)==v for k,v in hashes.items()),
                case_window=dict(begin=begin,end=end), selector_reasons=dict(reasons),
                rechecks=dict(rechecks), guard_positive_count=len(positive_guard),
                guard_remaining_command_life_s=stats([d['remaining_life_s'] for d in positive_guard]),
                stale_sample_categories=dict(stale), examples=examples, side_transitions=transitions,
                scope='Saved candidate/publication times, not guard callback receipts. Correlation and remaining command lifetime do not prove the result of changing the controller.')


if __name__ == '__main__':
    result = analyze(Path(sys.argv[1]))
    assert result['source_unchanged']
    with Path(sys.argv[2]).open('x', encoding='utf8') as stream:
        json.dump(result,stream,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k not in ('source_sha256','examples','side_transitions')}))
