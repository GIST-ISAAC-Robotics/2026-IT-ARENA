"""run32의 주행 창 안 판단 전이만 읽는다. 제어 재실행·판정 변경 없이 원시 해시를 함께 남긴다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys


def compact(data):
    speed, steering, detail = data['output']
    lateral = detail.get('m3_lateral') or {}
    checks = {}
    for name, check in (lateral.get('checks') or {}).items():
        observation = check.get('observation') or {}
        command = check.get('command') or {}
        checks[name] = {
            'eligible': check.get('eligible_for_trial'), 'reasons': check.get('reasons'),
            'observation_supported': observation.get('supported'),
            'command_accepted': command.get('accepted'), 'command_reason': command.get('reason'),
        }
    return {
        't': data['input']['reference_s'], 'speed': speed, 'steering': steering,
        'reason': detail.get('reason'), 'mode': lateral.get('mode'),
        'threat_sides': [side for side, threat in (lateral.get('threats') or {}).items() if threat.get('present')],
        'release_reason': (lateral.get('release_gate') or {}).get('reason'),
        'released': bool(lateral.get('released_to_baseline')),
        'all_rejected': bool(detail.get('m3_all_commands_rejected')),
        'small_center': lateral.get('small_center'), 'state': lateral.get('state'),
        'checks': checks,
    }


def groups(rows, predicate, max_gap=.25):
    result, current = [], []
    for row in rows:
        if not predicate(row) or (current and row['t']-current[-1]['t'] > max_gap):
            if current:
                result.append(current)
                current = []
        if predicate(row):
            current.append(row)
    if current:
        result.append(current)
    return result


def analyze(run):
    report = json.loads((run/'report.json').read_text(encoding='utf8'))
    trace = json.loads((run/'trace.json').read_text(encoding='utf8'))
    green = report['evaluation']['t_green']
    end = green+40.
    rows = []
    for line in (run/'follow_decisions.jsonl').open(encoding='utf8'):
        event = json.loads(line)
        if event['kind'] == 'call':
            row = compact(event['data'])
            if green <= row['t'] <= end:
                rows.append(row)
    late = [r for r in rows if r['t'] >= green+15.]
    stopped = groups([r for r in trace if green+15. <= r['t'] <= end],
                     lambda r: r['ground_speed_mps'] < .05, max_gap=.25)
    stops = []
    for span in stopped:
        if span[-1]['t']-span[0]['t'] < .5:
            continue
        first, last = span[0], span[-1]
        during = [r for r in late if first['t'] <= r['t'] <= last['t']]
        before = [r for r in rows if first['t']-.25 <= r['t'] < first['t']]
        checks = Counter((name, reason) for r in during for name, c in r['checks'].items()
                         for reason in (c.get('reasons') or []))
        stops.append({
            'first_s': first['t'], 'last_s': last['t'], 'duration_between_samples_s': last['t']-first['t'],
            'first_pose': {k: first.get(k) for k in ('x', 'y', 'yaw', 's', 'progress_m')},
            'last_pose': {k: last.get(k) for k in ('x', 'y', 'yaw', 's', 'progress_m')},
            'decision_reasons': dict(Counter(r['reason'] for r in during)),
            'release_reasons': dict(Counter(str(r['release_reason']) for r in during)),
            'threat_calls': sum(bool(r['threat_sides']) for r in during),
            'plan_rejections': [{'candidate': key[0], 'reason': key[1], 'count': count}
                                for key, count in sorted(checks.items())],
            'preceding_decisions': before, 'first_decisions': during[:2], 'last_decisions': during[-2:],
        })
    exceeded = [r for r in late if r['small_center'] and not r['small_center']['ok']]
    threats = groups(rows, lambda r: bool(r['threat_sides']), max_gap=.12)
    return {
        'scope': 'read-only post-run decision transitions; truth pose used only for evaluation; no verdict rewrite',
        'window_s': [green, end], 'late_window_s': [green+15., end],
        'passed_original': report.get('passed'), 'run_valid_original': report.get('run_valid'),
        'decision_count': len(rows), 'late_reasons': dict(Counter(r['reason'] for r in late)),
        'late_modes': dict(Counter(str(r['mode']) for r in late)),
        'release_times_s': [r['t'] for r in rows if r['released']],
        'threat_intervals': [{'first_s': span[0]['t'], 'last_s': span[-1]['t'], 'calls': len(span),
                              'sides': sorted(set(s for r in span for s in r['threat_sides']))} for span in threats],
        'late_small_center_not_ok_count': len(exceeded),
        'first_late_small_center_not_ok': exceeded[:2],
        'late_all_rejected_count': sum(r['all_rejected'] for r in late),
        'stalls_over_half_second': stops,
        'input_sha256': {name: hashlib.sha256((run/name).read_bytes()).hexdigest()
                         for name in ('report.json', 'trace.json', 'follow_decisions.jsonl')},
    }


if __name__ == '__main__':
    run, output = map(Path, sys.argv[1:])
    if output.exists():
        raise FileExistsError(output)
    result = analyze(run)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({k: result[k] for k in ('passed_original', 'run_valid_original', 'decision_count',
                                          'late_reasons', 'release_times_s', 'late_all_rejected_count')},
                     ensure_ascii=False))
    print('stalls_over_half_second', len(result['stalls_over_half_second']))
