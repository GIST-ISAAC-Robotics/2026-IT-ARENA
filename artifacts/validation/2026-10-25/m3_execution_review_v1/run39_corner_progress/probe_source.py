"""run32/run35 마지막 구간의 진행·명령·노면 거절을 분리한다. 원본 변경/재주행 없음."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import numpy as np


def analyze(run):
    names = ['trace.json', 'follow_decisions.jsonl', 'report.json']
    hashes = {name: hashlib.sha256((run/name).read_bytes()).hexdigest() for name in names}
    trace = json.loads((run/'trace.json').read_text())
    report = json.loads((run/'report.json').read_text())
    calls = [e['data'] for e in map(json.loads, (run/'follow_decisions.jsonl').open()) if e['kind'] == 'call']
    rows = []
    for lo, hi in ((18.6, 25.), (25., 30.), (30., 35.), (35., 40.), (40., 43.5)):
        tr = [r for r in trace if lo <= r['t'] <= hi]
        decisions = [r for r in calls if lo <= r['input']['reference_s'] <= hi]
        details = [r['output'][2] for r in decisions]
        checks = [d.get('m3_lateral', {}).get('checks', {}) for d in details]
        values = {
            'window': [lo, hi], 'trace_count': len(tr), 'calls': len(decisions),
            'progress_delta_m': tr[-1]['progress_m']-tr[0]['progress_m'] if tr else None,
            'yaw_delta_rad': float(np.unwrap([r['yaw'] for r in tr])[-1]-tr[0]['yaw']) if tr else None,
            'safe_positive': sum(r['safe_speed_mps'] > 0 for r in tr),
            'safe_reasons': dict(Counter(str(r.get('safety', {}).get('reason')) for r in tr)),
            'measured_speed_minmax': [min(r['input']['speed_mps'] for r in decisions), max(r['input']['speed_mps'] for r in decisions)],
            'ground_speed_minmax': [min(r['ground_speed_mps'] for r in tr), max(r['ground_speed_mps'] for r in tr)],
            'output_reasons': dict(Counter(d['reason'] for d in details)),
            'plan_reasons': {name: dict(Counter(reason for c in checks for reason in c.get(name, {}).get('reasons', [])))
                             for name in ('straight', 'center', 'hold')},
            'selected': dict(Counter(str(d.get('m3_candidate')) for d in details))}
        rows.append(values)
    samples = []
    for reason in ('m3_command_unobserved', 'm3_release_wait', 'm3_execution_trial', 'follow_reverse_motion'):
        candidates = [r for r in calls if r['input']['reference_s'] > 38 and r['output'][2]['reason'] == reason]
        if not candidates:
            continue
        r = candidates[0]
        d, lat = r['output'][2], r['output'][2].get('m3_lateral', {})
        samples.append({'t': r['input']['reference_s'], 'input_speed': r['input']['speed_mps'],
            'previous_steering': r['input']['previous_steering_rad'], 'output': r['output'][:2],
            'reason': reason, 'baseline_reason': d.get('baseline_reason'), 'lateral': lat,
            'envelope': d.get('m3_envelope'), 'presteer': d.get('m3_presteer')})
    return {'windows': rows, 'late_samples': samples, 'stop_checks': report['stop_checks'],
            'source_hash_lines': len((run.parent/'source_sha256_before.txt').read_text().splitlines()),
            'input_hashes': hashes, 'input_unchanged': all(hashlib.sha256((run/n).read_bytes()).hexdigest()==h for n,h in hashes.items())}


if __name__ == '__main__':
    output = Path(sys.argv[1])
    if output.exists():
        raise FileExistsError(output)
    root = Path('artifacts/validation/2026-10-25/m3_execution_review_v1')
    result = {name: analyze(root/name/'run') for name in ('run32_corner_after_threat', 'run35_committed_surface_corner')}
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({k: v['windows'] for k, v in result.items()}, ensure_ascii=False))
