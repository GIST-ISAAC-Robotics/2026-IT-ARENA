"""경로 출력과 독립 보호 출력의 정지 위치를 과거 수신 표본에서 분리한다.

토픽 수신 시차가 있으므로 표본 수/구간을 보고하며 정확한 주기별 인과로 확대하지 않는다.
"""
from collections import Counter
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
INPUTS = {
    'official_original': BASE.parent/'official_traffic_v1/run1_course',
    'temporary_completed': BASE.parent/'traffic_side_scope_v1/run1_course',
    'onroad_left': BASE.parent/'onroad_candidate_v1/drive_left',
}


def audit(run):
    path = run/'trajectory.jsonl'
    before = hashlib.sha256(path.read_bytes()).hexdigest()
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    window = report['case_window']
    rows = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
    rows = [r for r in rows if window['green_sim_s'] <= r['t'] <= window['end_sim_s']]
    segments = []
    last_zero_index = None
    for index, r in enumerate(rows):
        drive, safe = r.get('drive_speed_mps'), r.get('safe_speed_mps')
        if drive is None or safe is None:
            category = 'missing_received_command'
        elif drive <= 1e-6:
            category = 'planner_zero'
        elif safe <= 1e-6:
            category = 'independent_guard_zero'
        else:
            category = 'positive_received_command'
        r['_category'] = category
        if category == 'positive_received_command':
            continue
        reason = r['autonomy'].get('reason') if category == 'planner_zero' else r.get('safety_reason')
        key = [category, reason]
        if (segments and last_zero_index == index-1 and segments[-1]['key'] == key
                and r['t']-segments[-1]['last_s'] <= .35):
            segments[-1]['last_s'] = r['t']
            segments[-1]['samples'] += 1
        else:
            segments.append(dict(key=key, first_s=r['t'], last_s=r['t'], samples=1,
                                 progress_m=r['progress_m']))
        last_zero_index = index
    for row in segments:
        row['sample_span_s'] = row['last_s']-row['first_s']
    fixed = [r for r in rows if 119. <= r['t'] <= 162.5] if run.name == 'run1_course' and 'official_traffic_v1' in str(run) else []
    assert hashlib.sha256(path.read_bytes()).hexdigest() == before
    return dict(input_sha256=before, input_unchanged=True, samples=len(rows),
                command_categories=dict(Counter(r['_category'] for r in rows)),
                safety_reasons=dict(Counter(str(r.get('safety_reason')) for r in rows)),
                longest_zero_segments=sorted(segments, key=lambda r:r['sample_span_s'], reverse=True)[:8],
                original_stall_119_to_162_5=dict(samples=len(fixed),
                    command_categories=dict(Counter(r['_category'] for r in fixed)),
                    safety_reasons=dict(Counter(str(r.get('safety_reason')) for r in fixed))))


if __name__ == '__main__':
    result = {name:audit(run) for name,run in INPUTS.items()}
    with (BASE/'stop_layers_v2.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    for name, data in result.items():
        print(name, data['command_categories'], data['original_stall_119_to_162_5'])
