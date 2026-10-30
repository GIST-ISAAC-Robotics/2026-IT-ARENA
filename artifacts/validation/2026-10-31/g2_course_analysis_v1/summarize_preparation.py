"""완료된 주행의 준비/진입 전환과 실제 사용된 유지 정책을 발췌한다. 평가 전용."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def summarize(run):
    report = json.loads((run / 'report.json').read_text(encoding='utf-8'))
    start = report['green_observed_sim_s']
    end = report['case_window']['end_sim_s']
    path = run / 'follow_decisions.jsonl'
    before = sha(path)
    reasons = Counter()
    modes = Counter()
    calls = held = reverse = reverse_positive = 0
    holds = []
    transitions = []
    previous = None
    config = None
    for line in path.open(encoding='utf-8'):
        event = json.loads(line)
        if event['kind'] == 'header':
            config = event['data']['execution_config']
        if event['kind'] != 'call':
            continue
        data = event['data']
        t = data['input']['reference_s']
        if not start <= t <= end:
            continue
        out = data['output']
        detail = out[2]
        prep = detail.get('preparation') or {}
        reserve = detail.get('follow_entry_space') or {}
        persistence = detail.get('persistence') or {}
        bypass = detail.get('bypass') or {}
        entry = detail.get('entry_space') or {}
        calls += 1
        modes.update([detail.get('handoff_state')])
        reasons.update([(prep.get('reason'), prep.get('reference_reason'))])
        if data['input']['speed_mps'] < 0:
            reverse += 1
            reverse_positive += out[0] > 0
        if reserve.get('reason') == 'bounded_preparation_hold_candidate':
            held += 1
            holds.append(dict(time_s=t, age_s=reserve['hold_age_s'], speed_command=out[0],
                displacement_m=reserve['compensated_patch_displacement_m']))
        key = (detail.get('handoff_state'), detail.get('reason'), prep.get('reason'),
            prep.get('reference_reason'), reserve.get('reason'), persistence.get('ready'),
            bypass.get('reason'), entry.get('hold'))
        if key != previous:
            transitions.append(dict(time_s=t, since_green_s=t-start, state=key[0], reason=key[1],
                preparation=key[2], reference_reason=key[3], reserve=key[4], ready=key[5],
                bypass=key[6], preparation_hold=key[7], command=out[:2],
                measured_speed_mps=data['input']['speed_mps'],
                front_xy=persistence.get('front_median_xy'), source_span_s=persistence.get('source_span_s')))
            previous = key
    return dict(run=str(run), source_sha256=sha(Path(__file__)), input_sha256=before,
        input_unchanged=sha(path) == before, green_s=start, end_s=end, execution_config=config,
        active_calls=calls, modes=dict(modes), preparation_reasons=[dict(reason=k[0], reference_reason=k[1], calls=v)
            for k, v in reasons.items()], retained_calls=held, holds=holds,
        reverse_feedback_calls=reverse, positive_during_reverse=reverse_positive,
        transitions=transitions, scope='completed trace diagnostics; no controller inputs or new pass claim')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = summarize(args.run)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps({k: result[k] for k in ('active_calls', 'modes', 'retained_calls',
        'reverse_feedback_calls', 'positive_during_reverse', 'input_unchanged')}))
