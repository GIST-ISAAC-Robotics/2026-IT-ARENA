"""저장 M3 판단을 읽기만 하는 실패 분해. 제어 재실행/판정 변경 없음."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.resolve().is_relative_to(args.run.resolve()):
        raise ValueError('new output outside raw run required')
    source = args.run/'follow_decisions.jsonl'
    counters = {k: Counter() for k in ('reason', 'baseline_reason', 'mode', 'selected', 'plan_rejection', 'envelope_rejection')}
    spans, examples = [], {}
    count = positive = selected_positive = 0
    for line in source.open(encoding='utf-8'):
        event = json.loads(line)
        if event['kind'] != 'call':
            continue
        data = event['data']
        speed, steering, detail = data['output']
        lateral = detail.get('m3_lateral', {})
        count += 1
        positive += speed > 0
        selected_positive += speed > 0 and detail.get('m3_selected', False)
        for key, value in (('reason', detail.get('reason')), ('baseline_reason', detail.get('baseline_reason')),
                           ('mode', lateral.get('mode')), ('selected', lateral.get('selected'))):
            counters[key][str(value)] += 1
        checks = lateral.get('checks', {})
        for name, check in checks.items():
            for reason in check.get('reasons', []):
                counters['plan_rejection'][name+':'+reason] += 1
        envelope = detail.get('m3_envelope', {})
        for reason in envelope.get('reasons', []):
            counters['envelope_rejection'][str(reason)] += 1
        t = data['input']['reference_s']
        present = ','.join(k for k,v in lateral.get('threats', {}).items() if v.get('present'))
        key = (detail.get('reason'), lateral.get('mode'), lateral.get('selected'), present)
        if not spans or tuple(spans[-1]['key']) != key:
            spans.append({'key': key, 'first_s': t, 'last_s': t, 'calls': 1, 'positive': int(speed > 0)})
        else:
            spans[-1]['last_s'] = t
            spans[-1]['calls'] += 1
            spans[-1]['positive'] += speed > 0
        if key not in examples:
            examples[key] = {'seq': event['seq'], 't': t, 'speed': speed, 'steering': steering,
                'measured_speed': data['input']['speed_mps'],
                'detail': {k: v for k, v in detail.items() if k not in ('path_points', 'support_points')},
                'source_first_s': data['input']['scan']['sec']+data['input']['scan']['nanosec']*1e-9}
    report = {'scope': 'stored output diagnosis only; not replay, control input, or revised run verdict',
        'input_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'calls': count, 'positive': positive, 'selected_positive': selected_positive,
        'counts': {k: dict(v) for k,v in counters.items()}, 'spans': spans,
        'examples': list(examples.values())}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('spans', 'examples')}, ensure_ascii=False))
    print('spans', len(spans), 'examples', len(examples))


if __name__ == '__main__':
    main()
