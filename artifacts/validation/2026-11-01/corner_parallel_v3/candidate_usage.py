"""저장된 출력에서 후반부 후보의 실제 호출만 집계한다. 재생/궤적 예측이 아니다."""
from collections import Counter
from hashlib import sha256
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOURCE = HERE / 'run1/follow_decisions.jsonl'


def audit():
    counts = Counter()
    accepted = []
    digest = sha256()
    footer = None
    sequence = -1
    with SOURCE.open('rb') as stream:
        for line in stream:
            digest.update(line)
            event = json.loads(line)
            if footer is not None or event['seq'] != sequence + 1:
                raise ValueError('trace order/footer')
            sequence = event['seq']
            if event['kind'] == 'footer':
                footer = event['data']
            elif event['kind'] == 'call':
                data = event['data']
                speed, steering, detail = data['output']
                normal = detail.get('normal', detail)
                risk = normal.get('selector_released_rear_sweep') or {}
                counts['calls'] += 1
                policy = risk.get('policy', 'unused')
                counts[policy + ':' + str(risk.get('accepted', False))] += 1
                if risk.get('accepted'):
                    accepted.append(dict(reference_s=data['input']['reference_s'],
                                         speed_mps=speed, steering_rad=steering,
                                         policy=policy, reason=normal.get('selector_reason'), risk=risk))
    if (footer is None or footer['written'] != sequence or footer['accepted'] != sequence
            or footer['worker_alive'] is not False
            or any(footer[k] for k in ('failed', 'dropped', 'capture_errors', 'pending'))):
        raise ValueError('incomplete trace')
    return dict(scope='실행 시 기록된 후보 호출. 미관측 공간이나 실차 안전 증명이 아님.',
                input_sha256=digest.hexdigest(), script_sha256=sha256(Path(__file__).read_bytes()).hexdigest(),
                counts=dict(counts), accepted=accepted)


if __name__ == '__main__':
    result = audit()
    with (HERE / 'candidate_usage.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(result['counts'], ensure_ascii=False))
