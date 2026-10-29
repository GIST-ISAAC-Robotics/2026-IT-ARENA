"""완료 run2의 최초 소실 기억과 수치 거절을 분리한다. 원시 결과는 변경하지 않는다."""
from collections import Counter
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


if __name__ == '__main__':
    source = BASE/'run2/follow_decisions.jsonl'
    before = sha(source)
    first = None
    transitions = []
    fits = []
    counts = Counter()
    previous = None
    window = []
    with source.open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if row['kind'] != 'call':
                continue
            data = row['data']; t = data['input']['reference_s']
            detail = data['output'][2]
            partial = detail.get('partial_observation') or {}
            if t < 130.:
                continue
            anchor = partial.get('anchor_diagnostic') or {}
            clearance = partial.get('clearance_diagnostic') or {}
            fit = anchor.get('fit') or {}
            key = (detail.get('reason'), partial.get('reason'), anchor.get('reason'),
                   clearance.get('reason'))
            counts[str(key)] += 1
            if first is None and partial.get('reason') == 'known_obstacle_latched':
                first = dict(seq=row['seq'], t=t, details=detail)
            if key != previous:
                transitions.append(dict(seq=row['seq'], t=t, reasons=key,
                    speed_mps=data['output'][0], anchor=anchor, clearance=clearance))
                previous = key
            if fit:
                fits.append(dict(t=t, speed_mps=data['output'][0], fit=fit,
                    anchor_reason=anchor.get('reason'), matched=anchor.get('matched'),
                    distances=anchor.get('nearest_distances_m'),
                    corrected_points=anchor.get('corrected_points')))
            if 133.8 <= t <= 135.5:
                window.append(dict(seq=row['seq'], t=t, output=data['output']))
    result = dict(scope='recorded planner first-latch/fit diagnostics; no new control or original verdict change',
        input_sha256=before, source_sha256=sha(Path(__file__)), first_latch=first,
        transitions=transitions, reason_counts=dict(counts), fits=fits, onset_window=window,
        input_unchanged=sha(source) == before)
    with (BASE/'run2_decision_diagnosis.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(dict(first_latch_time=first['t'] if first else None,
        fits=[dict(t=r['t'],reason=r['fit'].get('reason'),anchor_reason=r['anchor_reason'],
            small=r['fit'].get('small_alignment'),matched=r['matched'],distances=r['distances']) for r in fits],
        transitions=[dict(t=r['t'],reasons=r['reasons']) for r in transitions],
        input_unchanged=result['input_unchanged']), ensure_ascii=True))
