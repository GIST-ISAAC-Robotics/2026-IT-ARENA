"""완료한 주행의 실제 재관측 분기/비용과 별도 상대차 실패를 요약한다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import statistics

BASE = Path(__file__).resolve().parent
RUN = BASE.parent/'rotation_stress_v1/run4_course'


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


if __name__ == '__main__':
    paths = [RUN/'follow_decisions.jsonl', RUN/'report.json']
    before = {str(p): sha(p) for p in paths}
    fits, observed = [], []
    reasons = Counter()
    with paths[0].open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if row['kind'] != 'call': continue
            data = row['data']; meta = data['output'][2]
            partial = meta.get('partial_observation') or {}
            anchor = partial.get('anchor_diagnostic') or {}
            fit = anchor.get('fit') or {}
            if anchor:
                reasons[str(anchor.get('reason'))] += 1
            if fit:
                fits.append(dict(t=data['input']['reference_s'], reason=fit.get('reason'),
                    accepted=fit.get('accepted'), translation_m=fit.get('translation_m'),
                    processing_ms=fit.get('processing_ms'), matched=anchor.get('matched'),
                    confirmations=anchor.get('consecutive_fresh_sources'),
                    target_speed_mps=data['output'][0]))
            if anchor.get('matched'):
                observed.append(dict(t=data['input']['reference_s'],
                    partial_reason=partial.get('reason'), command_reason=meta.get('reason'),
                    speed_mps=data['output'][0], nearest_distances_m=anchor.get('nearest_distances_m')))
    report = json.loads(paths[1].read_text(encoding='utf-8'))
    failures = [f'primary.{k}' for k, v in report.get('gates', {}).items() if not v]
    for actor, details in report.get('additional_actors', {}).get('actors', {}).items():
        failures += [f'{actor}.{k}' for k, v in details.get('gates', {}).items() if not v]
    times = [r['processing_ms'] for r in fits if isinstance(r['processing_ms'], (int, float))]
    result = dict(scope='recorded planner diagnostics only; not safety-node timing or total pipeline cost',
        input_sha256=before, input_unchanged=all(sha(p)==before[str(p)] for p in paths),
        recorded_run_passed=report['passed'], recorded_run_valid=report['run_valid'],
        all_actor_failed_gates=failures, anchor_reasons=dict(reasons), fit_calls=len(fits),
        fit_reasons=dict(Counter(str(r['reason']) for r in fits)), fits=fits, matched=observed,
        fit_median_ms=statistics.median(times) if times else None,
        fit_maximum_ms=max(times) if times else None,
        actor_tail_drain=report.get('actor_tail_drain'),
        note='original report unchanged; successful lap does not override the 150 mm comparison failure')
    with (BASE/'run4_audit.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if k not in ('input_sha256','fits')},ensure_ascii=True))
