"""155초 대표 코스와 상대 양보 감속 사용을 확인한다."""
import argparse
import hashlib
import json
from pathlib import Path

import course_eval


def limited_checks(original, requested, positive):
    checks = dict(original['checks'])
    restart = original['restart']
    checks.update(
        restart_follow_progress_5m=restart.get('post_restart_follow_progress_m', 0.) >= 5.,
        yield_cap_requested=requested > 0,
        positive_yield_command=positive > 0,
    )
    return checks


def evaluate(output):
    original = course_eval.evaluate(output)
    trace = output/'follow_decisions.jsonl'
    before = hashlib.sha256(trace.read_bytes()).hexdigest()
    requested, positive, first, last, reasons = 0, 0, None, None, {}
    with trace.open(encoding='utf-8') as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] != 'call':
                continue
            call = event['data']
            speed, _, detail = call['output']
            advice = detail.get('relative_yield') or {}
            reason = (advice.get('current_observer') or {}).get('reason')
            reasons[reason or 'missing'] = reasons.get(reason or 'missing', 0) + 1
            if not advice.get('cap_requested'):
                continue
            requested += 1
            positive += 0 < speed <= .20 + 1e-9
            now = call['input']['reference_s']
            first = now if first is None else first
            last = now
    checks = limited_checks(original, requested, positive)
    checks['trace_unchanged'] = hashlib.sha256(trace.read_bytes()).hexdigest() == before
    return dict(
        passed=all(checks.values()), checks=checks,
        scope='155s representative scripted restart/yield return and full lap; not hardware',
        original_full_course=original,
        relative_yield=dict(requested_calls=requested, positive_capped_commands=positive,
                            first_s=first, last_s=last, observer_reasons=reasons),
        trace_sha256=before,
    )


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = evaluate(args.run)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps(dict(passed=result['passed'], checks=result['checks'],
                          yield_usage=result['relative_yield'])))
    raise SystemExit(0 if result['passed'] else 1)
