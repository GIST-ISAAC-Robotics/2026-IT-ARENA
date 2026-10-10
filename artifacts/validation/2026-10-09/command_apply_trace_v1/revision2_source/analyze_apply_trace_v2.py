#!/usr/bin/env python3
"""`analyze_apply_trace.py`(r2 계약에 동결된 v1) revision 2(2026-10-10). v1 파일은 보존한다.

보강 두 가지만 v1 모듈 함수에 덮어써 같은 CLI로 실행한다.
1. registered_sources: 계약의 실행 소스(평가 스크립트 after·런타임 sha256·구현 sha256) 전체가 report에 있고 같아야 한다.
   v1은 report에 없는 경로를 건너뛰어 필수 소스 삭제를 통과시켰다.
2. first_application_difference: 차이가 생긴 전환의 시각은 두 실행 중 더 이른 적용 시각이다.
   v1은 a 실행 시각만 써 b가 먼저 적용한 경우 1 step 늦게 잡을 수 있었다.
"""
import analyze_apply_trace as v1
from analyze_scheduled_phase import VALUE_TOL


def required_sources(contract):
    required = {path: change['after'] for path, change in contract['approved_evaluation_scripts'].items()}
    for path, value in contract['sha256'].items():
        required[path] = value
    for path, value in contract['implementation_sha256'].items():
        required[path] = value
    return required


def registered_sources(recorded, contract):
    if not isinstance(recorded, dict):
        return False
    return all(recorded.get(path) == value for path, value in required_sources(contract).items())


def first_application_difference(trace_a, trace_b):
    out, first = {}, None
    for channel in ('speed', 'steering'):
        x, y = trace_a[channel], trace_b[channel]
        if len(x) != len(y) or any(abs(p['to']-q['to']) > VALUE_TOL for p, q in zip(x, y)):
            out[channel] = {'comparable': False, 'applies': [len(x), len(y)]}
            continue
        diffs = [q['apply_rel_ns']-p['apply_rel_ns'] for p, q in zip(x, y)]
        out[channel] = {'comparable': True, 'apply_diff_ns': diffs, 'differing': sum(d != 0 for d in diffs),
                        'steps_after_rx': [[p['steps_after_rx'], q['steps_after_rx']] for p, q in zip(x, y)]}
        for p, q, d in zip(x, y, diffs):
            if d:
                earlier = min(p['apply_rel_ns'], q['apply_rel_ns'])
                first = earlier if first is None else min(first, earlier)
                break
    out['first_difference_rel_ns'] = first
    return out


if __name__ == '__main__':
    # CLI 실행에서만 v1 모듈 함수를 교체한다(import 시 v1 동작을 바꾸지 않음).
    v1.registered_sources = registered_sources
    v1.first_application_difference = first_application_difference
    v1.main()
