#!/usr/bin/env python3
"""G1 사후 감사 v6: 사례의 실제 정지 요구와 HOLD 요청 상태를 분리한다.

구판의 드문 저속 두 표본 사이 전체가 정지해야 한다는 가정만 대체한다.
기존 모든 다른 감사와 등록된 사례의 밀집 정지 증거는 필수다. 구판 결과와
실제 이동 기록은 별도 진단으로 보존하며 제어기/물성/안전 임계값은 바꾸지 않는다.
"""
import argparse
import json
from pathlib import Path

from audit_g1_traffic_coverage import audit_rows
from audit_g1_hold_evidence import audit as hold_audit, digest


def combine(legacy, hold, run_passed):
    if 'hold_matches_dense_pose' not in legacy.get('checks', {}):
        raise ValueError('legacy evidence incomplete')
    checks = {k: v for k, v in legacy['checks'].items() if k != 'hold_matches_dense_pose'}
    required = hold['hold_requirement_applicable']
    if type(required) is not bool:
        raise ValueError('hold applicability required')
    checks.update(original_run_passed=run_passed is True,
                  registered_hold_dense_evidence=(hold['dense_hold_requirement_met'] is True if required else
                                                  hold['dense_hold_requirement_met'] is None),
                  no_observed_motion_after_settled_zero_target=not any(
                      e['movement_after_settled'] for e in hold['sampled_zero_target_episodes']))
    return dict(schema='g1_post_case_evidence_v6', checks=checks,
                passed=all(v is True for v in checks.values()),
                legacy_sparse_hold_check=dict(passed=legacy['checks']['hold_matches_dense_pose'],
                    contradictions=legacy.get('hold_pose_contradictions', []),
                    meaning='diagnostic only: two asynchronous low-speed samples are not a stationary declaration'),
                legacy_audit=legacy, hold_evidence=hold,
                original_verdict_changed=False, driving_authorized=False,
                limits=['versioned replacement of an invalid sparse-sample implication, not a new controller result',
                        'all other legacy checks and original run gates remain required',
                        'registered stationary speed/duration/window and STOP limits are unchanged',
                        'sampled evidence only; no continuous motion, contact or hardware safety proof'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists(): parser.error('output must be new')
    names = ('report.json', 'trace.json', 'autonomy_status.jsonl', 'ego_poses.jsonl',
             'stop_trace.json', 'braking_diagnostics.jsonl', 'follow_decisions.jsonl')
    paths = {name: args.run/name for name in names}
    paths['config'] = args.config
    hashes = {name: digest(path) for name, path in paths.items()}
    def read(name): return json.loads(paths[name].read_text())
    def rows(name):
        with paths[name].open(encoding='utf-8') as stream:
            return [json.loads(line) for line in stream]
    report, trace, ego = read('report.json'), read('trace.json'), rows('ego_poses.jsonl')
    if report['config_sha256'] != hashes['config']:
        raise ValueError('original config required')
    old = audit_rows(report, trace, rows('autonomy_status.jsonl'), ego, read('stop_trace.json'))
    hold = hold_audit(ego, rows('braking_diagnostics.jsonl'), rows('follow_decisions.jsonl'),
                      trace, read('config'), report)
    result = combine(old, hold, report['passed'])
    result.update(input_sha256=hashes, inputs_unchanged=all(digest(paths[n]) == h for n, h in hashes.items()),
                  source_sha256={n: digest(Path(__file__).with_name(n)) for n in (
                      'audit_g1_case_evidence.py', 'audit_g1_hold_evidence.py',
                      'audit_g1_traffic_coverage.py', 'g1_traffic_eval.py')})
    result['passed'] &= result['inputs_unchanged']
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: result[k] for k in ('schema', 'checks', 'passed', 'legacy_sparse_hold_check',
                                          'inputs_unchanged')}, ensure_ascii=False))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
