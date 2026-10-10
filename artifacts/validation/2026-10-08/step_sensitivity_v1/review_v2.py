#!/usr/bin/env python3
"""최종 검토 보완(v2). 기존 분석·판정·사전 기준은 그대로 두고 다음을 분리해 기록한다.

1. 실행/보존 유효성(기존 audit_passed)
2. step 단독 효과 식별 가능성(요청/safe/MCU 적용 입력 동일성)
또 새 분석기 출력의 지표·분류가 기존 출력과 같은지(지표 회귀)와 산출물 해시를 확인한다.
"""
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
OLD_SCRIPT_SHA = 'd34e19b1f7b9e9fc3760f47d5ca880ce939bb6c854e9585ede04308b9839239c'  # pre_run_freeze.sha256 기록값
PAIRS = ['analysis_passive', 'analysis_legacy_aux', 'analysis_official_rerun_ref', 'analysis_official_repeat025']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(name):
    return json.loads((BASE/f'{name}.json').read_text(encoding='utf-8'))


freeze = dict(reversed(line.split(' *', 1)) for line in (BASE/'pre_run_freeze.sha256').read_text().splitlines())
regression, identifiability = {}, {}
for name in PAIRS:
    old, new = load(name), load(f'{name}_v2')
    regression[name] = {
        'old_script_sha256_is_registered': old['analysis_sha256'] == OLD_SCRIPT_SHA == freeze['scripts/analyze_physics_step.py'],
        'new_script_sha256_current': new['analysis_sha256'] == sha(ROOT/'scripts/analyze_physics_step.py'),
        'audit_same': old['audit_passed'] == new['audit_passed'],
        'metrics_same': [r['metrics'] for r in old['runs']] == [r['metrics'] for r in new['runs']],
        'classifications_same': ({k: v['classification'] for k, v in old['classifications'].items()} ==
                                 {k: v['classification'] for k, v in new['classifications'].items()}),
        'aligned_command_summaries_same': [c['aligned_commands'] for c in old['comparisons']] ==
                                          [c['aligned_commands'] for c in new['comparisons']]}
    identifiability[name] = {
        'execution_preservation_valid': new['execution_preservation_valid'],
        'step_only_effect_identifiable': new['step_only_effect_identifiable'],
        'comparisons': [{'surface': c['surface'], 'candidate_step_s': c['candidate_step_s'],
                         'reference': c['reference'].split('/')[-1], 'candidate': c['candidate'].split('/')[-1],
                         'mismatched_of_samples': {k: [v['mismatched'], v['samples']] for k, v in c['aligned_mismatches'].items()},
                         'max_abs': {k: v['max_abs'] for k, v in c['aligned_commands'].items()}}
                        for c in new['comparisons']],
        'classifications': {k: v['classification'] for k, v in new['classifications'].items()}}
timing = {}
for name in PAIRS:
    for run in load(f'{name}_v2')['runs']:
        timing[run['path']] = run['timing'] and round(run['timing']['real_time_factor'], 4)
phase = json.loads((BASE/'command_phase_v2.json').read_text())
failed_shutdown = (BASE/'analysis_flat_legacy_failed_shutdown_v2.log').read_text()
duplicate = (BASE/'analysis_duplicate_negative_v2.log').read_text()
checks = {
    'metric_regression_identical': all(all(v.values()) for v in regression.values()),
    'execution_preservation_valid_all': all(v['execution_preservation_valid'] for v in identifiability.values()),
    'step_only_effect_identifiable_any': any(v['step_only_effect_identifiable'] for v in identifiability.values()),
    'v1_last_transition_equals_speed_zero': phase['last_transition_is_speed_zero'],
    'failed_shutdown_analysis_still_rejected': '"audit_passed": false' in failed_shutdown,
    'duplicate_input_rejected_on_real_runs': 'duplicate run for official/passive_damping/0.00025' in duplicate
                                             and not (BASE/'analysis_duplicate_negative_v2.json').exists(),
}
result = {'checks': checks, 'regression': regression, 'identifiability': identifiability,
          'real_time_factor': timing, 'speed_zero_lag_after_profile_s': {k: v['speed_zero_lag_after_profile_s']
                                                                          for k, v in phase['runs'].items()},
          'interpretation': [
              '기존 audit_passed/final_audit.json은 실행·보존 유효성 판정이다. 적용 입력 동일성(step 단독 비교 타당성)을 판정하지 않았다.',
              '모든 step 비교에서 요청/safe/MCU 목표의 20 ms 격자 불일치 표본이 있어 step 단독 효과는 식별되지 않는다.',
              'consistent/not_converged는 사전 허용치에 따른 여러 실행 지표 분류이며 수학적 수렴 증명이 아니다.',
              '같은 step 반복에서 출력과 발행/전달 위상이 함께 달랐다. 위상은 후보 원인이며 원인 분리는 미완료다.',
              '작은 step 반복은 2회뿐이고 기존식 0.25 ms 두 실행도 달랐으므로 0.25 ms의 안정성을 확정하지 않는다.'],
          'review_script_sha256': sha(Path(__file__))}
with (BASE/'final_review_v2.json').open('x', encoding='utf-8') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2)
print(json.dumps({'checks': checks, 'real_time_factor': timing}, ensure_ascii=False, indent=2))
for name, value in identifiability.items():
    for c in value['comparisons']:
        print(name, c['reference'], c['candidate'], c['mismatched_of_samples'])
