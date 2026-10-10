#!/usr/bin/env python3
"""기존 10/8 1 ms 실행(r0 probe, 추적 없음)의 스폰 부류와 같은 sim 시각 진단 분기 회고 분석.

새 실행 전 가설 등록 근거. 원본 자료는 읽기만 한다. 추적이 없으므로 적용 시점은 판정하지 않는다.
"""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
from analyze_apply_trace import divergence, spawn_from_diagnostics  # noqa: E402
from analyze_braking_ab import rows, sha  # noqa: E402

OLD = ROOT/'artifacts/validation/2026-10-08'
RUNS = {'R1': 'scheduled_phase_v1/o1ms_p10_a', 'R3': 'scheduled_phase_v1/o1ms_p0', 'R5': 'scheduled_phase_v1/o1ms_p10_b'}
PAIRS = [('R1', 'R5', 'same_condition_repeat'), ('R5', 'R3', 'phase_10_vs_0_same_spawn_class'),
         ('R1', 'R3', 'phase_10_vs_0_different_spawn_class')]
data = {k: rows(OLD/v/'braking_diagnostics.jsonl') for k, v in RUNS.items()}
reports = {k: json.loads((OLD/v/'report.json').read_text(encoding='utf-8')) for k, v in RUNS.items()}
classes = {}
for path in sorted(OLD.glob('*/*/braking_diagnostics.jsonl')):
    with path.open(encoding='utf-8') as stream:
        first = json.loads(stream.readline())
    step = '1ms' if '0p25' not in path.parent.name and 'o025' not in path.parent.name and '0p5' not in path.parent.name else 'fine'
    classes[str(path.parent.relative_to(ROOT))] = {'first_diag_sim_s': first['sim_time_s'], 'truth_z_m': first['truth_z_m'],
                                                   'step_class': step}
result = {'runs': {k: {'path': str((OLD/v).relative_to(ROOT)), 'spawn_from_diagnostics': spawn_from_diagnostics(data[k]),
                       'start_ns': reports[k]['command_schedule']['start_ns'],
                       'diagnostics_sha256': sha(OLD/v/'braking_diagnostics.jsonl')} for k, v in RUNS.items()},
          'pairs': [{'a': a, 'b': b, 'label': label,
                     **divergence(data[a], data[b], reports[a]['command_schedule']['start_ns'])} for a, b, label in PAIRS],
          'first_diagnostic_sample_all_10_08_runs': classes,
          'interpretation_limits': ['첫 진단 표본 시각은 플러그인 첫 PostUpdate이며 스폰 step의 간접 관측이다.',
                                    '추적이 없어 플러그인 적용 시점 차이는 판정하지 않는다.',
                                    '같은 값은 진단 12자리 출력 정밀도에서 같다는 뜻이다.']}
out = Path(__file__).with_name('retro_divergence_v1.json')
with out.open('x', encoding='utf-8') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2)
for p in result['pairs']:
    print(p['a'], p['b'], p['label'], p['first_any_difference_rel_ns'], p['first_any_difference_keys'][:4],
          p['first_wheel_or_body_speed_exceeds_rel_ns'])
print({k: v['spawn_from_diagnostics'] for k, v in result['runs'].items()})
