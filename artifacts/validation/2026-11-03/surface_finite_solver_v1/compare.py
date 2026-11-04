"""격리 유한선분 후보를 첫 실패 센서 문맥과 기존 반례로 비교한다. 런타임 수정 없음."""
import importlib.util
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'src/arena_autonomy'))
from arena_autonomy.follow_trace import thaw


def load(name):
    spec = importlib.util.spec_from_file_location(f'isolated_surface_{name}', HERE/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.surface_translation


def first_context():
    trace = ROOT/'artifacts/validation/2026-11-02/solo_ray_combination_v1/run1/follow_decisions.jsonl'
    with trace.open(encoding='utf-8') as stream:
        for line in stream:
            event = thaw(json.loads(line))
            if event['kind'] != 'call':
                continue
            part = event['data']['output'][2].get('partial_observation') or {}
            if part.get('latch_evidence'):
                return event['data']['input']['reference_s'], part['latch_evidence']['correspondence_context']
    raise ValueError('preserved first latch not found')


def main():
    ref, context = first_context()
    original, candidate = load('original'), load('candidate')
    args = (context['previous_points_now_xy'], context['current_return_points_xy'], context['current_return_source_indices'])
    results = {name: fn(*args) for name, fn in [('original', original), ('candidate', candidate)]}
    output = dict(scope='첫 래치 센서 문맥만. 재생/주행/런타임 변경 없음.', reference_s=ref, variants=results)
    (HERE/'first_latch.json').write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')
    print(json.dumps({k: dict(accepted=v['accepted'], reason=v['reason'], translation=v.get('translation_m'),
                             steps=len(v.get('steps', [])), last=v.get('steps', [None])[-1]) for k, v in results.items()}, ensure_ascii=False))
    # 프로세스 내부에서만 후보로 치환한 뒤 기존 반례를 실행한다. 디스크/runtime은 그대로다.
    import arena_autonomy.partial_surface_alignment as target
    import pytest
    target.surface_translation = candidate
    code = pytest.main(['-q', str(ROOT/'tests/test_partial_surface_alignment.py')])
    (HERE/'counterexamples.exit').write_text(f'ACTUAL_EXIT_CODE={code}\n', encoding='utf-8')
    return code


if __name__ == '__main__':
    raise SystemExit(main())
