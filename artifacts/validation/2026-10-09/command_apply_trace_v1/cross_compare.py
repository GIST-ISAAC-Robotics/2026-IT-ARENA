#!/usr/bin/env python3
"""탐색 비교(관문 아님): 새 실행과 10/8 R1/R5의 같은 sim 시각 진단 분기·스폰 부류.

같은 프로필 시작 시각(start_ns)인 쌍만 비교한다. 인자: <출력 json> <이름=경로>... (경로는 저장소 기준)
"""
import json
from itertools import combinations
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
from analyze_apply_trace import divergence, spawn_from_diagnostics  # noqa: E402
from analyze_braking_ab import rows  # noqa: E402


def main():
    output, *items = sys.argv[1:]
    runs = {}
    for item in items:
        name, path = item.split('=', 1)
        report = json.loads((ROOT/path/'report.json').read_text(encoding='utf-8'))
        data = rows(ROOT/path/'braking_diagnostics.jsonl')
        events = ROOT/path/'plugin_command_events.jsonl'
        first = None
        if events.is_file():
            first = next((json.loads(l) for l in events.read_text(encoding='utf-8').splitlines()
                          if '"first_update"' in l), None)
        runs[name] = {'path': path, 'start_ns': report['command_schedule']['start_ns'], 'data': data,
                      'spawn_from_diagnostics': spawn_from_diagnostics(data),
                      'trace_first_update': first and {k: first[k] for k in ('iteration', 'sim_ns')}}
    pairs = []
    for a, b in combinations(runs, 2):
        if runs[a]['start_ns'] != runs[b]['start_ns']:
            pairs.append({'a': a, 'b': b, 'comparable': False, 'reason': 'profile start differs'})
            continue
        pairs.append({'a': a, 'b': b, 'comparable': True,
                      'same_spawn_class': runs[a]['spawn_from_diagnostics'] == runs[b]['spawn_from_diagnostics'],
                      **divergence(runs[a]['data'], runs[b]['data'], runs[a]['start_ns'])})
    result = {'runs': {k: {x: v[x] for x in ('path', 'start_ns', 'spawn_from_diagnostics', 'trace_first_update')}
                       for k, v in runs.items()}, 'pairs': pairs,
              'scope': '탐색 비교. 소스/빌드가 다른 실행 사이의 일치는 물리 결정성과 계측 무영향의 보조 근거일 뿐 관문이 아니다.'}
    with open(output, 'x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    for p in pairs:
        print(p['a'], p['b'], p.get('same_spawn_class'), p.get('first_any_difference_rel_ns'),
              p.get('first_wheel_or_body_speed_exceeds_rel_ns'))


if __name__ == '__main__':
    main()
