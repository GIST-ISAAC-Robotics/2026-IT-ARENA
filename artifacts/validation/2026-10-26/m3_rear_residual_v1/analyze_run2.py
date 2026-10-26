"""run2 저장 입력 비교의 명령 단위 대조와 해제 시각 사후 정답 위치 요약(평가 전용)."""
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
run = HERE/'run2_premises'
summary = json.loads((run/'summary.json').read_text(encoding='utf-8'))
out = {'scope': 'stored-input open-loop; command (speed, steer) comparison vs current source; truth only post hoc',
       'inputs': {}}
for name, entry in summary['inputs'].items():
    plain = json.loads((run/f'{name}__plain.json').read_text(encoding='utf-8'))
    calls = [r for r in plain if not r.get('reset')]
    item = {}
    for variant, value in entry['variants'].items():
        rows = [r for r in json.loads((run/f'{name}__{variant}.json').read_text(encoding='utf-8')) if not r.get('reset')]
        diff = [(a['t'], a['speed'], b['speed'], a['steer'], b['steer']) for a, b in zip(calls, rows)
                if abs(a['speed']-b['speed']) > 1e-12 or abs(a['steer']-b['steer']) > 1e-12]
        truth = value.get('posthoc_at_first_release') or {}
        item[variant] = {
            'released_at': value['released_at'][:3], 'positive': value['positive'],
            'command_differences_vs_plain': len(diff), 'first_command_difference_t': diff[0][0] if diff else None,
            'full_output_differences_vs_plain': value.get('output_differences_vs_plain'),
            'active_calls': value['active_calls'], 'memory_blocked_first_t': value['memory_blocked_first_t'],
            'memory_blocked_reasons': value['memory_blocked_reasons'], 'cost_ms': value['cost_ms'],
            'truth_at_release': {k: {kk: round(vv, 4) for kk, vv in v.items() if kk != 'pose_t'}
                                 for k, v in truth.items() if isinstance(v, dict)}}
    out['inputs'][name] = item
(HERE/'run2_analysis.json').write_text(json.dumps(out, ensure_ascii=False, indent=1)+'\n', encoding='utf-8', newline='\n')
for name, item in out['inputs'].items():
    print(name)
    for variant, v in item.items():
        print(f"  {variant:24s} rel={v['released_at']} pos={v['positive']} cmd_diff={v['command_differences_vs_plain']}"
              f" full_diff={v['full_output_differences_vs_plain']} latch={v['memory_blocked_first_t']}"
              f" p95={v['cost_ms']['p95']:.0f} max={v['cost_ms']['max']:.0f} truth={v['truth_at_release']}")
sys.exit(0)
