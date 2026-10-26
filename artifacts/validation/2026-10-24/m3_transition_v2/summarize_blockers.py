"""run3 행 기록의 남은 막힘 분류(읽기 전용 사후 분석). 판정 소스를 다시 계산하지 않는다."""
import json
from pathlib import Path
import sys

RECORDS = ('e3_onroad_g1', 'e3_onroad_r0plus', 'e3_stop_restart_g1')


def classify_stop(row):
    """HOLD_STOP_TRIAL 판단에서 평가된 모든 후보의 거절 이유를 묶는다."""
    reasons = row['plan_reasons']
    obs = row['plan_observation']
    if reasons and all('lateral_clearance_short' in r for r in reasons.values()):
        return 'lateral_budget_all_candidates'
    tags = set()
    for key, r in reasons.items():
        o = obs.get(key) or {}
        if r == ['current_sweep_unobserved'] and o.get('unsupported') is not None:
            masked_left = (o.get('masked_candidates') or 0)-(o.get('history_supported') or 0)
            tags.add('masked_band_unverified' if o['unsupported'] <= masked_left else 'unobserved_other')
        elif 'grass_edge_reference_unverified' in r:
            tags.add('grass_edge_unverified')
        elif 'geometry_invalid' in r:
            tags.add('geometry_invalid')
        elif 'lateral_clearance_short' in r:
            tags.add('lateral_budget')
        else:
            tags.add('+'.join(sorted(r)) or 'none')
    order = ('masked_band_unverified', 'grass_edge_unverified', 'lateral_budget', 'unobserved_other', 'geometry_invalid')
    for tag in order:
        if tag in tags:
            return tag
    return sorted(tags)[0] if tags else 'none'


def classify_command(row):
    o = row.get('envelope_observation') or {}
    if row['reason'] == 'm3_command_unobserved':
        masked_left = (o.get('masked_candidates') or 0)-(o.get('history_supported') or 0)
        return 'masked_band_unverified' if (o.get('unsupported') or 0) <= masked_left else 'unobserved_other'
    return '+'.join(sorted({x for g in row.get('envelope_geometry') or [] for x in g})) or 'geometry'


def count(values):
    out = {}
    for v in values:
        out[v] = out.get(v, 0)+1
    return dict(sorted(out.items(), key=lambda kv: -kv[1]))


def main():
    run = Path(sys.argv[1])
    result = {}
    for mode in ('recorded', 'self_steering'):
        for name in RECORDS:
            path = run/mode/f'rows_{name}_full.jsonl'
            if not path.exists():
                continue
            rows = [json.loads(line) for line in path.open(encoding='utf-8')]
            stops = [r for r in rows if r['mode'] == 'HOLD_STOP_TRIAL']
            commands = [r for r in rows if r['reason'] in ('m3_command_unobserved', 'm3_command_geometry_unconfirmed')]
            stopped = [r for r in stops if abs(r['speed']) <= 1e-6]
            result.setdefault(mode, {})[name] = {
                'hold_stop_calls': len(stops),
                'hold_stop_by_cause': count(classify_stop(r) for r in stops),
                'hold_stop_while_measured_stopped': len(stopped),
                'command_rejections_by_cause': count(classify_command(r) for r in commands),
                'reasons': count(r['reason'] for r in rows if r['mode'] not in ('BASELINE', 'BASE_GATE')),
            }
    out = run/'analysis'
    out.mkdir(exist_ok=True)
    (out/'remaining_blockers.json').write_text(json.dumps(result, indent=1, ensure_ascii=False), encoding='utf-8',
                                               newline='\n')
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
