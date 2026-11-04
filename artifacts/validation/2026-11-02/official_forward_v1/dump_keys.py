"""전환 판단의 하위 근거 추출. 평가 전용. 인자: 실행, 시작, 끝, 간격(초)."""
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
name, lo, hi, step = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
green = json.loads((HERE/name/'report.json').read_bytes())['evaluation']['t_green']


def small(value):
    if isinstance(value, dict):
        return {k: small(v) for k, v in value.items() if not isinstance(v, (list,)) and k not in ('path_points', 'selector_sides', 'corridor_candidate', 'fragmented_boundary_evidence', 'partial_observation', 'parallel_hold_trial')}
    return round(value, 3) if isinstance(value, float) else value


next_t = lo
for line in (HERE/name/'follow_decisions.jsonl').open(encoding='utf-8'):
    e = json.loads(line)
    if e['kind'] != 'call':
        continue
    i, o = e['data']['input'], e['data']['output']
    t = i['reference_s']-green
    if t < next_t or t > hi:
        continue
    next_t = t+step
    d = o[2]
    n = d.get('normal', {})
    out = dict(t=round(t, 2), cmd=round(o[0], 3), state=d.get('handoff_state'), reason=d.get('reason'),
               normal=dict(cmd_reason=n.get('reason'), base=n.get('baseline_reason'), wall=n.get('path_wall'), fallback=n.get('wall_fallback'),
                           ahead=n.get('observed_path_ahead_m'), clear=n.get('swept_clearance_m'), target=n.get('target_speed_before_hold_mps'),
                           open=n.get('open_route_speed_enabled'), selection=n.get('boundary_selection'), held=n.get('hold_reason')),
               persistence=small(d.get('persistence')), preparation=small(d.get('preparation')), space=small(d.get('follow_entry_space')),
               entry_space=small(d.get('entry_space')), admission=small(d.get('entry_admission')))
    bypass = d.get('bypass') or {}
    out['bypass'] = {k: small(bypass.get(k)) for k in ('reason', 'entry_phase', 'selected', 'phase', 'target_speed_mps') if k in bypass}
    extra = {k: small(v) for k, v in d.items() if 'yield' in k or 'pause' in k or 'hold' in k}
    if extra:
        out['extra'] = extra
    print(json.dumps(out, ensure_ascii=False)[:1100])
