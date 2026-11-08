"""replay_after_rows.jsonl 요약. 정지 전 차이와 정지 구간 명령 분포."""
import json
from collections import Counter
from pathlib import Path
HERE = Path(__file__).resolve().parent
rows = [json.loads(l) for l in open(HERE/'replay_after_rows.jsonl', encoding='utf-8') if '"t"' in l]
def summary(rs):
    ds = [r for r in rs if r['recorded']['speed'] != r['after']['speed'] or r['recorded']['steer'] != r['after']['steer']
          or r['recorded']['reason'] != r['after']['reason'] or r['recorded']['normal'].get('reason') != r['after']['normal'].get('reason')]
    return dict(calls=len(rs), command_or_reason_differences=len(ds),
                first_t=ds[0]['t'] if ds else None,
                speed_diff_calls=sum(r['recorded']['speed'] != r['after']['speed'] for r in rs),
                max_abs_speed_diff=max((abs(r['recorded']['speed']-r['after']['speed']) for r in rs), default=0),
                max_abs_steer_diff=max((abs(r['recorded']['steer']-r['after']['steer']) for r in rs), default=0),
                reason_pairs=Counter(f"{r['recorded']['normal'].get('reason')}->{r['after']['normal'].get('reason')}" for r in ds).most_common(8),
                selection_after=Counter(str(r['after']['normal'].get('boundary_selection')) for r in rs).most_common(6),
                budget_hit_calls=sum(r['corner_path_budget_hits'] > 0 for r in rs))
pre = [r for r in rows if r['t'] < 28.12]
stall = [r for r in rows if r['t'] >= 28.12]
out = {'pre_stall_0_28_12': summary(pre), 'stall_28_12_38': summary(stall)}
st = stall
out['stall_after_speed_quantiles'] = sorted(r['after']['speed'] for r in st)[::50]
out['stall_after_steer_range'] = [min(r['after']['steer'] for r in st), max(r['after']['steer'] for r in st)]
out['stall_after_swept_clearance_range'] = [min(r['after']['normal'].get('swept_clearance_m', 9) for r in st),
                                            max(r['after']['normal'].get('swept_clearance_m', -9) for r in st)]
out['stall_after_path_wall'] = Counter(str(r['after']['normal'].get('path_wall')) for r in st).most_common()
out['stall_measured_speed_max'] = max(r['measured_speed_mps'] for r in st)
out['first_stall_rows'] = [{k: r[k] for k in ('t', 'recorded', 'after')} for r in st[:2]]
(HERE/'analyze_after.json').write_text(json.dumps(out, indent=1, ensure_ascii=False)+'\n', encoding='utf-8')
print(json.dumps(out, ensure_ascii=False)[:5000])
