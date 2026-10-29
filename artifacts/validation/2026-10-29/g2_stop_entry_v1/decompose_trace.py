"""closed1 schema5 기록에서 진입 직전/직후 G2 판단을 분해한다. 새 주행 아님.

인수: 사례 이름, 시작/끝 시각. 평가 자세(records.json)는 사후 확인용이다.
"""
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
CLOSED = BASE.parent/'g2_prepare_cap_v1/closed1'
name, lo, hi = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
policy = 'observed_entry_relevance_v1'
records = {round(r['t'], 3): r for r in json.load(open(CLOSED/(name+'__'+policy+'.records.json'), encoding='utf-8'))}
rows = []
with open(CLOSED/(name+'__'+policy+'.jsonl'), encoding='utf-8') as stream:
    for line in stream:
        event = json.loads(line)
        if event['kind'] != 'call':
            continue
        data = event['data']; t = round(data['input']['reference_s'], 3)
        if not lo <= t <= hi:
            continue
        out = data['output']; d = out[2]; b = d.get('bypass') or {}
        rec = records.get(t, {})
        ev = rec.get('evaluation', {})
        cands = b.get('candidates') or []
        sel = b.get('selected')
        chosen = None
        for c in cands:
            if sel and c.get('offset_m') == sel.get('offset_m') and c.get('transition_m') == sel.get('transition_m'):
                chosen = c
        if chosen is None and len(cands) == 1:
            chosen = cands[0]
        row = dict(t=t, state=d.get('handoff_state'), reason=d.get('reason'), g2=b.get('reason'), phase=b.get('entry_phase'),
            out=[round(out[0], 3), round(out[1], 3)], prev=round(data['input']['previous_steering_rad'], 3),
            v=round(data['input']['speed_mps'], 3), sel=None if not sel else (sel.get('offset_m'), sel.get('transition_m')),
            truth_center=[round(x, 3) for x in ev.get('center_pose', [])], truth_gap=round(ev.get('target_body_gap_m', -1), 3))
        if chosen:
            row.update(cand=(chosen.get('offset_m'), chosen.get('transition_m')), c_reason=chosen.get('reason'),
                steer=round(chosen.get('steering_rad', 0), 3), tgt=round(chosen.get('target_speed_mps', 0), 3),
                clear=round(chosen.get('command_clearance_m', 0), 3), prog=round(chosen.get('preview_return_progress_m', 0), 3),
                pgap=round(chosen.get('preview_point_gap_m', 0), 3), reach=chosen.get('transition_reaches_offset'),
                maxst=round(chosen.get('preview_maximum_steering_rad', 0), 3),
                path_end=[round(x, 3) for x in (chosen.get('path_points') or [[0, 0]])[-1]],
                path_len=len(chosen.get('path_points') or []))
        if d.get('handoff_state') == 'FOLLOW' and cands:
            row['entry_options'] = [(c['offset_m'], c['transition_m'], c['reason'][:24], round(c.get('preview_return_progress_m', 0), 2),
                round(c.get('preview_point_gap_m', 0), 3)) for c in cands if c.get('offset_m') != 0][:18]
        rows.append(row)
for row in rows:
    print(json.dumps(row, ensure_ascii=False))
