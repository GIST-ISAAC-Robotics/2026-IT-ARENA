"""run3 측방 선택 뒤 양수 판단의 사후 정답 간격 분류(읽기 전용). 개루프 기록 자세의 간격이다."""
import json, math, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from analyze_run17 import load_truth, at, rect_distance, to_world  # noqa: E402

RUN = HERE/'run3_execution_replay'
BORDER = np.array([[x, y] for x in np.linspace(-.1, .1, 21) for y in (-.075, .075)] +
                  [[x, y] for x in (-.1, .1) for y in np.linspace(-.075, .075, 16)])
out = {}
for mode in ('recorded', 'self_steering'):
    for name in ('e3_onroad_g1', 'e3_onroad_r0plus', 'e3_stop_restart_g1'):
        rows = [json.loads(l) for l in (RUN/mode/f'rows_{name}_full.jsonl').open(encoding='utf-8')]
        ego, actors = load_truth(name)
        items = []
        for r in rows:
            if not (r['selected'] and r['output'][0] > 0):
                continue
            pose = at(ego, r['t'])
            world = to_world(BORDER, pose)
            best = None
            for key, series in actors.items():
                a = at(series, r['t'])
                d = float(np.min(rect_distance(world, a)))
                c, s = math.cos(pose[2]), math.sin(pose[2])
                dx, dy = a[0]-pose[0], a[1]-pose[1]
                rel = (c*dx+s*dy, -s*dx+c*dy)
                if best is None or d < best['gap_m']:
                    best = {'actor': key, 'gap_m': d, 'rel_x_m': rel[0], 'rel_y_m': rel[1]}
            # 자차 센서(0.06,0) 기준 상대차 중심 방위. 150° 초과면 후방 마스크 쪽
            bearing = math.degrees(math.atan2(best['rel_y_m'], best['rel_x_m']-.06))
            items.append({'t': r['t'], 'candidate': r['candidate'], 'speed_cmd': r['output'][0], **best,
                          'center_bearing_deg': bearing})
        close = [i for i in items if i['gap_m'] < .097]
        out.setdefault(mode, {})[name] = {
            'trial_positive': len(items),
            'gap_below_97mm': len(close),
            'close_examples': sorted(close, key=lambda i: i['gap_m'])[:8],
            'by_actor_below_97mm': {k: sum(i['actor'] == k for i in close) for k in {i['actor'] for i in close}},
            'note': 'recorded ego pose (open loop); actor at bearing >150 deg is in the rear mask'}
(RUN/'analysis'/'posthoc_positive_gaps.json').write_text(json.dumps(out, indent=1), encoding='utf-8', newline='\n')
for mode, v in out.items():
    for name, s in v.items():
        print(mode, name, s['trial_positive'], s['gap_below_97mm'], s['by_actor_below_97mm'])
        for e in s['close_examples'][:5]:
            print('   ', {k: (round(x, 3) if isinstance(x, float) else x) for k, x in e.items()})
