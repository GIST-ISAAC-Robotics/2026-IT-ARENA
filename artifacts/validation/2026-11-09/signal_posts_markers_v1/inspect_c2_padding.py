"""C2 정체의 명령·시간 팽창 반환을 원기록과 대조한다. 제어 변경 없음."""
import json
from pathlib import Path
import numpy as np
from replay_tools import build_controller, read_events
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision
from arena_autonomy.bypass_handoff_corridor_trial import ForecastGuide
A = Path(__file__).resolve().parent
original = ForecastGuide._padding
captures = []

def traced(self, scene, config, side, wall, growth_s=0.):
    padding, evidence = original(self, scene, config, side, wall, growth_s)
    if 24.17 < scene.t_ref_s < 24.19 and growth_s == 0.:
        rays = [r for r in scene.rays if r.status == 'return']
        points = np.asarray([r.hit_xy_m for r in rays])
        inside = (abs(points[:, 0]) <= config.length_m/2+padding) & (abs(points[:, 1]) <= config.width_m/2+padding)
        captures.append(dict(t=scene.t_ref_s, side=side, points=[dict(index=rays[i].source_index,
            xy_m=points[i].tolist(), padding_m=float(padding[i]), age_s=scene.t_ref_s-rays[i].acquired_at_s)
            for i in np.flatnonzero(inside)], evidence=evidence))
    return padding, evidence

ForecastGuide._padding = traced
calls = mismatches = 0
for event in read_events(A/'c_stationary_posts/run2/follow_decisions.jsonl'):
    d = event['data']
    if event['kind'] == 'header': c, mc = build_controller(d)
    elif event['kind'] == 'reset': c.reset()
    elif event['kind'] == 'call':
        out = freeze(replay_call(c, d['input'], mc)); calls += 1
        mismatches += not equal_output(comparable_decision(out), comparable_decision(d['output']))
        if d['input']['reference_s'] > 24.19: break
result = dict(calls=calls, mismatches=mismatches, captures=captures)
(A/'c2_padding_diagnosis.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(dict(calls=calls, mismatches=mismatches, captures=[dict(t=x['t'], side=x['side'],
    points=x['points']) for x in captures[:1]])), flush=True)
