"""run1 기록의 동일 재생 중 측방 창 판정을 계측한다(평가 전용). 제어 출력은 바꾸지 않는다.

`band_check` 호출을 감싸 실제 결과를 그대로 돌려주고, 확장 창 호출일 때만 뒤 범퍼(−L/2)~앞 0.35 m로 줄인 창의
결과를 **설명용 반사실**로 함께 기록한다. 정답 자세는 옆차 위치의 사후 표시에만 쓴다.
"""
from bisect import bisect_right
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
for part in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts'):
    sys.path.insert(0, str(ROOT/part))

import arena_autonomy.pass_release_trial as ps  # noqa: E402
from arena_autonomy.follow_trace import replay_events  # noqa: E402

RUN = HERE/'run1_corner'
original = ps.band_check
calls = []


def instrumented(scene, frame, sign, cfg, sel, longitudinal=None):
    result = original(scene, frame, sign, cfg, sel, longitudinal)
    row = {'t': scene.t_ref_s, 'sign': sign, 'kind': 'band' if longitudinal is None else 'window', 'result': result}
    if longitudinal is not None:
        row['ahead_only'] = original(scene, frame, sign, cfg, sel, (-cfg.length_m/2, longitudinal[1]))
    calls.append(row)
    return result


ps.band_check = instrumented
with (RUN/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
    report = replay_events(json.loads(line) for line in stream)
ps.band_check = original
ego = [json.loads(s) for s in (RUN/'ego_poses.jsonl').open(encoding='utf-8')]
side = json.loads((RUN/'additional_actors.json').read_text(encoding='utf-8'))['poses']['eval_e3_side']
et, st = [r[0] for r in ego], [r[0] for r in side]
windows = [r for r in calls if r['kind'] == 'window']
for r in windows:
    e = ego[max(0, bisect_right(et, r['t'])-1)]
    a = side[max(0, bisect_right(st, r['t'])-1)]
    c, s = math.cos(e[3]), math.sin(e[3])
    dx, dy = a[1]-e[1], a[2]-e[2]
    r['eval_side_actor_ego_xy'] = [c*dx+s*dy, -s*dx+c*dy]


def first_two(rows, key):
    run = 0
    for r in rows:
        run = run+1 if r[key]['free'] else 0
        if run >= 2:
            return r['t']
    return None


after = [r for r in windows if r['t'] >= 20.3]
summary = {'scope': 'instrumented identical replay of run1; ahead_only window is an explanatory counterfactual, not control',
           'replay': {k: report[k] for k in ('calls', 'matched', 'passed')},
           'window_calls_after_20_3s': len(after),
           'window_free_after_20_3s': sum(r['result']['free'] for r in after),
           'window_first_two_free_after_20_3s': first_two(after, 'result'),
           'ahead_only_free_after_20_3s': sum(r['ahead_only']['free'] for r in after),
           'ahead_only_first_two_free_after_20_3s': first_two(after, 'ahead_only'),
           'band_calls': sum(r['kind'] == 'band' for r in calls),
           'band_not_free_times': [r['t'] for r in calls if r['kind'] == 'band' and not r['result']['free']][:40]}
out = {'summary': summary, 'window_rows': [{'t': r['t'], 'window': r['result'], 'ahead_only': r['ahead_only'],
                                            'eval_side_actor_ego_xy': r['eval_side_actor_ego_xy']} for r in windows]}
(HERE/'run1_window_counterfactual.json').write_text(json.dumps(out, ensure_ascii=False, indent=1)+'\n', encoding='utf-8',
                                                    newline='\n')
print(json.dumps(summary, ensure_ascii=False))
