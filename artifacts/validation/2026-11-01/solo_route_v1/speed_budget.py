"""완주한 프리랩의 명시 속도 상한을 분해한다. 새로운 제어 정책을 실행하지 않는다."""
import ast
from collections import Counter
import json
import math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def main():
    tree = ast.parse((ROOT/'src/arena_autonomy/arena_autonomy/local_path.py').read_text(encoding='utf-8'))
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'braking_speed')
    namespace = {'math': math}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), 'braking_speed', 'exec'), namespace)
    brake = namespace['braking_speed']
    m = json.loads((HERE/'run1_metrics.json').read_text(encoding='utf-8'))
    start = m['first_green_sample_s']
    end = start+m['lap_s_from_first_green_sample']
    limits, reasons, errors = Counter(), Counter(), []
    samples = []
    for line in (HERE/'run1/follow_decisions.jsonl').open(encoding='utf-8'):
        row = json.loads(line)
        d = row['data']
        if row['kind'] == 'header':
            c = d['follow_config']
        if row['kind'] != 'call':
            continue
        inp = d['input']; v, steering, out = d['output']; t = inp['reference_s']
        if not start <= t <= end:
            continue
        dt = inp['dt_s']; reasons[out.get('reason')] += dt
        path = out.get('path_points')
        if not path or out.get('reason') != 'local_follow':
            continue
        rear = np.asarray(path, dtype=float).copy()
        rear[:, 0] += c['wheelbase_m']/2
        distances = np.linalg.norm(rear, axis=1)
        nearest = int(np.argmin(distances))
        ahead = np.flatnonzero((rear[:, 0] > .1) & (np.arange(len(rear)) >= nearest))
        look = min(.6, max(.3, .3+.12*abs(inp['speed_mps'])))
        target = rear[ahead[np.argmin(abs(distances[ahead]-look))]]
        curvature = float(2*target[1]/max(.01, target@target))
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(rear, axis=0), axis=1))]
        remaining = float(arc[-1]-arc[nearest])
        dx = np.gradient(rear, axis=0); ddx = np.gradient(dx, axis=0)
        curves = abs(dx[:, 0]*ddx[:, 1]-dx[:, 1]*ddx[:, 0])/np.maximum(1e-8, np.linalg.norm(dx, axis=1)**3)
        preview = np.sqrt(c['lateral_acceleration_mps2']/np.maximum(.15, curves[nearest:])+
                          4*np.maximum(0., arc[nearest:]-arc[nearest]-.2))
        age = t-out['source_first_s']
        caps = dict(route_cap=c['open_route_speed_mps'] if out['open_route_speed_enabled'] else c['max_speed_mps'],
            path_stop=brake(remaining, max(age, out['boundary_age_s'])),
            path_curvature_preview=float(min(preview)),
            pursuit_curvature=math.sqrt(c['lateral_acceleration_mps2']/max(.15, abs(curvature))))
        if 'observed_length_speed_cap_mps' in out:
            caps['path_length_blend'] = out['observed_length_speed_cap_mps']
        available = max(0., out['swept_clearance_m']-c['standstill_gap_m'])
        caps.update(constant_arc_stop=brake(available, age), constant_arc_headway=available/c['headway_s'])
        before = min(caps.values())
        expected = out.get('constant_arc_target_mps', out['target_speed_before_hold_mps'])
        errors.append(abs(before-expected))
        if abs(before-expected) > 1e-7:
            raise ValueError('기록된 속도 상한을 재현하지 못함')
        limiting = min(caps, key=caps.get)
        limits[(limiting, out.get('turn_preview') == 'observed_turn_clear')] += dt
        samples.append(dict(t=t, target_mps=v, before_preview_mps=before, limiting=limiting,
                            caps=caps, preview=out.get('turn_preview')))
    result = dict(window_s=[start, end], matching_calls=len(errors), max_error_mps=max(errors),
        by_reason_s=dict(reasons), limiting_s=[dict(limit=k[0], preview_lift=k[1], seconds=v) for k,v in limits.most_common()],
        peak_targets=sorted(samples,key=lambda r:r['target_mps'],reverse=True)[:8],
        scope='같은 저장 경로에서 속도 상한을 분해한 진단. 가속/독립 보호/실제 차체 반응이나 새 설정 주행 예측이 아니다.')
    with (HERE/'speed_budget.json').open('x', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(json.dumps({k:v for k,v in result.items() if k!='peak_targets'},ensure_ascii=False))


if __name__ == '__main__':
    main()
