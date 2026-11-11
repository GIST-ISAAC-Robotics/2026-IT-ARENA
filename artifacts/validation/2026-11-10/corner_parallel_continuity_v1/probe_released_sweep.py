"""원 출력 불변 재생으로 코너 후반부 미관측의 광선 상태와 기존 제한 후보를 분리한다."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path.insert(0, str(ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'))
from replay_tools import build_controller, read_events
from arena_autonomy.follow_trace import replay_call, freeze, equal_output
from arena_autonomy.released_rear_sweep import bounded_released_rear_sweep
from arena_autonomy.rear_fringe_trial import _bounded_nonreturn
import arena_autonomy.pass_release_trial as ps


def brackets(scene, raw, q, padding):
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    dirs = np.array([r.direction_xy for r in scene.rays])
    result = []
    # 처음/가장 바깥/마지막 질의의 실제 이웃을 기록한다. 일부 표본만으로 승인하지 않는다.
    selected = sorted(set([0, int(np.argmax(np.abs(q[:, 1]))), len(q)-1]))
    for i in selected:
        delta = q[i]-origins
        angles = np.arctan2(dirs[:, 0]*delta[:, 1]-dirs[:, 1]*delta[:, 0], np.sum(dirs*delta, axis=1))
        lo = int(np.argmin(np.where(angles >= -1e-10, np.maximum(angles, 0.), np.inf)))
        hi = int(np.argmin(np.where(angles <= 1e-10, np.maximum(-angles, 0.), np.inf)))
        rays = []
        for k in sorted(set([lo, hi])):
            ray = scene.rays[k]
            rays.append(dict(index=k, status=ray.status, raw_range=raw.ranges[k],
                query_radius_m=float(np.linalg.norm(delta[k])), angle_rad=angles[k], padding_m=padding[k],
                hit_xy_m=ray.hit_xy_m, origin_xy_m=ray.ray_origin_xy_m))
        result.append(dict(query=q[i], rays=rays))
    return freeze(result)


def main():
    output = A/'released_sweep_probe'
    output.mkdir(exist_ok=False)
    path = A/'run1/follow_decisions.jsonl'
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    sources = {p.relative_to(ROOT).as_posix(): sha(p) for p in
        [path, *sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]}
    original = ps.PassReleaseSelector.select
    seen, rows, counts = set(), [], Counter()

    def select(self, scene, baseline, parallel, speed, previous, dt, **kw):
        result = original(self, scene, baseline, parallel, speed, previous, dt, **kw)
        if (result[2].get('selector_reason') != 'large_motion_region_unobserved'
                or self.mode != 'RELEASED' or scene.source_first_s in seen):
            return result
        seen.add(scene.source_first_s)
        raw = kw.get('raw_scan')
        row = dict(t=scene.t_ref_s, source_s=scene.source_first_s, measured_speed_mps=speed,
            baseline_mps=baseline[0], candidates=[])
        for target in (.35, .20):
            context = {}
            region = ps.motion_region(scene, speed, target, baseline[1], previous, dt,
                self.follow_config, self.lat, context=context)
            if not context:
                row['candidates'].append(dict(target_mps=target, region=region))
                continue
            q = context['queries'][~context['free'] & ~context['masked']]
            if not len(q):
                row['candidates'].append(dict(target_mps=target, region=region))
                continue
            bounded = bounded_released_rear_sweep(scene, raw, q, context['padding'],
                self.follow_config.length_m, self.follow_config.width_m, speed, target,
                self.config.small_excursion_m)
            # 광선 자체 실패를 속도/후반부 조건과 분리하는 진단이며 출력 허가로 쓰지 않는다.
            raw_only = _bounded_nonreturn(scene, raw, q, context['padding'], dict(
                accepted=False, maximum_gap_rad=math.radians(3), minimum_neighbor_range_m=.6))
            row['candidates'].append(dict(target_mps=target, region=region,
                bounded=bounded, ray_diagnostic_only=raw_only,
                unknown_bounds=[q.min(axis=0), q.max(axis=0)],
                brackets=brackets(scene, raw, q, context['padding'])))
        rows.append(freeze(row))
        return result

    ps.PassReleaseSelector.select = select
    try:
        for event in read_events(path):
            data = event['data']
            if event['kind'] == 'header': controller, motion = build_controller(data)
            elif event['kind'] == 'reset': controller.reset()
            elif event['kind'] == 'call':
                result = freeze(replay_call(controller, data['input'], motion))
                counts['calls'] += 1
                counts['command_mismatch'] += not equal_output(result[:2], data['output'][:2])
    finally:
        ps.PassReleaseSelector.select = original
    result = dict(scope='readonly diagnosis; raw-only check does not authorize commands',
        counts=dict(counts), rows=rows, sources=sources,
        source_unchanged=all(sha(ROOT/p) == h for p, h in sources.items()))
    (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    outcomes = Counter(c.get('bounded', {}).get('reason', 'no_unknown') for r in rows for c in r['candidates'])
    rays = Counter(c.get('ray_diagnostic_only', {}).get('reason', 'no_unknown') for r in rows for c in r['candidates'])
    print(json.dumps(dict(counts=dict(counts), scans=len(rows), outcomes=outcomes, rays=rays,
        source_unchanged=result['source_unchanged'])))
    assert counts['command_mismatch'] == 0 and result['source_unchanged']


if __name__ == '__main__': main()
