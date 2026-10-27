"""run1 이동 영역 거절의 센서 기하를 동일 재생 중 분해한다. 판단/반환을 바꾸지 않는다."""
import json
import math
from collections import Counter
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
for part in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts'):
    sys.path.insert(0, str(ROOT/part))

import numpy as np
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.follow_trace import replay_events
from arena_autonomy.scan_topology import cyclic_count, adjacent_indices


def audit(scene, speed, target, steering, previous, dt, cfg, lat):
    distance = ps.mx.envelope_distance(max(abs(speed), abs(target)), scene.t_ref_s-scene.source_first_s,
                                       dt, ps.mx.ExecutionConfig())
    queries = []
    for angle in sorted({float(steering), float(previous)}):
        poses = ps.extrema_arc(angle, max(distance, .01), lat)
        q, _ = ps.mx.body_queries(scene, poses, np.full(len(poses), angle), lat)
        queries.append(q.reshape(-1, 2))
    q = np.unique(np.vstack(queries), axis=0)
    q = q[(np.abs(q[:, 0]) > cfg.length_m/2+1e-9) | (np.abs(q[:, 1]) > cfg.width_m/2+1e-9)]
    pad = ps.mx.ray_padding(scene, lat, .5)
    free = ps.depth_support(scene, q, pad)
    masked = np.zeros(len(q), dtype=bool)
    masked[~free] = ps.mx.masked_only(scene, q[~free], pad)
    bad = q[~free & ~masked]
    out = {'region': ps.motion_region(scene, speed, target, steering, previous, dt, cfg, lat),
           'bad_bbox': [bad.min(axis=0).tolist(), bad.max(axis=0).tolist()] if len(bad) else None,
           'counts': {}, 'examples': []}
    origins = np.asarray([r.ray_origin_xy_m for r in scene.rays])
    directions = np.asarray([r.direction_xy for r in scene.rays])
    indices = np.asarray([r.source_index for r in scene.rays])
    depth = np.asarray([math.dist(r.ray_origin_xy_m, r.hit_xy_m) if r.status == 'return' else -1. for r in scene.rays])
    counter = Counter()
    for point in bad:
        delta = point-origins
        angles = np.arctan2(directions[:, 0]*delta[:, 1]-directions[:, 1]*delta[:, 0],
                            np.sum(directions*delta, axis=1))
        lo = int(np.argmin(np.where(angles >= -1e-10, np.maximum(angles, 0.), np.inf)))
        hi = int(np.argmin(np.where(angles <= 1e-10, np.maximum(-angles, 0.), np.inf)))
        ids = np.array([lo, hi])
        radius = np.linalg.norm(delta[ids], axis=1)
        checks = {'origin_overlap': bool(np.any(np.sum(delta**2, axis=1) <= 1e-18)),
                  'angle_gap': bool(abs(angles[lo])+abs(angles[hi]) > .025),
                  'nonadjacent': not bool(adjacent_indices(indices[[lo]], indices[[hi]], cycle_count=cyclic_count(scene, None))[0]),
                  'nonreturn': any(scene.rays[i].status != 'return' for i in ids),
                  'depth_with_padding': bool(np.any(depth[ids]-radius-pad[ids] <= 0))}
        failed = [k for k, v in checks.items() if v]
        counter.update(failed or ['other'])
        if len(out['examples']) < 8:
            out['examples'].append({'point': point.tolist(), 'causes': failed, 'rays': [int(indices[i]) for i in ids],
                                    'status': [scene.rays[i].status for i in ids],
                                    'angle_gap_rad': abs(float(angles[lo]))+abs(float(angles[hi])),
                                    'range_minus_query_m': (depth[ids]-radius).tolist(), 'padding_m': pad[ids].tolist(),
                                    'hits': [scene.rays[i].hit_xy_m for i in ids]})
    out['counts'] = dict(counter)
    if len(bad) != out['region']['unobserved']:
        raise AssertionError('instrumentation does not match runtime query classification')
    return out


rows, seen = [], set()
original = ps.PassReleaseSelector.select


def instrumented(self, scene, baseline, parallel, speed, previous, dt):
    out = original(self, scene, baseline, parallel, speed, previous, dt)
    detail = out[2]
    key = int(scene.t_ref_s)
    if (key in (31, 32, 35, 38, 40, 43) and key not in seen
            and detail.get('selector_reason') == 'guard_curve_rejected_large_motion_region_unobserved'):
        seen.add(key)
        g = detail['selector_guard_candidate']
        variants = {str(v): audit(scene, speed, min(baseline[0], v), baseline[1], previous, dt,
                                  self.follow_config, self.lat) for v in (.35, .20, .10, 0.)}
        rows.append({'t': scene.t_ref_s, 'measured_speed': speed, 'previous': previous, 'baseline': baseline[:2],
                     'candidate': g, 'caps': variants})
    return out


if __name__ == '__main__':
    path = HERE/'run1_region_audit.json'
    if path.exists():
        raise FileExistsError(path)
    ps.PassReleaseSelector.select = instrumented
    with (HERE/'run1_corner/follow_decisions.jsonl').open(encoding='utf-8') as stream:
        report = replay_events(json.loads(line) for line in stream)
    ps.PassReleaseSelector.select = original
    result = {'scope': 'same-output replay, sensor query geometry; lower caps are counterfactuals, not adopted commands',
              'replay': {k: report[k] for k in ('calls', 'matched', 'passed')}, 'samples': rows}
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'replay': result['replay'], 'samples': [
        {'t': r['t'], 'speed': r['measured_speed'], 'caps': {k: {'counts': v['counts'], 'bbox': v['bad_bbox'],
           'unobserved': v['region']['unobserved']} for k, v in r['caps'].items()}} for r in rows]}, ensure_ascii=False))
