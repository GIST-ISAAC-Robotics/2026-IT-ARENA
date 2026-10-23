"""첫 실패를 수정 없이 3자 대조한다. 운영/독립 감사/최적화 후보."""
import json
import math
import numpy as np
from test_observation_depth_window import candidate, depth_support, bracket_depth, motion_scene, scan, history

rows = []
for seed in (19, 73, 1019):
    rng = np.random.default_rng(seed)
    raw = scan(.5, rng.uniform(.06, 12., 500).tolist())
    for i in rng.choice(500, 30, replace=False):
        raw.ranges[i] = math.inf
    scene = motion_scene(raw, history(.9, -1.3), .635, timing_verified=True, rear_half_angle_deg=30.)
    queries = list(rng.uniform([-.3, -.7], [2., .7], (240, 2)))
    for ray in [*scene.rays[:3], *scene.rays[248:253], *scene.rays[-3:]]:
        o, d = np.array(ray.ray_origin_xy_m), np.array(ray.direction_xy)
        queries.append(o)
        for angle in (0., -1e-11, 1e-11, -1e-9, 1e-9, -.025, .025):
            c, s = math.cos(angle), math.sin(angle)
            queries.append(o+.2*np.array([c*d[0]-s*d[1], s*d[0]+c*d[1]]))
    pad = rng.uniform(0., .2, 500)
    a, b = candidate.depth_support(scene, queries, pad), depth_support(scene, queries, pad)
    audit = bracket_depth(scene, queries, pad)
    c = np.array([r['supported'] for r in audit])
    for i in np.flatnonzero((a != b) | (a != c)):
        q = np.asarray(queries[i])
        origins = np.array([r.ray_origin_xy_m for r in scene.rays])
        dirs = np.array([r.direction_xy for r in scene.rays])
        dx, dy = q[0]-origins[:, 0], q[1]-origins[:, 1]
        cross = dirs[:, 0]*dy-dirs[:, 1]*dx
        dot = dirs[:, 0]*dx+dirs[:, 1]*dy
        angle = np.arctan2(cross, dot)
        lo = int(np.argmin(np.where(angle >= -1e-10, np.maximum(angle, 0.), np.inf)))
        hi = int(np.argmin(np.where(angle <= 1e-10, np.maximum(-angle, 0.), np.inf)))
        exact = np.flatnonzero((dx == 0) & (dy == 0)).tolist()
        rows.append(dict(seed=seed, query_index=int(i), xy=q.tolist(), candidate=bool(a[i]), runtime=bool(b[i]),
            audit=audit[i], runtime_indices=[lo, hi], runtime_angles=angle[[lo, hi]].tolist(),
            exact_origin_indices=exact, exact_origin_dot=dot[exact].tolist(),
            exact_origin_cross=cross[exact].tolist(), exact_origin_angle=angle[exact].tolist()))
    print(json.dumps(dict(seed=seed, candidate_runtime_differences=int(np.count_nonzero(a != b)),
        candidate_audit_differences=int(np.count_nonzero(a != c)))))
print(json.dumps(rows, indent=2))
