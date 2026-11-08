"""새 C1 실패의 현재 관측 경로 분석. 원시 주행/제어 소스 변경 없음."""
from pathlib import Path
import json, types
import numpy as np
from arena_autonomy.follow_trace import restore_call, motion_scene, MotionConfig, FollowConfig
from arena_autonomy.local_path import local_path
import arena_autonomy.piecewise_wall as pw

A = Path(__file__).resolve().parent
TRACE = A.parent/'g1g2_lidar_stress_pair_v1/B_c1_injection/run1/follow_decisions.jsonl'
targets = (18.38, 18.48, 18.58, 19., 25.)
snapshots = []
for line in TRACE.open():
    e = json.loads(line); d = e['data']
    if e['kind'] == 'header':
        cfg = FollowConfig(**d['follow_config']); motion = MotionConfig(**d['motion_config'])
    if e['kind'] != 'call': continue
    if len(snapshots) < len(targets) and d['input']['reference_s'] >= targets[len(snapshots)]:
        snapshots.append(d)
    if len(snapshots) == len(targets): break
original = pw.corner_path
all_rows = []; sample_cache = []
for d in snapshots:
    i, scan, history = restore_call(d['input'], motion)
    scene = motion_scene(scan, history, i['reference_s'], timing_verified=i['timing_verified'],
                         epoch=i['epoch'], rear_half_angle_deg=i['rear_half_angle_deg'])
    points = np.asarray(scene.occupied_endpoints_xy_m)
    for side in ('left', 'right'):
        rows = []
        for window in (1, 3, 5, 7):
            stats = []
            def vertices(pts, tolerance=.025):
                if window == 1: return pw._vertices(pts, tolerance)
                padded = np.pad(pts, ((window//2, window//2),(0,0)), mode='edge')
                smoothed = np.median(np.lib.stride_tricks.sliding_window_view(padded, window, axis=0),axis=-1)
                return pw._vertices(smoothed, tolerance)
            def test_corner(samples, side, offset, minimum_radius):
                v = vertices(samples)
                stats.append(dict(vertices=len(v), sample_count=len(samples)))
                sample_cache.append(dict(t=scene.t_ref_s,side=side,window=window,samples=np.asarray(samples).tolist(),vertices=v))
                fn = types.FunctionType(original.__code__,{**original.__globals__,'_vertices':vertices},original.__name__,original.__defaults__)
                return fn(samples,side,offset,minimum_radius)
            pw.corner_path = test_corner
            path, info = local_path(points, side, cfg.wall_offset_m, horizon=1.8,
                piecewise_corner=True, minimum_radius=cfg.wheelbase_m/np.tan(cfg.max_steering_rad))
            rows.append(dict(window=window,reason=info.get('reason'),model=info.get('wall_model'),
                ahead=info.get('boundary_observed_ahead_m'),anchor=info.get('path_anchor_error_m'),
                corner_attempts=stats,path=None if path is None else path.tolist()))
        all_rows.append(dict(t=scene.t_ref_s,side=side,rows=rows))
pw.corner_path = original
(A/'snapshots.json').write_text(json.dumps(dict(motion_config=motion.__dict__,follow_config=cfg.__dict__,calls=snapshots)),encoding='utf8')
(A/'samples.json').write_text(json.dumps(sample_cache),encoding='utf8')
(A/'result.json').write_text(json.dumps(all_rows,indent=2),encoding='utf8')
for entry in all_rows:
    print(entry['t'],entry['side'],[{k:v for k,v in row.items() if k!='path'} for row in entry['rows']])
