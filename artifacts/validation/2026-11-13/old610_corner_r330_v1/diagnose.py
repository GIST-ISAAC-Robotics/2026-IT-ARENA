"""같은 센서 입력의 좌우 경로/저장 경로를 비교한다. 지도는 사용하지 않는다."""
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, replay_reset, restore_call, freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_path import local_path
from arena_autonomy.numeric_kernels import configure

TARGETS = (30.8, 31.14, 31.4, 38.2)


def main():
    path = A.parent/'old610_drive_r328_v1/run1/follow_decisions.jsonl'
    with path.open() as f:
        header = json.loads(next(f))['data']
        events = [json.loads(line) for line in f]
    configure(header.get('numeric_execution', 'python'))
    follow, motion = make_controller(header)
    cfg = follow.config
    rows = []
    fig, axes = plt.subplots(2, 2, figsize=(12, 10), layout='constrained')
    targets = list(TARGETS)
    for event in events:
        if event['kind'] == 'reset':
            replay_reset(follow, event['data'])
        if event['kind'] != 'call':
            continue
        call = event['data']
        ref = call['input']['reference_s']
        selected = bool(targets and ref >= targets[0]-1e-8)
        old_path = None if follow.path is None else follow.path.copy()
        old_ref, old_source = follow.path_ref, follow.path_source
        output = freeze(replay_call(follow, call['input'], motion))
        if not selected:
            continue
        targets.pop(0)
        info, scan, history = restore_call(call['input'], motion)
        scene = motion_scene(scan, history, ref, timing_verified=info['timing_verified'],
                             epoch=info['epoch'], rear_half_angle_deg=info['rear_half_angle_deg'])
        points = np.asarray(scene.occupied_endpoints_xy_m)
        rays = [r.source_index for r in scene.rays if r.status == 'return']
        age = ref-scene.source_first_s
        v = cfg.open_route_speed_mps
        horizon = max(1.8, .06+cfg.standstill_gap_m+v*(age+.1)+v**2/4)
        row = dict(time=ref, old_path_ref=old_ref, old_path_source=old_source,
                   output=output, sides={})
        ax = axes.flat[len(rows)]
        ax.scatter(points[:, 0], points[:, 1], s=4, c='#555555', label='LiDAR returns')
        for side, color in [('left', '#047bbb'), ('right', '#b6267c')]:
            candidate, detail = local_path(points, side, cfg.wall_offset_m, horizon=horizon,
                rays=rays, piecewise_corner=True, minimum_radius=cfg.wheelbase_m/math.tan(cfg.max_steering_rad),
                narrow_occluder_bridge=True)
            row['sides'][side] = {**detail, 'path': None if candidate is None else candidate.tolist()}
            if candidate is not None:
                ax.plot(candidate[:, 0], candidate[:, 1], color=color, label=side+' candidate')
        if old_path is not None and old_ref is not None:
            from arena_autonomy.scene_motion import transport_points
            try:
                carried = transport_points(old_path, history, old_ref, ref)
                ax.plot(carried[:, 0], carried[:, 1], '--', c='#d78300', label='Previous path transported')
                row['carried'] = carried.tolist()
            except ValueError as e:
                row['carried_error'] = str(e)
        ax.add_patch(plt.Rectangle((-cfg.length_m/2, -cfg.width_m/2), cfg.length_m, cfg.width_m,
                                   fc='#8298aa', alpha=.6))
        ax.arrow(0, 0, .22, 0, head_width=.035, color='black')
        ax.set(title=f"t={ref:.2f}s {output[2]['reason']}", aspect='equal', xlim=(-.8, 1.7),
               ylim=(-1.2, 1.2), xlabel='Forward X (m)', ylabel='Left Y (m)')
        ax.grid(alpha=.2)
        ax.legend(fontsize=7)
        rows.append(row)
        if not targets:
            break
    assert len(rows) == len(TARGETS)
    (A/'corner_evidence.json').write_text(json.dumps(freeze(rows), indent=2)+'\n')
    fig.savefig(A/'corner_evidence.png', dpi=150)
    print(json.dumps([dict(time=r['time'], sides={k: {n: v.get(n) for n in
        ('reason', 'wall_model', 'boundary_observed_ahead_m', 'path_anchor_error_m')} for k, v in r['sides'].items()}) for r in rows]))


if __name__ == '__main__':
    main()
