"""r544 최초 정체의 저장 센서 입력만 재계산한다. ROS/차량 실행 없음."""
import json
import sys
from pathlib import Path
import numpy as np

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy import current_wall_reacquire as candidate

P = ROOT/'artifacts/validation/2026-11-14/measured_steering_lap_r544_attempt2_v1/stall_extract'
if len(sys.argv) > 2:
    P = ROOT/sys.argv[2]
header = json.loads((P/'header.json').read_text())['record']['data']
owner, motion = make_controller(header)
original_path, original_pursue = candidate.local_path, candidate.pursue_path
results = []
for case in ('last_positive', 'first_zero_discarded', 'first_zero_candidate'):
    for phase in ('compute', 'current'):
        path = P/f'{case}_{phase}_input.json'
        if not path.exists():
            continue
        data, scan, history = restore_call(json.loads(path.read_text()), motion)
        scene = motion_scene(scan, history, data['reference_s'],
            timing_verified=data['timing_verified'], epoch=data['epoch'],
            rear_half_angle_deg=data['rear_half_angle_deg'],
            timing_policy=data.get('timing_policy', 'verified_only'))
        row = dict(case=case, phase=phase, reference_s=data['reference_s'],
            speed_mps=data['speed_mps'], previous=data['previous_steering_rad'],
            dt=data['dt_s'])
        def traced_path(*args, **kwargs):
            try:
                p, boundary = original_path(*args, **kwargs)
                row['boundary'] = freeze(boundary)
                row['path'] = None if p is None else p.tolist()
                return p, boundary
            except Exception as exc:
                row['path_error'] = str(exc)
                raise
        def traced_pursue(p, *args, **kwargs):
            rear = p.copy(); rear[:, 0] += owner.config.wheelbase_m/2
            distances = np.linalg.norm(rear, axis=1)
            nearest = int(np.argmin(distances))
            ahead = np.flatnonzero((rear[:, 0] > .10) & (np.arange(len(p)) >= nearest))
            row['pursue_geometry'] = dict(nearest=nearest, ahead_count=len(ahead),
                max_ahead_radius_m=float(max(distances[ahead])) if len(ahead) else None,
                remaining_arc_m=float(np.linalg.norm(np.diff(p[nearest:], axis=0), axis=1).sum()))
            try:
                out = original_pursue(p, *args, **kwargs)
                row['pursue_result'] = freeze(out)
                return out
            except Exception as exc:
                row['pursue_error'] = str(exc)
                raise
        candidate.local_path, candidate.pursue_path = traced_path, traced_pursue
        output = candidate.current_wall_candidate(scene, data['speed_mps'], data['side'],
            data['previous_steering_rad'], data['dt_s'], owner.config)
        row['output'] = freeze(output)
        row['occupied_points'] = np.asarray(scene.occupied_endpoints_xy_m).tolist()
        results.append(row)
        print(case, phase, 'output', None if output is None else output[:2],
              'path_error', row.get('path_error'), 'pursue_error', row.get('pursue_error'),
              'geometry', row.get('pursue_geometry'),
              'boundary_reason', row.get('boundary', {}).get('reason'))
candidate.local_path, candidate.pursue_path = original_path, original_pursue
(A/(sys.argv[1] if len(sys.argv) > 1 else 'diagnostic.json')).write_text(
    json.dumps(freeze(results), indent=2, allow_nan=False), encoding='utf8')
