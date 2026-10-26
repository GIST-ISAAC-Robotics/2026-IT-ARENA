"""run7/8 고정 입력의 도로 끝 근거를 분해한다. 제어 출력 변경·주행 없음."""
import json
import math
from pathlib import Path
import hashlib

import numpy as np
import yaml

from arena_autonomy.lateral_intent import LateralIntentConfig, ego_state, surface_check, corridor_reference
from arena_autonomy.m3_execution_trial import ExecutionFrame, straight_candidate, straight_surface_prefix
from arena_autonomy.local_scene import scene_from_scan
from analyze_grid_start_constraints import decode_ranges
from probe_grid_follow import ROOT, CAMERA, VEHICLE, json_value


def main():
    base = Path(__file__).resolve().parent
    inputs = [base/'run7/data/report.json', base/'run8/data/report.json', CAMERA, VEHICLE]
    original = {str(p.relative_to(ROOT)): p.read_bytes() for p in inputs}
    scans, outputs = [json.loads(p.read_text(encoding='utf-8')) for p in inputs[:2]]
    sensor = yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    vehicle = yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    cfg = LateralIntentConfig(enabled=True, surface_policy='two_wheel_grass',
        wheelbase_m=vehicle['drivetrain']['wheelbase_m'])
    lidar = vehicle['sensors']['lidar_2d']
    rows = []
    for case in outputs['cases']:
        if case['slot'] != 5 or case['delay'] or case['fault']:
            continue
        source = next(c for c in scans['cases'] if c['slot'] == case['slot'] and
            c['bias_m'] == case['bias'] and not c['additional_delay_s'] and not c['fault_selected'])
        calls = case['comparisons'][1]['calls']
        for row in (calls[49], calls[-1]):
            meta = row['meta']
            if 'path_points' not in meta or 'straight' not in meta.get('m3_lateral', {}).get('checks', {}):
                continue
            old = meta['m3_lateral']['checks']['straight']
            raw = source['scans'][row['frame']]
            now = row['t_s']
            stamps = raw['first_s']+np.arange(500)*.0002
            scene = scene_from_scan(decode_ranges(raw['ranges']).tolist(), angle_min=-math.pi,
                angle_increment=math.tau/500, range_min=lidar['range_min_m'], range_max=lidar['range_max_m'],
                acquired_at_s=stamps.tolist(), t_ref_s=now, sensor_xy_m=tuple(sensor['xyz_m'][:2]),
                rear_half_angle_deg=sensor['masked_rear_half_angle_deg'])
            frame = ExecutionFrame(meta['path_points'])
            candidate = straight_surface_prefix(straight_candidate(frame, ego_state(frame, cfg), cfg),
                old['surface_scope']['end_m'])
            surface = surface_check(frame, candidate, cfg, current_pose=(0., 0., 0.))
            points = np.asarray(scene.occupied_endpoints_xy_m).reshape(-1, 2)
            need = surface['s_range_needing_edge']
            detail = None
            if need:
                lo, hi = need[0]-.05, need[1]+.05
                ref = corridor_reference(frame, points, lo, hi, cfg)
                s, l = frame.project(points)
                detail = dict(s_lo=lo, s_hi=hi, reference=ref, projected_bands={})
                for side, sign in (('left', 1.), ('right', -1.)):
                    # 진단용으로 넓게 잡은 띠. 허가·주행 판정에는 사용하지 않는다.
                    take = (s >= lo) & (s <= hi) & (abs(l-sign*cfg.wall_offset_m) <= .15)
                    residual = l[take]-sign*cfg.wall_offset_m
                    detail['projected_bands'][side] = dict(points=int(take.sum()),
                        residual_min_m=float(min(residual)) if len(residual) else None,
                        residual_median_m=float(np.median(residual)) if len(residual) else None,
                        residual_max_m=float(max(residual)) if len(residual) else None)
            rows.append(dict(bias_m=case['bias'], frame=row['frame'], t_s=now,
                saved_reasons=old['reasons'], surface={k:surface[k] for k in (
                    'ok','grass_side','road_only_ok','max_body_excess_m','current_body_excess_m',
                    'edge_margin_m','s_range_needing_edge')}, detail=detail))
    report = dict(scope='stored fixed-pose surface diagnostic only; no command change', cases=rows,
        source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in original.items()},
        sources_unchanged=all((ROOT/n).read_bytes()==b for n,b in original.items()))
    output = base/'surface_reference_diagnostic.json'
    with output.open('x', encoding='utf-8') as f:
        json.dump(json_value(report), f, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(json_value(rows), ensure_ascii=False))


if __name__ == '__main__':
    main()
