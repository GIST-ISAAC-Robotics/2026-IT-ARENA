"""원 로그의 점유 원인만 Windows 공개 Python 기하 경로로 대조한다.

전체 evaluator/native 동일 재생이나 실제 객체 식별의 증명은 아니다.
"""
from pathlib import Path
import collections
import hashlib
import json
import math
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for package in ('arena_autonomy', 'arena_vehicle_interface'):
    sys.path.insert(0, str(HERE/'frozen_source/src'/package))
import numpy as np
from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import _measurement_transforms
from arena_autonomy.local_scene import scene_from_measurements
from arena_autonomy.bypass_entry_trial import current_outer_wall_padding
from arena_autonomy.local_path import swept_limit, braking_speed


def restore_scene(data, motion):
    d, scan, history = restore_call(data, motion)
    transforms, cfg, epoch = _measurement_transforms(scan, history, d['reference_s'],
        timing_verified=d['timing_verified'], motion_epoch=d['epoch'])
    scene = scene_from_measurements(scan.ranges_m, angles_rad=scan.angles_rad,
        acquired_at_s=scan.acquired_at_s, t_ref_s=d['reference_s'],
        range_min=scan.range_min_m, range_max=scan.range_max_m,
        source_frame_id=scan.source_frame_id, source_epoch=epoch,
        sensor_xy_m=(cfg.lidar_x_m, cfg.lidar_y_m), sensor_yaw_rad=cfg.lidar_yaw_rad,
        acquisition_to_ref_se2=transforms.tolist(),
        rear_half_angle_deg=d['rear_half_angle_deg'], preserve_sampling=True)
    return d, scan, history, scene


def first_hit(points, padding, angle, cfg, returns):
    curvature = math.tan(angle)/cfg.wheelbase_m
    for distance in np.arange(0., 2.501, .04):
        yaw = curvature*distance
        c, s = math.cos(yaw), math.sin(yaw)
        rx, ry = ((distance-cfg.wheelbase_m/2, 0.) if abs(curvature)<1e-5
                  else (s/curvature-cfg.wheelbase_m/2, (1-c)/curvature))
        dx, dy = points[:, 0]-rx, points[:, 1]-ry
        x, y = c*dx+s*dy-cfg.wheelbase_m/2, -s*dx+c*dy
        hits = np.flatnonzero((abs(x)<=cfg.length_m/2+padding)&(abs(y)<=cfg.width_m/2+padding))
        if len(hits):
            return {'first_sample_progress_m': float(distance),
                'reported_clearance_m': max(0.,float(distance)-.04),
                'points': [{'ray_index': returns[i].source_index,
                    'xy_body_m': points[i].tolist(), 'padding_m': float(padding[i]),
                    'acquired_at_s': returns[i].acquired_at_s,
                    'inside_unpadded_rectangle_at_sample': bool(abs(x[i])<=cfg.length_m/2 and abs(y[i])<=cfg.width_m/2)}
                    for i in hits]}
    return {'reported_clearance_m': 2.5, 'points': []}


def main():
    trace = ROOT/'run1/follow_decisions.jsonl'
    rows = [json.loads(line) for line in trace.read_text(encoding='utf8').splitlines()]
    header = next(r['data']['header'] for r in rows if r['kind']=='runtime_evaluator_header')
    motion, cfg = MotionConfig(**header['motion_config']), FollowConfig(**header['follow_config'])
    calls = [r for r in rows if r['kind']=='call']
    results=[]
    for seq in (560, 563, 650, 740):
        row = next(r for r in calls if r['seq']==seq)
        d, scan, history, scene = restore_scene(row['data']['input'], motion)
        output = row['data']['output']; entry = output[2]['moving_pass']['entry']
        candidate = entry['candidates'][0]; side=candidate['current_outer_wall']
        padding, evidence = current_outer_wall_padding(scene,cfg,side,entry['reference']['walls'][side])
        points = np.asarray(scene.occupied_endpoints_xy_m).reshape(-1,2)
        returns = [r for r in scene.rays if r.status=='return']
        angles = {'candidate':candidate['steering_rad'],'previous':d['previous_steering_rad']}
        hits = {key:first_hit(points,padding,angle,cfg,returns) for key,angle in angles.items()}
        clearance=min(swept_limit(points, angle,cfg.wheelbase_m,horizon=2.5,
            margin=padding,length=cfg.length_m,width=cfg.width_m) for angle in angles.values())
        results.append({'seq':seq, 'reference_s':d['reference_s'],'speed_mps':d['speed_mps'],
            'original_command':output[:2], 'reason':output[2]['reason'],
            'candidate_reason':candidate['reason'], 'recorded_clearance_m':candidate['command_clearance_m'],
            'python_clearance_m':clearance, 'python_braking_speed_mps':braking_speed(clearance,scene.t_ref_s-scene.source_first_s),
            'original_motion_region':candidate['motion_region'], 'hit':hits,
            'target':output[2]['moving_pass']['target'],
            'entry_phase':entry['entry_phase'], 'wall_heading_rad':math.atan(entry['reference']['walls'][side]['slope']),
            'outer_padding':evidence})
    summary={'scope':'원 저장 scan/history를 공개 Python scene 생성과 기존 swept 식으로 재계산; native 전체 재생 아님',
        'source_archive_sha256':hashlib.sha256((ROOT/'sources.zip').read_bytes()).hexdigest(),
        'source_trace_sha256':hashlib.sha256(trace.read_bytes()).hexdigest(),
        'kind_counts':dict(collections.Counter(r['kind'] for r in rows)),
        'plan_reason_counts':dict(collections.Counter(r['data']['output'][2].get('reason') for r in calls)),
        'unavailable_candidate_reason_counts':dict(collections.Counter(c.get('reason') for r in calls
            if r['data']['output'][2].get('reason')=='moving_pass_current_path_unavailable'
            for c in r['data']['output'][2]['moving_pass']['entry']['candidates'])),
        'footer':next(r['data'] for r in rows if r['kind']=='footer'), 'representatives':results}
    (HERE/'failure_geometry.json').write_text(json.dumps(freeze(summary),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    for r in results:
        print(r['seq'],r['reference_s'],r['reason'],r['recorded_clearance_m'],r['python_clearance_m'],
            {k:[(p['ray_index'],p['xy_body_m'],p['padding_m']) for p in v['points']] for k,v in r['hit'].items()})


if __name__=='__main__':
    main()
