"""짧은 관측 경로의 저장 입력 비교. 운영 제어기에 연결하지 않는 분석 도구."""
import argparse
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for part in ('src/arena_autonomy', 'src/arena_vehicle_interface'):
    sys.path.insert(0, str(ROOT / part))

from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig, pursue_path
from arena_autonomy.local_path import local_path, braking_speed, swept_limit
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.observation_support import sweep_support
from arena_autonomy.scene_motion import motion_scene


def prefix_geometry(points, side, config):
    """반대 경로와 일치하는 첫 구간 끝에서 선호 경로를 자르는 기하 후보.

    반대 경로는 대조에만 쓰며 그 먼 끝으로 연장하지 않는다. 편측 지지만
    있는 시작 부분은 차체 길이 이내, 공동 지지는 기존 비교와 같은 0.25 m다.
    이는 시험 가정이며 벽 정체·자유 공간·차량 주행 허가가 아니다.
    """
    base = dict(supported=False, reason='invalid_input', path=None,
                runtime_connected=False, driving_authorized=False,
                memory_refresh_authorized=False, wall_identity_certified=False)
    try:
        p = np.asarray(points, dtype=float)
    except (TypeError, ValueError):
        return base
    if side not in ('left', 'right') or p.ndim != 2 or p.shape[1] != 2 or not np.all(np.isfinite(p)):
        return base
    own, info = local_path(p, side, config.wall_offset_m)
    other, other_info = local_path(p, 'left' if side == 'right' else 'right', config.wall_offset_m)
    base.update(preferred_info=info, opposite_info=other_info)
    if own is None or other is None:
        return {**base, 'reason':'missing_path'}
    if info['boundary_observed_ahead_m'] >= config.boundary_min_ahead_m:
        return {**base, 'reason':'not_a_short_boundary'}
    if abs(info['wall_distance_m']-config.wall_offset_m) > config.boundary_anchor_error_m:
        return {**base, 'reason':'preferred_anchor_unconfirmed'}
    if other_info['boundary_observed_ahead_m'] < config.boundary_min_ahead_m:
        return {**base, 'reason':'opposite_reference_short'}
    nearest = int(np.argmin(np.linalg.norm(own, axis=1)))
    if np.linalg.norm(own[nearest]) > config.boundary_anchor_error_m:
        return {**base, 'reason':'path_origin_displaced'}
    prefix = own[nearest:]
    if len(prefix) < 6 or prefix[1, 0] <= prefix[0, 0]:
        return {**base, 'reason':'path_not_forward'}
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(prefix, axis=0), axis=1))]
    distances = np.linalg.norm(prefix[:, None]-other[None], axis=2)
    selected = np.argmin(distances, axis=1)
    close = (np.min(distances, axis=1) <= config.boundary_agreement_m) & (prefix[:, 0] >= .05)
    starts = np.flatnonzero(close & np.r_[True, ~close[:-1]])
    ends = np.flatnonzero(close & np.r_[~close[1:], True])+1
    # 두 번째 섬으로 넘어가며 중간의 형상 불일치를 덮지 않는다.
    if not len(starts):
        return {**base, 'reason':'no_common_prefix'}
    first, end = int(starts[0]), int(ends[0])
    if (arc[first] > config.length_m or end-first < 5 or arc[end-1]-arc[first] < .25
            or np.any(np.diff(selected[first:end]) < 0)):
        return {**base, 'reason':'insufficient_contiguous_common_prefix'}
    return {**base, 'supported':True, 'reason':'short_observed_prefix_geometry_only',
            'path':own[:nearest+end].tolist(),
            'shared_span_m':float(arc[end-1]-arc[first]),
            'one_sided_lead_in_m':float(arc[first]),
            'retained_ahead_m':float(arc[end-1]),
            'max_common_distance_m':float(np.max(np.min(distances[first:end],axis=1))),
            'opposite_path_used_for_steering':False, 'unknown_filled':False}


def compare_call(data, motion, config):
    d, scan, history = restore_call(data['input'], motion)
    scene = motion_scene(scan, history, d['reference_s'], timing_verified=d['timing_verified'],
                         epoch=d['epoch'], rear_half_angle_deg=d['rear_half_angle_deg'])
    points = np.asarray(scene.occupied_endpoints_xy_m).reshape(-1,2)
    geometry = prefix_geometry(points,d['side'],config)
    result = dict(t_s=d['reference_s'],baseline_speed_mps=data['output'][0],
                  baseline_reason=data['output'][2]['reason'],geometry=geometry,
                  positive=False, runtime_connected=False, driving_authorized=False)
    if not geometry['supported']:
        return result
    age = scene.t_ref_s-scene.source_first_s if scene.source_first_s is not None else math.inf
    front = [r for r in scene.rays if abs(math.atan2(r.direction_xy[1],r.direction_xy[0])) <= math.radians(100)]
    if not 0 <= age < config.memory_ttl_s or not front or any(r.status != 'return' for r in front):
        return {**result,'reason':'stale_or_unobserved'}
    if d['speed_mps'] < 0:
        return {**result,'reason':'reverse_motion'}
    # 기존 GUARD와 같은 저속 상한. 시간 확인·독립 노드 허가는 아직 연결하지 않는다.
    candidate_config = replace(config,max_speed_mps=min(.35,config.max_speed_mps))
    try:
        cap, desired, ahead = pursue_path(geometry['path'],d['speed_mps'],candidate_config,age)
    except MotionUnavailable as error:
        return {**result,'reason':'pursuit_unavailable','error':str(error)}
    previous=d['previous_steering_rad'];dt=d['dt_s']
    steering=float(np.clip(desired,previous-3*dt,previous+3*dt))
    padding=per_ray_padding(scene,.025+config.measurement_padding_m,config.object_motion_bound_mps)
    clearance=min(swept_limit(points,angle,config.wheelbase_m,horizon=2.5,margin=padding,
                             length=config.length_m,width=config.width_m) for angle in (steering,previous))
    available=max(0.,clearance-config.standstill_gap_m)
    # 관측한 경로를 차체 전단·측정 여유까지 모두 쓰지 않도록 예산을 차감한다.
    path_budget=max(0.,ahead-config.length_m/2-config.measurement_padding_m)
    target=min(cap,braking_speed(path_budget,age),braking_speed(available,age),available/config.headway_s)
    observation=sweep_support([scene],d['speed_mps'],target,steering,previous,config)
    return {**result,'reason':'current_scan_comparison_only','steering_rad':steering,
            'proposed_speed_mps':target,'clearance_m':clearance,'path_budget_m':path_budget,
            'observation':observation,'all_current_occupancy_retained':True,
            'positive':bool(target>0 and clearance>=config.resume_clearance_m and observation['supported'])}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    rows=[]
    with args.trace.open() as file:
        header=json.loads(next(file))['data']
        config=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
        for line in file:
            row=json.loads(line)
            if row['kind']!='call':continue
            stamp=row['data']['input']['reference_s']
            if any(abs(stamp-t)<1e-7 for t in (113.4,113.62,114.,120.)):
                rows.append(compare_call(row['data'],motion,config))
    if len(rows)!=4:raise ValueError('expected all four fixed diagnostic timestamps')
    paths=[Path(__file__),args.trace,ROOT/'src/arena_autonomy/arena_autonomy/local_path.py',
           ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py',ROOT/'src/arena_autonomy/arena_autonomy/observation_support.py']
    report=dict(scope='stateless saved input shadow; no command, temporal/reacquisition or closed-loop proof',
                rows=rows,source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
    (args.output/'report.json').write_text(json.dumps(freeze(report),ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(freeze([dict(t_s=r['t_s'],geometry=r['geometry']['reason'],positive=r['positive'],
        speed=r.get('proposed_speed_mps'),reason=r.get('reason'),observation=r.get('observation',{}).get('reason')) for r in rows])))


if __name__=='__main__':main()
