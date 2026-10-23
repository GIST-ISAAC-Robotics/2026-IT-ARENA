#!/usr/bin/env python3
"""저장 입력의 미관측 광선과 제동 외곽 대조. 운영 제어/보호 변경 없음.

unknown을 거리 상한으로 채우지 않는다. 각 취득 원점의 한 각 셀 전체를
미관측 부채꼴로 남기고, 현재 명목 차체 밖의 정곡률 외곽과 교차시킨다.
현 차체 내부를 제외하는 것은 그 공간이 free라는 뜻이 아니다.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

from arena_autonomy.follow_trace import restore_call, thaw
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_runtime import digest
from audit_corridor_sweep import required_stopping_distance, sweep_visibility
from summarize_corner_trace import summarize

ROOT = Path(__file__).resolve().parents[1]


def envelope_poses(steering, horizon, config, step=.01):
    if (not all(math.isfinite(v) for v in (steering,horizon,step))
            or not abs(steering) <= config.max_steering_rad < .5
            or not 0 < horizon <= 2.5 or not 0 < step <= .04):
        raise ValueError('recorded steering limit and bounded horizon required')
    distance = np.r_[np.arange(0.,horizon,step),horizon]
    curvature = math.tan(steering)/config.wheelbase_m
    yaw = curvature*distance
    x = distance if abs(curvature) < 1e-8 else np.sin(yaw)/curvature
    y = np.zeros_like(distance) if abs(curvature) < 1e-8 else (1-np.cos(yaw))/curvature
    return np.column_stack((distance,x-config.wheelbase_m/2+config.wheelbase_m/2*np.cos(yaw),
                            y+config.wheelbase_m/2*np.sin(yaw),yaw))


def swept_region(steering, horizon, config, margin=0., step=.01):
    """고정 곡률 연속 명목 외곽을 덮는 상계. 조향 응답/슬립 보장은 아니다."""
    if not math.isfinite(margin) or not 0 <= margin <= .5 or not 0 < step <= .04:
        raise ValueError('bounded nonnegative margin and sampling step required')
    poses = envelope_poses(steering, horizon, config, step)
    corner = np.array([[-1,-1],[-1,1],[1,1],[1,-1]]) * [config.length_m/2, config.width_m/2]
    # 후륜축 s 변화당 차체 점의 이동 상계. 가까운 표본까지 거리는 <= step/2.
    radius = math.hypot(config.length_m/2+config.wheelbase_m/2, config.width_m/2)
    bound = (1.+abs(math.tan(steering)/config.wheelbase_m)*radius)*step/2
    cells, sampled_cells = [], []
    for _, x, y, yaw in poses:
        c, s = math.cos(yaw), math.sin(yaw)
        poly = Polygon(corner @ np.array([[c,s],[-s,c]])+[x,y])
        sampled_cells.append(poly if margin == 0 else poly.buffer(margin, join_style=2))
        # 사각형 miter 팽창은 유클리드 원 팽창을 포함한다.
        cells.append(poly.buffer(margin+bound, join_style=2))
    body = box(-config.length_m/2, -config.width_m/2, config.length_m/2, config.width_m/2)
    return unary_union(cells).difference(body), bound, unary_union(sampled_cells).difference(body)


def unknown_sector(ray, half_angle, region):
    if not math.isfinite(half_angle) or not 0 < half_angle <= math.radians(1):
        raise ValueError('known <= 2 degree source cells required')
    origin = np.asarray(ray.ray_origin_xy_m)
    a,b,c,d = region.bounds
    radius = max(np.linalg.norm(np.array([x,y])-origin) for x in (a,c) for y in (b,d))+1.
    angle = math.atan2(ray.direction_xy[1], ray.direction_xy[0])
    # 삼각형의 먼 변이 검사 영역 전체보다 바깥이도록 한다. 무응답 거리 제한 아님.
    far = radius/math.cos(half_angle)
    return Polygon([origin, origin+far*np.array([math.cos(angle-half_angle), math.sin(angle-half_angle)]),
                    origin+far*np.array([math.cos(angle+half_angle), math.sin(angle+half_angle)])])


def inspect_envelope(scene, steering, distance, increment, config, margin=0.):
    region, sampling_bound, sampled_region = swept_region(steering, distance, config, margin)
    hits = []
    for ray in scene.rays:
        if ray.status == 'return':
            continue
        sector = unknown_sector(ray, increment/2, region)
        overlap = sector.intersection(region)
        if not overlap.is_empty:
            representative = overlap.representative_point()
            hits.append(dict(index=ray.source_index, status=ray.status,
                acquired_at_s=ray.acquired_at_s, intersection_area_m2=overlap.area,
                sampled_overlap_area_m2=sector.intersection(sampled_region).area,
                example_xy_m=[representative.x, representative.y],
                current_direction_deg=math.degrees(math.atan2(ray.direction_xy[1],ray.direction_xy[0]))))
    return dict(unknown_intersections=hits, unknown_intersection_count=len(hits),
        sampled_overlap_count=sum(r['sampled_overlap_area_m2'] > 1e-12 for r in hits),
        no_unknown_intersection=not hits, margin_m=margin, sampling_cover_m=sampling_bound,
        region_area_m2=region.area, region_bounds=list(region.bounds),
        unknown_filled=False, dynamic_cut_in_verified=False, actual_braking_verified=False,
        continuous_fixed_curvature_cover=True, driving_authorized=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    before = digest(args.capture)
    sources = {}
    for module in tuple(sys.modules.values()):
        file = getattr(module, '__file__', None)
        if file:
            p = Path(file).resolve()
            if p.is_relative_to(ROOT) and p.suffix == '.py':
                sources[p.relative_to(ROOT).as_posix()] = digest(p)
    p = ROOT/'tests/test_observation_envelope.py'
    sources[p.relative_to(ROOT).as_posix()] = digest(p)
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(ROOT/name, name)
    with args.capture.open() as stream:
        complete = summarize(json.loads(line) for line in stream)
    chosen, prior, seen = [], None, set()
    with args.capture.open() as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'header':
                motion = MotionConfig(**event['data']['motion_config'])
                config = FollowConfig(**event['data']['follow_config'])
            if event['kind'] != 'call':
                continue
            row = thaw(event['data'])
            if row['output'][2]['reason'] == 'follow_front_unobserved':
                key = (row['input']['scan']['sec'], row['input']['scan']['nanosec'])
                if key not in seen:
                    if not chosen and prior:
                        chosen.append(prior)
                    chosen.append(event)
                    seen.add(key)
            prior = event
    if not chosen or len(seen) > 128:
        raise ValueError('expected a bounded recorded unknown interval')
    rows = []
    for event in chosen:
        data, scan, history = restore_call(event['data']['input'], motion)
        scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                             epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        # 기록 직전각/직진/최대 좌우각의 개별 비교이며 명령 생성은 아니다.
        # 속도 0.2/현행 상한은 저장된 미래 주행이 아니라 요구 영역의 민감도다.
        cases = []
        speed = max(0., data['speed_mps'])
        age = scene.t_ref_s-scene.source_first_s
        for target in (.2, config.max_speed_mps):
            distance = required_stopping_distance(speed, target, age, config)
            if distance > 2.5:
                cases.append(dict(target=target, evaluated=False, reason='horizon_exceeded'))
                continue
            for label, steering in [('previous', data['previous_steering_rad']), ('straight',0.),
                                    ('left_limit',config.max_steering_rad), ('right_limit',-config.max_steering_rad)]:
                base = inspect_envelope(scene, steering, distance, scan.angle_increment, config)
                margin = .025+config.measurement_padding_m+config.object_motion_bound_mps*age
                padded = inspect_envelope(scene, steering, distance, scan.angle_increment, config, margin)
                depth = (sweep_visibility(scene, steering, distance, config) if abs(steering) <= .37 else
                         dict(stopping_sweep_depth_supported=None, first_unsupported=None,
                              reason='legacy_depth_audit_angle_limit'))
                cases.append(dict(label=label, target_speed_mps=target, steering_rad=steering,
                    evaluated=True, distance_m=distance, nominal=base, padded=padded,
                    sampled_depth_supported=depth['stopping_sweep_depth_supported'],
                    depth_audit_reason=depth.get('reason'),
                    first_depth_failure=depth['first_unsupported']))
        rows.append(dict(seq=event['seq'], time_s=data['reference_s'], source_s=scene.source_first_s,
            original_reason=thaw(event['data']['output'])[2]['reason'], measured_speed_mps=data['speed_mps'], cases=cases))
    result = dict(scope='unknown_sector_vs_fixed_curvature_region_only', capture_summary=complete,
        source_sha256=sources, input_sha256=before, config=asdict(config), rows=rows,
        input_unchanged=before==digest(args.capture),
        source_unchanged=all(digest(ROOT/p)==value for p,value in sources.items()), runtime_changed=False)
    (args.output/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(scenes=len(rows), unknown_scans=len(seen), input_unchanged=result['input_unchanged'],
                         source_unchanged=result['source_unchanged'])))
    return 0 if result['input_unchanged'] and result['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
