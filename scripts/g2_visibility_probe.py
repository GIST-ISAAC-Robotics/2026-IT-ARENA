#!/usr/bin/env python3
"""보존된 G2 운동학 궤적의 합성 순차 광선/가림 증거. 주행 허가 없음.

정답 box와 ego pose는 평가에만 사용한다. 실제 엔코더/IMU 보정 검증 아님.
"""
import argparse
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import zipfile

from arena_autonomy.local_scene import scene_from_scan
from g0_geometry import Footprint, TruthBox, ray_box_distance
from g2_geometry_probe import ROOT
from g2_tracking_probe import bicycle_step
from g2_ray_evidence import QueryBox, query_evidence


def pose_at(rows, t):
    times = [row['t_s'] for row in rows]
    if not times or t < times[0] or t > times[-1]+.02:
        raise ValueError('trajectory does not cover query time')
    row = rows[max(0, bisect_right(times, t)-1)]
    remainder = t-row['t_s']
    rear = (row['rear_x_m'], row['rear_y_m'], row['yaw_rad'])
    if remainder > 0:
        rear = bicycle_step(*rear, row['applied_speed_mps'], row['applied_steering_rad'], remainder, .135)
    x,y,yaw = rear
    return x+.0675*math.cos(yaw), y+.0675*math.sin(yaw), yaw


def local_pose(world_pose, reference):
    x,y,yaw = world_pose
    rx,ry,ryaw = reference
    dx,dy = x-rx, y-ry
    c,s = math.cos(ryaw), math.sin(ryaw)
    return c*dx+s*dy, -s*dx+c*dy, yaw-ryaw


def sequential_scene(rows, t_ref, target_height, *, target_y=.24, rear_half_angle_deg=30., extra_boxes=()):
    """추가 인수는 평가 장면 대조용이다. 기존 10/13 기본 입력은 유지한다."""
    if not math.isfinite(target_y) or not 0 <= rear_half_angle_deg < 180:
        raise ValueError('finite target position and bounded mask required')
    # G0 폐쇄 직선 시험실. 공식 전체 코스도 아니고 상대 차량 동역학도 아니다.
    boxes = [TruthBox(2., sign*.45, .4, Footprint(8., .05), label='wall') for sign in (-1, 1)]
    boxes += [TruthBox(x, 0, .4, Footprint(.05, .95), label='wall') for x in (-2.025, 6.025)]
    boxes += [TruthBox(1.5, target_y, target_height, label='target')]
    boxes += list(extra_boxes)
    count, step = 500, .1/500
    first = t_ref-(count-1)*step
    stamps = [first+i*step for i in range(count)]
    t_ref = stamps[-1]
    reference = pose_at(rows, t_ref)
    transforms, ranges, labels = [], [], []
    for i, stamp in enumerate(stamps):
        pose = pose_at(rows, stamp)
        x,y,yaw = pose
        angle = -math.pi+i*math.tau/count
        origin = (x+.06*math.cos(yaw), y+.06*math.sin(yaw), .095)
        direction = (math.cos(angle+yaw), math.sin(angle+yaw), 0.)
        hits = [(d, box.label) for box in boxes
                if (d := ray_box_distance(origin, direction, box, 0., 12.)) is not None]
        distance, label = min(hits, default=(math.inf, None))
        ranges.append(distance if distance >= .05 else math.nan)
        labels.append(label)
        transforms.append(local_pose(pose, reference))
    scene = scene_from_scan(ranges, angle_min=-math.pi, angle_increment=math.tau/count,
                            range_min=.05, range_max=12., acquired_at_s=stamps, t_ref_s=t_ref,
                            sensor_xy_m=(.06, 0.), acquisition_to_ref_se2=transforms,
                            rear_half_angle_deg=rear_half_angle_deg)
    return scene, labels, reference


def inspect_stage(rows, stage_x, height):
    row = min(rows, key=lambda r:abs(r['rear_x_m']-stage_x))
    scene, labels, reference = sequential_scene(rows, row['t_s'], height)
    observations = dict(target_return_rays=sum(label == 'target' and ray.status == 'return'
                                             for label,ray in zip(labels,scene.rays)),
                        target_masked_rays=sum(label == 'target' and ray.status == 'masked'
                                             for label,ray in zip(labels,scene.rays)))
    queries = []
    query_poses = [('target_region', (1.5, .24, 0.))]
    for delta in (.5, 1.):
        if row['t_s']+delta <= rows[-1]['t_s']+.02:
            query_poses.append((f'body_after_{delta}s', pose_at(rows,row['t_s']+delta)))
    query_poses.append(('final_recorded_body', pose_at(rows,rows[-1]['t_s'])))
    for name, pose in query_poses:
        local = local_pose(pose, reference)
        evidence = query_evidence(scene,QueryBox(*local),now_s=scene.t_ref_s)
        # 광선별 상세 증거를 출력에 남긴다. 합계만으로 자유 영역을 만들지 않는다.
        queries.append(dict(name=name, query_pose_base=list(local), **evidence))
    return dict(stage_rear_x_m=stage_x, t_ref_s=scene.t_ref_s, target_height_m=height,
                source_first_s=scene.source_first_s, source_last_s=scene.source_last_s,
                source_stamp_preserved=scene.source_last_s-scene.source_first_s > .099,
                truth_transport_evaluation_only=True, scan_samples=len(scene.rays),
                **observations, queries=queries, driving_authorized=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    output=args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/validation'):
        parser.error('new validation subdirectory required')
    trace_path=args.trace.resolve()
    data=trace_path.read_bytes()
    rows=json.loads(data)
    keys=('t_s','rear_x_m','rear_y_m','yaw_rad','applied_speed_mps','applied_steering_rad')
    if (len(rows)<2 or any(not all(isinstance(row.get(k),(int,float)) and not isinstance(row[k],bool)
                                  and math.isfinite(row[k]) for k in keys) for row in rows)
            or any(abs(b['t_s']-a['t_s']-.02)>1e-9 for a,b in zip(rows,rows[1:]))):
        raise ValueError('expected finite 50 Hz stored tracking trajectory')
    output.mkdir(parents=True,exist_ok=False)
    names=['scripts/g2_visibility_probe.py','scripts/g2_ray_evidence.py','scripts/g2_tracking_probe.py',
           'scripts/g2_geometry_probe.py','scripts/g0_geometry.py','tests/test_g2_ray_evidence.py',
           'tests/test_g2_visibility_probe.py','src/arena_autonomy/arena_autonomy/local_scene.py']
    raw={name:(ROOT/name).read_bytes() for name in names}
    hashes={name:hashlib.sha256(value).hexdigest() for name,value in raw.items()}
    with zipfile.ZipFile(output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as bundle:
        for name,value in raw.items():
            bundle.writestr(name,value)
    stages=[inspect_stage(rows,x,height) for height in (.12,.06) for x in (.4,1.,1.5,1.85,2.15,2.6)]
    unchanged=all(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==value for name,value in hashes.items())
    input_unchanged=trace_path.read_bytes()==data
    report=dict(scope='synthetic_sequential_ray_evidence_on_preserved_kinematic_trajectory',
                stages=stages,source_sha256=hashes,sources_unchanged=unchanged,input_unchanged=input_unchanged,
                input_trace=str(trace_path),input_sha256=hashlib.sha256(data).hexdigest(),
                driving_authorized=False,free_area_certified=False,
                limits=['Known boxes/poses are evaluation truth, not controller inputs.',
                        'No noise, dynamic target, real encoder/IMU transport or Gazebo ray rendering.',
                        'Individual clear rays do not certify the volume between rays or below the scan plane.',
                        'Future query poses are from the stored reference test, not sensor-derived predictions.'])
    (output/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    print(json.dumps(dict(stages=len(stages),sources_unchanged=unchanged,input_unchanged=input_unchanged,
                         driving_authorized=False)))
    return 0 if unchanged and input_unchanged else 1


if __name__=='__main__':
    raise SystemExit(main())
