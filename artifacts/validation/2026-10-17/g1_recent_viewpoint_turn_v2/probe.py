"""평가 전용 양방향 정적 시점 대조: v1의 양의 조향뿐 아니라 반대 조향도 검사.

보존 월드/정답 자세는 가상 센서 생성기에서만 사용한다. 실행 제어기에 연결하지
않으며 이동 중 센서/동역학·노면·도달 가능성·실물 증거가 아니다.
"""
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

from arena_autonomy.corridor_roles import role_aware_evidence
from arena_autonomy.follow_trace import freeze, restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.observation_support import sweep_support
from diagnose_front_unknown import digest, pose, wall_geometry
from g0_geometry import ray_box_distance
from probe_observable_steering import path_deviation

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
RUN = ROOT/'artifacts/validation/2026-10-17/g1_recent_solo_v1/run'


def main():
    assert not (OUT/'report.json').exists()
    selected = None
    for line in (RUN/'follow_decisions.jsonl').open():
        e = json.loads(line)
        if e['kind'] == 'header':
            cfg = FollowConfig(**e['data']['follow_config'])
            motion = MotionConfig(**e['data']['motion_config'])
        if e['kind'] == 'call' and e['seq'] == 3501:
            selected = e
            break
    assert selected is not None
    data, raw, _ = restore_call(selected['data']['input'], motion)
    poses = np.array([json.loads(line) for line in (RUN/'ego_poses.jsonl').open()])
    nearest = int(np.argmin(abs(poses[:, 0]-data['reference_s'])))
    t, x, y, yaw = poses[nearest]
    assert abs(t-data['reference_s']) <= .006
    vehicle = ET.parse(RUN/'runtime_vehicle.sdf').find('model')
    sensor, chassis = pose(vehicle.find("link[@name='lidar_link']")), pose(vehicle.find("link[@name='chassis']"))
    runtime_report = json.loads((RUN/'report.json').read_text())
    z = runtime_report['last_observation']['dynamics']['truth_z_m']+sensor[2]-chassis[2]
    assert abs(sensor[0]-motion.lidar_x_m) < 1e-9 and abs(sensor[1]-motion.lidar_y_m) < 1e-9
    boxes, _, meshes = wall_geometry(RUN/'runtime_world.sdf')
    inputs = [RUN/n for n in ('follow_decisions.jsonl', 'ego_poses.jsonl', 'runtime_vehicle.sdf', 'runtime_world.sdf', 'report.json')]+meshes
    hashes = {str(p): digest(p) for p in inputs}
    sources = {}
    for module in tuple(sys.modules.values()):
        filename = getattr(module, '__file__', None)
        if filename:
            p = Path(filename).resolve()
            if p.is_relative_to(ROOT) and p.suffix == '.py':
                sources[p.relative_to(ROOT).as_posix()] = digest(p)
    with zipfile.ZipFile(OUT/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(ROOT/name, name)
    c, s = math.cos(yaw), math.sin(yaw)
    front = [i for i in range(500) if abs(raw.angle_min+i*raw.angle_increment) <= math.radians(100)]
    recorded_missing = [i for i in front if not math.isfinite(raw.ranges[i])]
    rows = []
    # 원래 관측 나이/점당 취득 시각 폭을 유지하되, 각 시점은 따로 정지해 있다고 가정.
    age = data['reference_s']-(raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9)
    stamps = [.5+i*raw.time_increment for i in range(500)]
    for shift in (0., .025, .05, .10):
        origin = (x+c*(sensor[0]+shift)-s*sensor[1], y+s*(sensor[0]+shift)+c*sensor[1], z)
        ranges = []
        for i in range(500):
            a = yaw+sensor[5]+raw.angle_min+i*raw.angle_increment
            direction = (math.cos(a), math.sin(a), 0.)
            hit = min((d for box in boxes if (d := ray_box_distance(origin, direction, box, 0., 100.)) is not None), default=math.inf)
            ranges.append(float(hit) if raw.range_min <= hit <= raw.range_max else math.inf)
        scene = scene_from_scan(ranges, angle_min=raw.angle_min, angle_increment=raw.angle_increment,
            range_min=raw.range_min, range_max=raw.range_max, acquired_at_s=stamps, t_ref_s=.5+age,
            sensor_xy_m=(motion.lidar_x_m, motion.lidar_y_m), sensor_yaw_rad=motion.lidar_yaw_rad,
            rear_half_angle_deg=data['rear_half_angle_deg'])
        geometry = role_aware_evidence(scene.occupied_endpoints_xy_m, data['side'], cfg)
        cases = []
        for angle in (-.18, -.12, -.06, -.03, 0., .03, .06, .12, .18):
            support = sweep_support([scene], 0., .2, angle, 0., cfg,
                protected_sensor_pose=(motion.lidar_x_m, motion.lidar_y_m, motion.lidar_yaw_rad))
            distance = support.get('required_distance_m')
            agreement = path_deviation(geometry.get('path'), angle, distance, cfg) if distance else None
            cases.append(dict(steering_rad=angle, one_tick_rate_possible=abs(angle) <= .06,
                observation=support, path=agreement))
        finite_pairs = [i for i in front if math.isfinite(ranges[i]) and math.isfinite(raw.ranges[i])]
        differences = [abs(ranges[i]-raw.ranges[i]) for i in finite_pairs] if shift == 0 else []
        rows.append(dict(translation_m=shift, unknown_indices=[i for i in front if not math.isfinite(ranges[i])],
            geometry=freeze(geometry), cases=cases,
            zero_pose_finite_comparison=dict(count=len(differences), p50=float(np.median(differences)),
                p95=float(np.percentile(differences, 95)), maximum=max(differences)) if differences else None))
    result = dict(scope='static_truth_generated_sensor_diagnostic_not_control_or_rollout',
        purpose='bidirectional_static_comparison_not_recovery_policy',
        source_seq=3501, recorded_pose=poses[nearest].tolist(), recorded_missing=recorded_missing,
        measurement_age_s=age, used_last_z_from_report=True, rows=rows,
        input_sha256=hashes, input_unchanged=all(digest(p) == h for p, h in hashes.items()),
        source_sha256=sources, source_unchanged=all(digest(ROOT/n) == h for n, h in sources.items()),
        controller_received_truth=False, runtime_files_changed=False, physical_reachability_verified=False,
        simulated_motion_or_new_gazebo_run=False, driving_authorized=False)
    (OUT/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(recorded_missing=recorded_missing, input_unchanged=result['input_unchanged'],
        source_unchanged=result['source_unchanged'], rows=[dict(translation_m=r['translation_m'],
        unknown_indices=r['unknown_indices'], geometry_supported=r['geometry']['supported'],
        zero_pose_finite_comparison=r['zero_pose_finite_comparison'], cases=[dict(angle=c['steering_rad'],
        observation=c['observation']['supported'], reason=c['observation']['reason'], path=c['path']) for c in r['cases']]) for r in rows])))
    return 0 if result['input_unchanged'] and result['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
