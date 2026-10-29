"""첫 소실 래치에서 센서 취득 시각만으로 세 가지 이동 범위를 비교한다."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'),
               str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from probe_joint_missing_sweep import joint_missing_sweep
from probe_timed_missing_sweep import timed_missing_sweep


def main():
    out = BASE/'first.json'
    if out.exists() or (BASE/'source.zip').exists():
        raise FileExistsError('preserve existing diagnostic')
    trace = ROOT/'artifacts/validation/2026-10-27/short_prefix_connection_v1/run1_course/follow_decisions.jsonl'
    calls = []
    with trace.open() as stream:
        header = json.loads(next(stream))['data']
        for line in stream:
            row = json.loads(line)
            if row['kind'] != 'call':
                continue
            data = row['data']
            if data['input']['reference_s'] < 129.2:
                continue
            calls.append(data)
            partial = data['output'][2].get('partial_observation') or {}
            if partial.get('latch_evidence'):
                break
    evidence = partial['latch_evidence']
    previous = next(c for c in calls if abs(c['input']['reference_s']-evidence['previous_reference_s']) < 1e-8)
    motion = MotionConfig(**header['motion_config'])
    d, scan, history = restore_call(previous['input'], motion)
    scene = motion_scene(scan, history, d['reference_s'], timing_verified=d['timing_verified'],
                         epoch=d['epoch'], rear_half_angle_deg=d['rear_half_angle_deg'])
    rays = [r for r in scene.rays if r.status == 'return' and np.linalg.norm(r.hit_xy_m) <= 3.]
    context = evidence['correspondence_context']
    old = np.asarray(context['previous_points_at_reference_xy'])
    assert np.allclose(old, [r.hit_xy_m for r in rays], rtol=0., atol=1e-9)
    carried = np.asarray(context['previous_points_now_xy'])
    points = np.array([w['previous_point_now_xy'] for w in evidence['witnesses']])
    indices = [int(np.argmin(np.linalg.norm(carried-p, axis=1))) for p in points]
    assert np.allclose(carried[indices], points, rtol=0., atol=1e-9)
    times = np.array([rays[i].acquired_at_s for i in indices])
    witnesses = {w['source_index']: w for w in evidence['witnesses']}
    origins = [w['origin_xy'] for w in witnesses.values()]
    directions = [w['direction_rad'] for w in witnesses.values()]
    current_scan = data['input']['scan']
    missing_times = [evidence['source_first_s'] + i*current_scan['time_increment'] for i in witnesses]
    args = dict(half_angle=current_scan['angle_increment']/2, required=partial['history_distance_m'],
                speed=evidence['sweep_speed_mps'], age=evidence['reference_s']-evidence['source_first_s'],
                steering=evidence['steering_rad'], previous_steering=evidence['previous_steering_rad'])
    # 기존 기하 계산은 그대로 두고 입력 시각의 차이만 비교한다.
    distance, _ = joint_missing_sweep(points, origins, directions, **args)
    reference_radius = .03 + 2*(evidence['reference_s']-evidence['previous_reference_s'])
    point_radius = .03 + 2*(evidence['reference_s']-times)
    timed = timed_missing_sweep(points, times, origins, directions, missing_times,
                               evidence['reference_s'], **args)
    sources = [Path(__file__), ROOT/'scripts/probe_timed_missing_sweep.py',
               ROOT/'scripts/probe_joint_missing_sweep.py',
               *sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    with zipfile.ZipFile(BASE/'source.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in sources:
            archive.write(p, p.relative_to(ROOT).as_posix())
    result = dict(scope='sensor-only first-latch timing diagnosis; no ROS, driving, object identity or safety proof',
        reference_s=evidence['reference_s'], previous_reference_s=evidence['previous_reference_s'],
        previous_ray_indices=[rays[i].source_index for i in indices], previous_acquired_s=times.tolist(),
        missing_ray_indices=list(witnesses), missing_acquired_s=missing_times,
        static_joint_distance_m=distance.tolist(), reference_radius_m=reference_radius,
        point_radius_m=point_radius.tolist(),
        reference_possible=(distance <= reference_radius).tolist(),
        point_time_only_possible=(distance <= point_radius).tolist(), timed=timed,
        source_sha256=hashes, trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest())
    result['source_unchanged'] = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p,h in hashes.items())
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('source_sha256',)}))


if __name__ == '__main__':
    main()
