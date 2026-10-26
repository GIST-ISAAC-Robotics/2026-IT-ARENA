"""E2 제한 정적 대조: 보존된 G1 주행의 세 자세와 벽, 12/8/6 m 상한.

새 주행·실제 스캔 재생이 아니다. 벽 이외 물체·재질·시각 오차는 재현하지 않는다.
"""
import hashlib
import json
import math
from pathlib import Path
import sys

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from diagnose_front_unknown import wall_geometry
from g0_geometry import ray_box_distance

RUN = ROOT/'artifacts/validation/2026-10-16/g1_corridor_solo_v1/run'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    destination = OUT/'report.json'
    if destination.exists():
        raise FileExistsError('기존 분석 결과를 덮어쓰지 않습니다.')
    rows = json.loads((RUN/'trace.json').read_text())
    boxes, _, meshes = wall_geometry(RUN/'runtime_world.sdf')
    # 저장된 C1 각도 배열을 쓰되 여기서는 정지·수평 자세에서 동시에 교차한다.
    scan = json.loads((RUN/'last_scan.json').read_text())
    angles = [scan['angle_min']+i*scan['angle_increment'] for i in range(len(scan['ranges']))]
    angles = [a for a in angles if abs(a) <= math.radians(100)]
    selected = [('start_straight', min(rows, key=lambda r: abs(r['progress_m']-1.))),
                ('middle_course', min(rows, key=lambda r: abs(r['progress_m']-20.))),
                ('observed_stop_pose', max(rows, key=lambda r: r['progress_m']))]
    report = {'scope': 'three historical static poses, wall-only ray-box geometry; not lap-wide deadlock count, hardware range or controller replay',
              'run': RUN.relative_to(ROOT).as_posix(), 'range_caps_m': [12., 8., 6.],
              'sensor_origin': {'forward_m': .06, 'height_m': .095, 'pitch_rad': 0.}, 'poses': [],
              'source_sha256': {p.relative_to(ROOT).as_posix(): digest(p) for p in
                  [RUN/'runtime_world.sdf', RUN/'trace.json', RUN/'last_scan.json', *meshes,
                   ROOT/'scripts/diagnose_front_unknown.py', ROOT/'scripts/g0_geometry.py', Path(__file__)]}}
    for name, row in selected:
        yaw = row['yaw']
        origin = (row['x']+.06*math.cos(yaw), row['y']+.06*math.sin(yaw), .095)
        distances = []
        for angle in angles:
            direction = (math.cos(yaw+angle), math.sin(yaw+angle), 0.)
            values = [ray_box_distance(origin, direction, box, .05, 100.) for box in boxes]
            distances.append(min((v for v in values if v is not None), default=None))
        report['poses'].append({'name': name, 'progress_m': row['progress_m'], 'time_s': row.get('t', row.get('time')),
            'x': row['x'], 'y': row['y'], 'yaw': yaw, 'rays': len(angles),
            'caps': [{'range_m': cap, 'no_wall_return_rays': sum(v is None or v > cap for v in distances)}
                     for cap in report['range_caps_m']],
            'maximum_wall_distance_m': max((v for v in distances if v is not None), default=None)})
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report['poses'], ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
