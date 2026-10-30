"""오프라인 v2: 같은 스캔의 강체 좌표 변환과 차체 외곽 이동을 구별한다.

v1의 전역 반환점 5 mm 비교는 9 m 벽에서 작은 yaw를 과대평가했다.
동일 광선 전체의 강체 변환 일치(수치 오차 1e-7 m/rad 수준)를 먼저 요구하고,
동일한 5 mm 한도를 차량 네 모서리 이동의 유클리드 노름에 적용한다.
이는 IMU 잡음 제거 또는 실물 정지 증명이 아니다. 현재 적격성·팽창·0명령은
v1과 같고, 바뀌는 것은 다음 fresh 스캔까지 확인 1회를 보존할지뿐이다.
"""
import importlib.util
from pathlib import Path
import sys

import numpy as np

BASE = Path(__file__).resolve().parent
V1 = BASE.parent/'fresh_confirmation_memory_v1'
sys.path.append(str(V1))
spec = importlib.util.spec_from_file_location('_fresh_memory_v1', V1/'candidate_state.py')
legacy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(legacy)

RIGID_NUMERICAL_TOLERANCE = 1e-7
BODY_MOTION_LIMIT_M = legacy.GEOMETRY_POINT_TOLERANCE_M


def snapshot(scene):
    rays = scene.rays
    return dict(
        signature=(scene.source_frame_id, scene.source_epoch, scene.source_first_s, scene.source_last_s,
                   tuple((r.source_index, r.acquired_at_s, r.status, r.source_frame_id, r.source_epoch) for r in rays)),
        origins=np.asarray([r.ray_origin_xy_m for r in rays], dtype=float).reshape(-1, 2),
        directions=np.asarray([r.direction_xy for r in rays], dtype=float).reshape(-1, 2),
        points=np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2))


def geometry_check(current, old, length, width):
    if current['signature'] != old['signature'] or not len(current['origins']):
        return dict(same_raw_scan=False, rigid_scan=False, small_body_motion=False)
    a, b = old['directions'][0], current['directions'][0]
    c, s = float(a@b), float(a[0]*b[1]-a[1]*b[0])
    rotation = np.array([[c, s], [-s, c]])
    translation = current['origins'][0]-old['origins'][0]@rotation
    residuals = []
    for key in ('origins', 'points', 'directions'):
        q, p = old[key], current[key]
        if q.shape != p.shape:
            return dict(same_raw_scan=True, rigid_scan=False, small_body_motion=False)
        if len(p):
            prediction = q@rotation+(0. if key == 'directions' else translation)
            residuals.append(float(np.linalg.norm(prediction-p, axis=1).max()))
    residual = max(residuals, default=float('inf'))
    corners = np.array([[x*length/2, y*width/2] for x in (-1, 1) for y in (-1, 1)])
    motion = float(np.linalg.norm(corners@rotation+translation-corners, axis=1).max())
    return dict(same_raw_scan=True, rigid_scan=residual <= RIGID_NUMERICAL_TOLERANCE,
                small_body_motion=motion <= BODY_MOTION_LIMIT_M,
                rigid_residual=residual, body_corner_displacement_m=motion)


class FreshMemoryParallelTrial(legacy.FreshMemoryParallelTrial):
    def _record(self, scene, baseline, previous, result, now):
        return {**super()._record(scene, baseline, previous, result, now), 'scene_snapshot': snapshot(scene)}

    def _classify(self, scene, baseline, speed, previous, dt, result, now):
        record = self.confirmed
        if record is None:
            return super()._classify(scene, baseline, speed, previous, dt, result, now)
        geometry = geometry_check(snapshot(scene), record['scene_snapshot'], self.config.length_m, self.config.width_m)
        # Reuse v1's wall/time/steering/old-padding counterfactual, changing only its geometry gate.
        # Restore the evidence record in finally; current scene/proposal/output are never altered.
        points = record['points']
        try:
            if all(geometry[k] for k in ('same_raw_scan', 'rigid_scan', 'small_body_motion')):
                record['points'] = np.asarray(scene.occupied_endpoints_xy_m).reshape(-1, 2)
                info = super()._classify(scene, baseline, speed, previous, dt, result, now)
            else:
                info = super()._classify(scene, baseline, speed, previous, dt, result, now)
                info['kept'] = False
        finally:
            record['points'] = points
        checks = info.get('checks', {})
        checks.pop('same_points', None)
        checks.update({k: geometry[k] for k in ('same_raw_scan', 'rigid_scan', 'small_body_motion')})
        info['body_geometry'] = geometry
        return info
