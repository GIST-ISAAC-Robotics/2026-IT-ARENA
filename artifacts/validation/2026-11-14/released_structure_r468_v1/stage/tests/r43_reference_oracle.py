"""r43 이전 원래 계산의 보존 사본(기준 오라클). 시험 비교 전용이며 제어기에서 import하지 않는다.

원본 바이트는 artifacts/validation/2026-11-08/grid_cost_r43_v1/before.zip에 있다. 아래 함수 본문은 그 원본의
`local_scene.blind_spec/rear_masked/scene_from_scan`, `lateral_intent.ObservationGrid.__init__`,
`pass_release_trial.threat_view`를 식·순서 그대로 옮겼다(클래스 대신 함수/객체 이름만 다름).
"""
import math
from types import SimpleNamespace

import numpy as np

import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.lateral_intent import (EGO, FREE, GEOMETRY_EPS_M, OCC, ObservationGrid, cell_coordinates,
                                           compact_return_mask, side_threats)
from arena_autonomy.local_scene import MAX_RAYS, LocalScene, SensorRay, _finite, _xy

SIDES = (('left', 1.), ('right', -1.))


# ------------------------------------------------------------------ local_scene (원본)
def blind_spec(value):
    if not isinstance(value, dict):
        return _finite(value, 'rear_half_angle_deg'), ()
    if set(value) - {'rear_half_angle_deg', 'sectors_deg'}:
        raise ValueError('unknown blind spec key')
    rear = _finite(value.get('rear_half_angle_deg', 0.), 'rear_half_angle_deg')
    sectors = []
    for sector in value.get('sectors_deg', ()):
        if len(sector) != 2:
            raise ValueError('blind sector needs center and half width')
        center, half = (_finite(v, 'sectors_deg') for v in sector)
        if not 0 < half < 90:
            raise ValueError('invalid blind sector width')
        sectors.append((center, half))
    return rear, tuple(sectors)


def rear_masked(angle_rad, rear_half_angle_deg=30.):
    angle = math.remainder(_finite(angle_rad, 'angle_rad'), math.tau)
    half, sectors = blind_spec(rear_half_angle_deg)
    if not 0 <= half < 180:
        raise ValueError('invalid rear mask angle')
    if abs(angle) > math.radians(180 - half) + 1e-12:
        return True
    return any(abs(math.remainder(angle - math.radians(center), math.tau)) <= math.radians(width) + 1e-12
               for center, width in sectors)


def scene_from_scan(ranges, *, angle_min, angle_increment, range_min, range_max,
                    acquired_at_s, t_ref_s, source_frame_id='laser_frame', source_epoch=0,
                    sensor_xy_m=(.06, 0.), sensor_yaw_rad=0.,
                    acquisition_to_ref_se2=None, rear_half_angle_deg=30.):
    if len(ranges) > MAX_RAYS or len(acquired_at_s) != len(ranges):
        raise ValueError('ray count/timestamps mismatch')
    values = tuple(ranges)
    stamps = tuple(_finite(v, 'acquired_at_s') for v in acquired_at_s)
    minimum, maximum = _finite(range_min, 'range_min'), _finite(range_max, 'range_max')
    start, increment = _finite(angle_min, 'angle_min'), _finite(angle_increment, 'angle_increment')
    yaw = _finite(sensor_yaw_rad, 'sensor_yaw_rad')
    sx, sy = _xy(sensor_xy_m, 'sensor_xy_m')
    if minimum < 0 or maximum <= minimum or increment <= 0:
        raise ValueError('invalid scan metadata')
    poses = ((0., 0., 0.),) * len(values) if acquisition_to_ref_se2 is None else tuple(acquisition_to_ref_se2)
    if len(poses) != len(values):
        raise ValueError('transform count mismatch')
    rays = []
    for index, (distance, stamp, pose) in enumerate(zip(values, stamps, poses)):
        if len(pose) != 3:
            raise ValueError('SE(2) transform must have three components')
        dx, dy, turn = (_finite(v, 'transform') for v in pose)
        c, s = math.cos(turn), math.sin(turn)
        origin = (dx + c * sx - s * sy, dy + s * sx + c * sy)
        angle = math.remainder(start + index * increment, math.tau)
        masked = rear_masked(angle, rear_half_angle_deg)
        valid = (not isinstance(distance, bool) and isinstance(distance, (int, float))
                 and math.isfinite(distance) and minimum <= distance <= maximum)
        status = 'masked' if masked else ('return' if valid else 'unknown')
        bearing = angle + yaw + turn
        direction = (math.cos(bearing), math.sin(bearing))
        hit = None
        if status == 'return':
            hit = (origin[0] + distance * direction[0], origin[1] + distance * direction[1])
        rays.append(SensorRay(origin, hit, stamp, status, source_frame_id, source_epoch, index, direction))
    return LocalScene(t_ref_s, stamps[0] if stamps else None, stamps[-1] if stamps else None,
                      source_frame_id, source_epoch, tuple(rays))


# ------------------------------------------------------------------ lateral_intent.ObservationGrid (원본 생성)
def reference_grid(scene, cfg):
    """원본 ObservationGrid.__init__의 전체 표본 래스터. 같은 메서드(index·observed_free 등)를 쓰는 객체를 돌려준다."""
    self = ObservationGrid.__new__(ObservationGrid)
    self.cfg = cfg
    r = cfg.grid_resolution_m
    self.x0, self.x1 = cfg.grid_x_m
    self.y0, self.y1 = cfg.grid_y_m
    self.nx = int(round((self.x1-self.x0)/r))
    self.ny = int(round((self.y1-self.y0)/r))
    self.r = r
    state = np.zeros((self.nx, self.ny), dtype=np.int8)
    origins, hits = [], []
    for ray in scene.rays:
        if ray.status == 'return':
            origins.append(ray.ray_origin_xy_m)
            hits.append(ray.hit_xy_m)
    self.returns = np.asarray(hits, dtype=float).reshape(-1, 2)
    self.ages = np.asarray([scene.t_ref_s-ray.acquired_at_s for ray in scene.rays if ray.status == 'return'], dtype=float)
    self.compact_returns, self.return_components = compact_return_mask(scene, cfg)
    self.reference_samples = 0
    if len(hits):
        o = np.asarray(origins, dtype=float)
        h = self.returns
        vec = h-o
        dist = np.linalg.norm(vec, axis=1)
        free = np.maximum(0., dist-cfg.budget.range_bias_m)
        step = r/2
        counts = np.ceil(free/step).astype(int)
        total = int(np.sum(counts))
        self.reference_samples = total
        if total:
            ray_index = np.repeat(np.arange(len(o)), counts)
            offsets = np.arange(total)-np.repeat(np.cumsum(counts)-counts, counts)
            t = np.minimum(offsets*step, np.repeat(free, counts))
            pts = o[ray_index]+(vec[ray_index]/np.maximum(dist[ray_index], 1e-9)[:, None])*t[:, None]
            ix, iy, inside = self.index(pts)
            self.reference_inside_samples = int(np.sum(inside))
            state[ix[inside], iy[inside]] = FREE
        ix, iy, inside = self.index(h)
        state[ix[inside], iy[inside]] = OCC
    ex, ey = self.centers()
    ego = (np.abs(ex) <= cfg.length_m/2+GEOMETRY_EPS_M) & (np.abs(ey) <= cfg.width_m/2+GEOMETRY_EPS_M)
    state[ego] = EGO
    self.state = state
    return self


# ------------------------------------------------------------------ pass_release_trial.threat_view (원본)
def threat_view(scene, path_points, cfg, grid_factory=reference_grid):
    path = np.asarray(path_points if path_points is not None else [], dtype=float).reshape(-1, 2)
    if len(path) < 2 or not np.all(np.isfinite(path)):
        return None
    try:
        frame = mx.ExecutionFrame(path)
    except (ValueError, TypeError):
        return None
    state = mx.ego_state(frame, cfg)
    light = mx.ReturnGrid(scene, cfg)
    ds, masks = mx.side_return_masks(frame, state, light, cfg)
    if any(np.any(m) for m in masks):
        grid = grid_factory(scene, cfg)
        s, l = cell_coordinates(grid, frame)
        observed = grid.observed_free()
    else:
        grid, s, l, observed = light, None, None, None
    threats = side_threats(frame, state, grid, cfg, s, l, observed)
    out = {'frame': frame, 'state': state}
    for (name, _), mask in zip(SIDES, masks):
        values = ds[mask] if len(ds) else np.zeros(0)
        out[name] = {'present': bool(threats[name]['present']), 'points': int(threats[name]['points']),
                     'rearmost_ds_m': float(np.min(values)) if len(values) else None,
                     'frontmost_ds_m': float(np.max(values)) if len(values) else None}
    return out


def grid_fields(grid):
    """비교할 전체 격자 필드. 배열은 dtype·shape·값을 모두 비교한다."""
    return SimpleNamespace(state=grid.state, returns=grid.returns, ages=grid.ages, compact=grid.compact_returns,
                           components=grid.return_components, observed=grid.observed_free(),
                           dims=(grid.x0, grid.x1, grid.y0, grid.y1, grid.nx, grid.ny, grid.r))
