"""G2 평가 전용 광선-차체 영역 증거. 영역 전체가 비었다고 판정하지 않는다.

LocalScene의 원점·방향·취득시각을 보존한다. 광선 사이 보간/가림 해제 없음.
"""
from dataclasses import dataclass
import math

from arena_autonomy.local_scene import LocalScene
from g2_geometry_probe import numeric


@dataclass(frozen=True)
class QueryBox:
    x_m: float
    y_m: float
    yaw_rad: float = 0.
    length_m: float = .20
    width_m: float = .15

    def __post_init__(self):
        for value in vars(self).values():
            numeric(value)
        if self.length_m <= 0 or self.width_m <= 0:
            raise ValueError('positive query dimensions required')


def ray_interval(origin, direction, query):
    """단위 광선이 회전 사각형 내부에 있는 양수 거리 구간. 접선도 포함."""
    c, s = math.cos(query.yaw_rad), math.sin(query.yaw_rad)
    dx, dy = origin[0]-query.x_m, origin[1]-query.y_m
    local_o = (c*dx+s*dy, -s*dx+c*dy)
    local_d = (c*direction[0]+s*direction[1], -s*direction[0]+c*direction[1])
    lo, hi = 0., math.inf
    for o, d, half in zip(local_o, local_d, (query.length_m/2, query.width_m/2)):
        if abs(d) < 1e-14:
            if abs(o) > half:
                return None
            continue
        a, b = (-half-o)/d, (half-o)/d
        lo, hi = max(lo, min(a, b)), min(hi, max(a, b))
        if hi < lo:
            return None
    return lo, hi


def query_evidence(scene, query, *, now_s, max_age_s=.3, endpoint_margin_m=.03):
    if not isinstance(scene, LocalScene) or not isinstance(query, QueryBox):
        raise ValueError('typed scene and query required')
    now, age_limit, margin = map(numeric, (now_s, max_age_s, endpoint_margin_m))
    if now != scene.t_ref_s or not 0 < age_limit <= .3 or margin < 0:
        raise ValueError('query time/frame or evidence bounds invalid')
    counts = dict(return_beyond=0, hit_or_margin=0, occluded_before=0,
                  unknown=0, masked=0, stale=0, invalid_ray=0)
    details = []
    for ray in scene.rays:
        interval = ray_interval(ray.ray_origin_xy_m, ray.direction_xy, query)
        if interval is None:
            continue
        lo, hi = interval
        age = now-ray.acquired_at_s
        if age < 0 or age >= age_limit:
            kind = 'stale'
        elif ray.status != 'return':
            kind = ray.status
        else:
            delta = (ray.hit_xy_m[0]-ray.ray_origin_xy_m[0], ray.hit_xy_m[1]-ray.ray_origin_xy_m[1])
            length = delta[0]*ray.direction_xy[0]+delta[1]*ray.direction_xy[1]
            cross = delta[0]*ray.direction_xy[1]-delta[1]*ray.direction_xy[0]
            if length < 0 or abs(cross) > 1e-8*max(1., abs(length)):
                kind = 'invalid_ray'
            elif length < lo-margin:
                kind = 'occluded_before'
            elif length > hi+margin:
                kind = 'return_beyond'
            else:
                kind = 'hit_or_margin'
        counts[kind] += 1
        details.append(dict(source_index=ray.source_index, acquired_at_s=ray.acquired_at_s,
                            age_s=age, kind=kind, intersection_range_m=[lo, hi]))
    return dict(counts=counts, intersecting_rays=len(details), details=details,
                no_intersecting_beam=not details, free_area_certified=False,
                driving_authorized=False, angular_gaps_filled=False,
                scope='individual_intersecting_ray_evidence_not_continuous_free_area')
