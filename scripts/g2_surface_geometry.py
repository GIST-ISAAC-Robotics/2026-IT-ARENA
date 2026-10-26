"""G2 두 바퀴 잔디 정책의 평가 전용 기하. 센서 경로/주행 허가가 아니다.

10/13의 후륜축 5차 곡선을 그대로 사용하고 10/24 측방 예산과 바퀴
접지 사각형을 적용한다. 정지 표적도 재출발할 수 있어 moving=True
예산을 유지한다. 차체/바퀴의 표본 사이 이동 상계로 연속 여유를 낮춘다.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np
from shapely.geometry import Polygon

from arena_autonomy.lateral_intent import LateralIntentConfig
from g0_geometry import TruthBox
from g2_geometry_probe import ProbeConfig, sample_path
from surface_contact_eval import VehicleGeometry, rectangle, wheel_shapes

ROOT = Path(__file__).resolve().parents[1]


class ComponentFootprint:
    """합집합 생성 없이 동일 거리/외곽을 계산한다. 유한 성분 합집합의 항등식이다."""

    def __init__(self, components):
        self.components = tuple(components)
        if not self.components or any(p.is_empty or not p.is_valid for p in self.components):
            raise ValueError('invalid footprint component')
        bounds = np.asarray([p.bounds for p in self.components])
        if not np.all(np.isfinite(bounds)):
            raise ValueError('nonfinite footprint')
        self.bounds = (float(bounds[:,0].min()), float(bounds[:,1].min()),
                       float(bounds[:,2].max()), float(bounds[:,3].max()))

    def distance(self, obstacle):
        return min(p.distance(obstacle) for p in self.components)


def footprint_at(x, y, yaw, steering, geometry):
    wheels = wheel_shapes(x, y, yaw, steering, geometry)
    body = rectangle(x, y, yaw, geometry.body_length_m, geometry.body_width_m)
    return ComponentFootprint([body]+[w['outline'] for w in wheels.values()]), wheels


def interval_bounds(path, cfg):
    """해석적 quintic의 차체·바퀴 꼭짓점 이동 상계. 조향각 변화도 포함한다."""
    offset, length = abs(path['offset_m']), path['transition_m']
    slope = 1.875*offset/length
    second = 10/math.sqrt(3)*offset/length**2
    third = 60*offset/length**3
    # k = y''/(1+y'^2)^1.5, |dk/dx| <= |y'''|+3|y'||y''|^2.
    curvature = path['curvature_bound_per_m']
    k_derivative = third+3*slope*second**2
    denominator = 1-cfg.track_m*curvature/2
    if denominator <= 0:
        raise ValueError('Ackermann singularity cannot be bounded')
    ackermann_derivative = cfg.wheelbase_m*k_derivative/denominator**2
    arc_per_x = math.sqrt(1+slope*slope)
    yaw_per_x = curvature*arc_per_x
    wheel_center_radius = math.hypot(cfg.wheelbase_m, cfg.track_m/2)
    center_per_x = arc_per_x+wheel_center_radius*yaw_per_x
    patch_radius = math.hypot(cfg.contact_patch_length_m/2, cfg.wheel_width_m/2)
    outline_radius = math.hypot(cfg.wheel_radius_m, cfg.wheel_width_m/2)
    angle_per_x = yaw_per_x+ackermann_derivative
    dx = np.diff([r['rear_x_m'] for r in path['samples']])
    patch = dx*(center_per_x+patch_radius*angle_per_x)
    outline = dx*(center_per_x+outline_radius*angle_per_x)
    body = np.asarray(path['interval_motion_bounds_m'])
    return dict(patch_m=np.r_[patch, 0.], footprint_m=np.r_[np.maximum(body, outline), 0.],
                curvature_bound_per_m=curvature)


def geometry_at(row, cfg):
    yaw = row['yaw_rad']
    x = row['rear_x_m']+cfg.wheelbase_m/2*math.cos(yaw)
    y = row['rear_y_m']+cfg.wheelbase_m/2*math.sin(yaw)
    steer = math.atan(cfg.wheelbase_m*row['curvature_per_m'])
    geometry = VehicleGeometry(body_length_m=cfg.length_m, body_width_m=cfg.width_m,
        wheelbase_m=cfg.wheelbase_m, track_width_m=cfg.track_m,
        wheel_radius_m=cfg.wheel_radius_m, wheel_width_m=cfg.wheel_width_m,
        contact_patch_length_m=cfg.contact_patch_length_m)
    return footprint_at(x, y, yaw, steer, geometry)


def assess(path, target, cfg=None):
    cfg = cfg or LateralIntentConfig(surface_policy='two_wheel_grass')
    if not isinstance(target, TruthBox):
        raise ValueError('evaluation TruthBox required')
    bounds = interval_bounds(path, cfg)
    curved = bounds['curvature_bound_per_m'] > cfg.curved_curvature_m_inv
    margin, _ = cfg.budget.edge_margin(curved)
    clearance, _ = cfg.budget.lateral_clearance(moving=True, curved=curved)
    obstacle = Polygon(target.footprint.polygon(target.x_m, target.y_m, target.yaw_rad))
    gap, road, wall = math.inf, math.inf, math.inf
    inside_pair = dict(left=True, right=True)
    max_outside = 0
    minimum_patch_edge = dict(left=math.inf, right=math.inf)
    for i, row in enumerate(path['samples']):
        body, wheels = geometry_at(row, cfg)
        footprint_bound = float(bounds['footprint_m'][i])
        patch_bound = float(bounds['patch_m'][i])
        gap = min(gap, max(0., body.distance(obstacle)-footprint_bound))
        extent = max(abs(body.bounds[1]), abs(body.bounds[3]))+footprint_bound
        road = min(road, cfg.road_half_m-extent)
        wall = min(wall, cfg.wall_offset_m-extent)
        unconfirmed = 0
        for side in inside_pair:
            for axle in ('front', 'rear'):
                patch = wheels[f'{axle}_{side}']['patch']
                patch_edge = cfg.road_half_m-max(abs(patch.bounds[1]), abs(patch.bounds[3]))-patch_bound
                minimum_patch_edge[side] = min(minimum_patch_edge[side], patch_edge)
                ok = patch_edge >= margin
                inside_pair[side] &= ok
                unconfirmed += not ok
        max_outside = max(max_outside, unconfirmed)
    steering_upper = math.atan(cfg.wheelbase_m*bounds['curvature_bound_per_m'])
    surface = road >= margin or any(inside_pair.values())
    checks = dict(steering=steering_upper <= cfg.max_steering_rad,
                  body_and_wheel_gap=gap >= clearance, wall_margin=wall >= margin,
                  two_wheel_surface=surface)
    return dict(offset_m=path['offset_m'], transition_m=path['transition_m'], curved_budget=curved,
                required_gap_m=clearance, required_edge_margin_m=margin,
                minimum_gap_lower_m=gap, minimum_road_edge_lower_m=road,
                minimum_wall_edge_lower_m=wall, inner_pair_confirmed=inside_pair,
                minimum_patch_edge_lower_m=minimum_patch_edge,
                maximum_unconfirmed_wheels=max_outside,
                max_footprint_interval_bound_m=float(max(bounds['footprint_m'])),
                steering_upper_rad=steering_upper, checks=checks,
                geometry_conditions_met=all(checks.values()), driving_authorized=False,
                observation_verified=False, tracking_verified=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts/validation'):
        parser.error('new validation output required')
    out.mkdir(parents=True, exist_ok=False)
    sources = ['scripts/g2_surface_geometry.py', 'scripts/g2_geometry_probe.py',
               'scripts/g0_geometry.py', 'scripts/surface_contact_eval.py',
               'src/arena_autonomy/arena_autonomy/lateral_intent.py',
               'tests/test_g2_surface_geometry.py']
    original = {p: (ROOT/p).read_bytes() for p in sources}
    with zipfile.ZipFile(out/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p, raw in original.items():
            archive.writestr(p, raw)
    cfg = LateralIntentConfig(surface_policy='two_wheel_grass')
    # 유한 27조건. 성공 조건을 찾을 때까지 후보를 자동 확대하지 않는다.
    paths = [sample_path(d, length, ProbeConfig(step_m=.001))
             for d in (-.16, -.20, -.24) for length in (.6, .9, 1.2)]
    results = []
    for lateral in (0., .08, .15):
        target = TruthBox(1.5, lateral, .12)
        rows = [assess(path, target, cfg) for path in paths]
        results.append(dict(target=asdict(target), candidates=rows,
                            geometry_candidates=sum(r['geometry_conditions_met'] for r in rows)))
    unchanged = all((ROOT/p).read_bytes()==raw for p, raw in original.items())
    report = dict(scope='truth geometry only; no controller input or driving authorization',
        config=asdict(cfg), results=results, source_unchanged=unchanged,
        source_sha256={p:hashlib.sha256(raw).hexdigest() for p,raw in original.items()},
        driving_authorized=False, limits=[
            'Exact analytic path and known rectangular target, not a perception or execution result.',
            'Moving lateral budget is retained; no proof the target remains stationary.',
            'No blind-region clearance, tracking, braking, material or tire dynamics proof.',
            'Two whole same-side contact patches stay within road minus the provisional budget.',
            'No claim about official long-duration grass penalty or all possible paths.'])
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(dict(source_unchanged=unchanged, geometry_candidates=[r['geometry_candidates'] for r in results])))
    return 0 if unchanged else 1


if __name__ == '__main__':
    raise SystemExit(main())
