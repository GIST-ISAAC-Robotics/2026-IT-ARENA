"""G2 평가 경로의 Pure Pursuit 추종·두 바퀴 정책. 센서·Gazebo·구동 연결 없음."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import zipfile

from shapely.geometry import Polygon

from arena_autonomy.lateral_intent import LateralIntentConfig
from g0_geometry import TruthBox
from g2_geometry_probe import ProbeConfig
from g2_tracking_probe import rollout, bicycle_step
from surface_contact_eval import VehicleGeometry, ackermann_wheel_angles
from g2_surface_geometry import footprint_at

ROOT = Path(__file__).resolve().parents[1]


def inspect_rollout(rows, target, cfg=None, *, dt=.02, subdivisions=20):
    """고정 조향 bicycle 구간과 경계의 순간 조향 변화를 각각 감싼다.

    최대 1 ms 간격에서 차체/바퀴 꼭짓점의 호길이 이동 상계를 뺀다.
    시간 경계에서 바퀴가 순간 회전하는 이상 모델도 별도로 회전 반경으로
    감싼다. 실제 서보 응답, 종가속·타이어·동적 상대차의 재현은 아니다.
    """
    cfg = cfg or LateralIntentConfig(surface_policy='two_wheel_grass')
    if not rows or not math.isfinite(dt) or dt <= 0 or type(subdivisions) is not int or subdivisions < 1:
        raise ValueError('nonempty rows, positive interval and integer subdivisions required')
    geometry = VehicleGeometry(body_length_m=cfg.length_m, body_width_m=cfg.width_m,
        wheelbase_m=cfg.wheelbase_m, track_width_m=cfg.track_m,
        wheel_radius_m=cfg.wheel_radius_m, wheel_width_m=cfg.wheel_width_m,
        contact_patch_length_m=cfg.contact_patch_length_m)
    obstacle = Polygon(target.footprint.polygon(target.x_m, target.y_m, target.yaw_rad))
    outline_radius = math.hypot(cfg.wheel_radius_m, cfg.wheel_width_m/2)
    patch_radius = math.hypot(cfg.contact_patch_length_m/2, cfg.wheel_width_m/2)
    center_radius = math.hypot(cfg.wheelbase_m, cfg.track_m/2)
    body_radius = math.hypot((cfg.length_m+cfg.wheelbase_m)/2, cfg.width_m/2)
    footprint_radius = max(body_radius, center_radius+outline_radius)
    patch_rear_radius = center_radius+patch_radius
    curvature = max(abs(math.tan(r['applied_steering_rad'])/cfg.wheelbase_m) for r in rows)
    curved = curvature > cfg.curved_curvature_m_inv
    margin = cfg.budget.edge_margin(curved)[0]
    clearance = cfg.budget.lateral_clearance(True, curved)[0]
    min_gap, min_road, min_wall = math.inf, math.inf, math.inf
    inner_min = dict(left=math.inf, right=math.inf)
    grass_run = max_grass_run = 0.
    max_wheels = 0
    count = 0

    def observe(pose, steer, footprint_bound, patch_bound):
        nonlocal min_gap, min_road, min_wall, max_wheels, count
        rx, ry, yaw = pose
        x, y = rx+cfg.wheelbase_m/2*math.cos(yaw), ry+cfg.wheelbase_m/2*math.sin(yaw)
        footprint, wheels = footprint_at(x, y, yaw, steer, geometry)
        min_gap = min(min_gap, max(0., footprint.distance(obstacle)-footprint_bound))
        extent = max(abs(footprint.bounds[1]), abs(footprint.bounds[3]))+footprint_bound
        min_road = min(min_road, cfg.road_half_m-extent)
        min_wall = min(min_wall, cfg.wall_offset_m-extent)
        outside = 0
        for side in inner_min:
            for axle in ('front', 'rear'):
                patch = wheels[f'{axle}_{side}']['patch']
                edge = cfg.road_half_m-max(abs(patch.bounds[1]), abs(patch.bounds[3]))-patch_bound
                inner_min[side] = min(inner_min[side], edge)
                outside += edge < 0
        max_wheels = max(max_wheels, outside)
        count += 1
        return outside > 0

    previous = 0.
    step = dt/subdivisions
    for i, row in enumerate(rows):
        if not math.isclose(row['t_s'], i*dt, abs_tol=1e-8):
            raise ValueError('nonuniform evaluation rollout')
        pose = (row['rear_x_m'], row['rear_y_m'], row['yaw_rad'])
        steer, speed = row['applied_steering_rad'], row['applied_speed_mps']
        if any(not math.isfinite(v) for v in (*pose, steer, speed)):
            raise ValueError('nonfinite evaluation rollout')
        old_angles, new_angles = ackermann_wheel_angles(previous, geometry), ackermann_wheel_angles(steer, geometry)
        angle_change = max(abs(a-b) for a,b in zip(old_angles, new_angles))
        # 바퀴 중심 고정 순간 회전의 모든 중간 각도. 시간 길이 0이므로 duration에는 더하지 않는다.
        observe(pose, previous, outline_radius*angle_change, patch_radius*angle_change)
        k = abs(math.tan(steer)/cfg.wheelbase_m)
        footprint_bound = abs(speed)*step*(1+footprint_radius*k)
        patch_bound = abs(speed)*step*(1+patch_rear_radius*k)
        for _ in range(subdivisions):
            grass = observe(pose, steer, footprint_bound, patch_bound)
            grass_run = grass_run+step if grass else 0.
            max_grass_run = max(max_grass_run, grass_run)
            pose = bicycle_step(*pose, speed, steer, step, cfg.wheelbase_m)
        previous = steer
    return dict(required_gap_m=clearance, required_edge_margin_m=margin,
        minimum_gap_lower_m=min_gap, minimum_road_edge_lower_m=min_road, minimum_wall_edge_lower_m=min_wall,
        minimum_inner_patch_edge_lower_m=inner_min,
        maximum_outside_wheels_upper=max_wheels, maximum_grass_duration_upper_s=max_grass_run,
        team_grass_duration_limit_s=cfg.grass_max_continuous_s, samples=count, curved_budget=curved,
        checks=dict(body_and_wheel_gap=min_gap >= clearance, wall_margin=min_wall >= margin,
                    two_wheel_surface=min_road >= margin or max(inner_min.values()) >= margin,
                    grass_duration_within_team_limit=max_grass_run <= cfg.grass_max_continuous_s),
        driving_authorized=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts/validation'):
        parser.error('new validation output required')
    out.mkdir(parents=True, exist_ok=False)
    names = ['scripts/g2_surface_tracking.py', 'scripts/g2_tracking_probe.py', 'scripts/g2_geometry_probe.py', 'scripts/g2_surface_geometry.py',
             'scripts/g0_geometry.py', 'scripts/surface_contact_eval.py',
             'src/arena_autonomy/arena_autonomy/lateral_intent.py',
             'src/arena_autonomy/arena_autonomy/local_follow.py',
             'src/arena_autonomy/arena_autonomy/local_path.py', 'tests/test_g2_surface_tracking.py']
    original = {p:(ROOT/p).read_bytes() for p in names}
    with zipfile.ZipFile(out/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p,raw in original.items(): archive.writestr(p,raw)
    results = []
    # 긴 전이 1.2 m 한 후보만, 기존 0.4/0.8 m/s·0/100 ms 네 조건.
    for speed in (.4, .8):
        for delay in (0, 5):
            name = f'speed{speed}_delay{delay}'
            result = rollout(-.16, 1.2, speed_cap=speed, delay_steps=delay, target_y=.15,
                             cfg=ProbeConfig(step_m=.001))
            rows = result.pop('rows')
            (out/(name+'_rollout.json')).write_text(json.dumps(rows,indent=2,allow_nan=False))
            surface = inspect_rollout(rows, TruthBox(1.5,.15,.12))
            checks = {**surface['checks'], **{k:result['checks'][k]
                      for k in ('checkpoint_reached','no_path_fault','rejoins_center','rejoins_heading')}}
            results.append(dict(name=name, historical_road_only_metrics=result, surface=surface,
                                checks=checks, conditions_met=all(checks.values()), driving_authorized=False))
    unchanged = all((ROOT/p).read_bytes()==v for p,v in original.items())
    report = dict(scope='exact-reference kinematic tracking only; no sensor/independent protection/Gazebo',
                  source_unchanged=unchanged, source_sha256={p:hashlib.sha256(v).hexdigest() for p,v in original.items()},
                  config=asdict(LateralIntentConfig(surface_policy='two_wheel_grass')), results=results,
                  driving_authorized=False, grass_duration_limit_is_team_assumption_not_official=True)
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(dict(source_unchanged=unchanged,conditions_met=[r['conditions_met'] for r in results])))
    return 0 if unchanged else 1


if __name__ == '__main__':
    raise SystemExit(main())
