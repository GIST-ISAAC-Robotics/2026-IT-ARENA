#!/usr/bin/env python3
"""G2 준비용 정답 기하 검사. 제어 입력/경로 선택/주행 허가로 연결하지 않는다.

직선 45 cm 도로에서 후륜축 quintic 횡이동의 차체 외곽을 검사한다.
샘플 사이 강체 운동을 호길이/곡률 상한으로 팽창해 간격의 하한을 낸다.
실제 Pure Pursuit 추종, 관측, 속도/제동, 시간축 상대 운동은 미검증이다.
"""
import argparse
from dataclasses import dataclass, asdict
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np
from shapely.geometry import Polygon

from g0_geometry import Footprint, TruthBox, cross_section_gaps

ROOT = Path(__file__).resolve().parents[1]


def numeric(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('finite numeric value required')
    return float(value)


@dataclass(frozen=True)
class ProbeConfig:
    road_width_m: float = .45
    wheelbase_m: float = .135
    length_m: float = .20
    width_m: float = .15
    edge_margin_m: float = .025  # G0 기존 임시 여유; 실측값 아님
    minimum_body_gap_m: float = .15  # G1 기존 시험선; G2를 위해 완화하지 않음
    max_steering_rad: float = .37
    step_m: float = .005

    def __post_init__(self):
        if any(numeric(value) <= 0 for value in vars(self).values()):
            raise ValueError('positive probe configuration required')
        if (self.wheelbase_m > self.length_m or self.max_steering_rad >= math.pi/2
                or self.step_m > .02 or self.width_m+2*self.edge_margin_m >= self.road_width_m):
            raise ValueError('invalid probe geometry')


def quintic(t):
    """위치·1차·2차 미분. t는 닫힌 구간 [0,1]이다."""
    t = numeric(t)
    if not 0 <= t <= 1:
        raise ValueError('quintic fraction outside interval')
    return (10*t**3-15*t**4+6*t**5,
            30*t*t*(1-t)**2, 60*t*(1-t)*(1-2*t))


def sample_path(offset_m, transition_m, config=ProbeConfig()):
    """후륜축 기준 해석적 경로. 반환된 정답은 평가 도구에서만 사용한다."""
    offset, length = numeric(offset_m), numeric(transition_m)
    if abs(offset) > .30 or not .2 <= length <= 1.2:
        raise ValueError('bounded probe family exceeded')
    start, return_start = .10, 1.85
    end = return_start+length+.35
    # x는 후륜축. 초기 차체 중앙은 (0,0), yaw=0이다.
    x_values = np.linspace(-config.wheelbase_m/2, end,
                           math.ceil((end+config.wheelbase_m/2)/config.step_m)+1)
    rear_to_corner = math.hypot((config.length_m+config.wheelbase_m)/2, config.width_m/2)
    slope_bound = 1.875*abs(offset)/length
    curvature_bound = (10/math.sqrt(3))*abs(offset)/length**2
    shape = Footprint(config.length_m, config.width_m)
    samples = []
    for x in x_values:
        if x <= start:
            y, slope, second = 0., 0., 0.
        elif x < start+length:
            f, d, dd = quintic(float((x-start)/length))
            y, slope, second = offset*f, offset*d/length, offset*dd/length**2
        elif x <= return_start:
            y, slope, second = offset, 0., 0.
        elif x < return_start+length:
            f, d, dd = quintic(float((x-return_start)/length))
            y, slope, second = offset*(1-f), -offset*d/length, -offset*dd/length**2
        else:
            y, slope, second = 0., 0., 0.
        yaw = math.atan(slope)
        curvature = second/(1+slope*slope)**1.5
        cx, cy = x+config.wheelbase_m/2*math.cos(yaw), y+config.wheelbase_m/2*math.sin(yaw)
        polygon = Polygon(shape.polygon(cx, cy, yaw))
        samples.append(dict(rear_x_m=float(x), rear_y_m=y, yaw_rad=yaw,
                            curvature_per_m=curvature, polygon=polygon))
    # 모든 구간에서 ds <= dx sqrt(1+max(y')²), |dtheta| <= max(|k|) ds.
    # 전체 꼭짓점 이동 <= ds + r |dtheta|. 보간/볼록 껍질의 과소평가에 기대지 않는다.
    bounds = [(b['rear_x_m']-a['rear_x_m'])*math.sqrt(1+slope_bound**2)
              *(1+rear_to_corner*curvature_bound) for a, b in zip(samples, samples[1:])]
    return dict(samples=samples, interval_motion_bounds_m=bounds,
                curvature_bound_per_m=curvature_bound, offset_m=offset, transition_m=length)


def assess(path, target, config=ProbeConfig()):
    if not isinstance(target, TruthBox):
        raise ValueError('evaluation TruthBox required')
    obstacle = Polygon(target.footprint.polygon(target.x_m, target.y_m, target.yaw_rad))
    samples = path['samples']
    bounds = path['interval_motion_bounds_m']+[0.]
    distances = [row['polygon'].distance(obstacle) for row in samples]
    gap_lower = min(max(0., gap-bound) for gap, bound in zip(distances, bounds))
    road_lower = min(config.road_width_m/2 - max(abs(row['polygon'].bounds[1]),
                      abs(row['polygon'].bounds[3])) - bound for row, bound in zip(samples, bounds))
    # sampled peak은 설명용. 합격은 analytic upper bound로만 판정한다.
    steering_upper = math.atan(config.wheelbase_m*path['curvature_bound_per_m'])
    checks = dict(road_margin=road_lower >= config.edge_margin_m,
                  body_gap=gap_lower >= config.minimum_body_gap_m,
                  steering_bound=steering_upper <= config.max_steering_rad)
    return dict(offset_m=path['offset_m'], transition_m=path['transition_m'],
                minimum_sampled_body_gap_m=min(distances), minimum_body_gap_lower_m=gap_lower,
                minimum_road_edge_lower_m=road_lower, steering_upper_rad=steering_upper,
                sampled_max_steering_rad=max(abs(math.atan(config.wheelbase_m*r['curvature_per_m'])) for r in samples),
                max_interval_motion_bound_m=max(bounds), checks=checks,
                geometry_conditions_met=all(checks.values()), driving_authorized=False,
                observation_verified=False, tracking_verified=False)


def cases():
    # y=.15는 도로 끝에 닿는다. .20/.24는 일부가 잔디 쪽으로 나온 별도 조건이다.
    for y in (0., .10, .15, .20, .24):
        for sign in ((1,) if y == 0 else (-1, 1)):
            yield f'y{sign*y:+.2f}_yaw0', TruthBox(1.5, sign*y, .12)
    for yaw in (-math.pi/6, math.pi/6):
        yield f'y+0.20_yaw{math.degrees(yaw):+.0f}', TruthBox(1.5, .20, .12, yaw_rad=yaw)
    yield 'low_y+0.20', TruthBox(1.5, .20, .06)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/validation'):
        parser.error('output must be a new validation subdirectory')
    output.mkdir(parents=True, exist_ok=False)
    names = ['scripts/g2_geometry_probe.py', 'scripts/g0_geometry.py', 'tests/test_g2_geometry_probe.py',
             'config/tests/g1_side_occlusion_v1.json', 'config/tests/multi_vehicle_g0.json']
    raw = {name: (ROOT/name).read_bytes() for name in names}
    hashes = {name: hashlib.sha256(value).hexdigest() for name, value in raw.items()}
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for name, data in raw.items():
            bundle.writestr(name, data)
    cfg = ProbeConfig()
    # 0 offset은 기준선. ±.15는 도로 여유에서 탈락하는 대조군도 겸한다.
    paths = [sample_path(offset, length, cfg) for offset in (-.15, -.125, -.10, -.075, 0., .075, .10, .125, .15)
             for length in (.6, .9, 1.2)]
    rows = []
    for name, target in cases():
        results = [assess(path, target, cfg) for path in paths]
        rows.append(dict(name=name, target=asdict(target),
                         g0_section=cross_section_gaps(target),
                         scan_plane_intersects=target.bottom_m <= .095 <= target.bottom_m+target.height_m,
                         geometry_candidates=sum(r['geometry_conditions_met'] for r in results), candidates=results))
    unchanged = all(hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == digest for name, digest in hashes.items())
    report = dict(scope='evaluation_truth_geometry_only_no_control', config=asdict(cfg), cases=rows,
                  sources_unchanged=unchanged, source_sha256=hashes, driving_authorized=False,
                  pass_gap_is_existing_G1_test_not_physical_safety=True,
                  limits=['Finite candidate family failure is not a proof that all paths are impossible.',
                          'No observation/occlusion, dynamic target, controller tracking or braking verification.',
                          'The road and obstacle are evaluation truth, never controller input.',
                          'Continuous bound assumes this exact analytic rear-axle path, not the real vehicle.',
                          'G0 section margin and G1 body-gap requirement are different tests.'])
    (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps(dict(sources_unchanged=unchanged, cases=[{k:r[k] for k in ('name', 'geometry_candidates', 'scan_plane_intersects')} for r in rows])))
    return 0 if unchanged else 1


if __name__ == '__main__':
    raise SystemExit(main())
