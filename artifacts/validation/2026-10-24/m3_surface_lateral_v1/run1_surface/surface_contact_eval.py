#!/usr/bin/env python3
"""바퀴별 노면 접촉·잔디 진입 사건의 사후 평가. 정답 월드·자세를 쓰는 평가 전용이며 제어 입력이 아니다.

기존 보수 지표(20×15 cm 차체 외형이 SDF 도로 합집합 밖으로 나간 면적, `audit_local_pursuit_road`·
`g1_traffic_eval`)는 바꾸지 않고 같은 식으로 다시 계산해 함께 남긴다. 이 모듈은 그 옆에 바퀴별
접지면·조향 외곽·공식 벌점 추정 사건·지속 시간을 따로 더한다.

- 지면 분류 우선순위: 벽 > 도로 > 잔디 > 기타 지면. 벽 상단 0.4 m, 도로 상단 3 mm, 잔디 2 mm,
  지면 0 m이므로 바퀴를 받치는 면의 높이 순서와 같다. 도로·잔디 어느 것도 아닌 지면(공식 #13의
  갈림길·접합부 공백 등)은 `other`이며 잔디로 세지 않는다.
- 공식 문구는 “바퀴가 과반 초과로 나간 경우”이고 바퀴 하나가 얼마나 걸쳐야 “나간” 것인지는
  정의하지 않았다. 따라서 `any`(접지면 일부라도 도로 밖)·`center`(접지 중심이 도로 밖)·
  `full`(접지면 전체가 도로 밖) 세 해석을 모두 계산한다. 어느 해석도 심판 판정이 아니다.
- 사건은 벌점 추정 상태(나간 바퀴 ≥3)의 상승 에지다. 원시 에지와, 짧은 복귀를 합친 사건을 함께
  남긴다. 병합 간격은 평가 가정이며 공식 값이 아니다.
"""
import argparse
from bisect import bisect_right
from dataclasses import dataclass, asdict
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
from shapely.prepared import prep

SURFACES = ('wall', 'road', 'grass', 'other')
INTERPRETATIONS = ('any', 'center', 'full')
WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')
EVALUATOR_VERSION = 'surface_contact_v1'
FRACTION_EPS = 1e-9


def rectangle(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    return Polygon([(x+lx*c-ly*s, y+lx*s+ly*c)
                    for lx, ly in ((-length/2, -width/2), (length/2, -width/2),
                                   (length/2, width/2), (-length/2, width/2))])


@dataclass(frozen=True)
class VehicleGeometry:
    """평가용 차량 기하. 기본값은 `vehicle.yaml`의 현재 임시값이며 실측이 아니다.

    접지면 길이는 실측 자료가 없는 평가 가정이다(65 mm 강체 바퀴·평판 접촉에서 수 mm–1 cm 규모).
    폭은 바퀴 폭 전체를 쓴다. 조향 축은 시뮬레이션 모델처럼 바퀴 중심을 지난다고 둔다.
    """
    body_length_m: float = .20
    body_width_m: float = .15
    wheelbase_m: float = .135
    track_width_m: float = .135
    wheel_radius_m: float = .0325
    wheel_width_m: float = .012
    contact_patch_length_m: float = .010
    max_wheel_steering_rad: float = .45

    def __post_init__(self):
        values = asdict(self)
        if any(isinstance(v, bool) or not math.isfinite(v) or v <= 0 for v in values.values()):
            raise ValueError('vehicle geometry must be finite and positive')
        if self.wheelbase_m+2*self.wheel_radius_m > self.body_length_m+1e-9 or \
                self.track_width_m+self.wheel_width_m > self.body_width_m+1e-9:
            raise ValueError('straight wheel envelope must stay inside the registered body footprint')
        if self.contact_patch_length_m > 2*self.wheel_radius_m:
            raise ValueError('contact patch longer than wheel diameter')

    def wheel_centers(self):
        a, b = self.wheelbase_m/2, self.track_width_m/2
        return {'front_left': (a, b), 'front_right': (a, -b), 'rear_left': (-a, b), 'rear_right': (-a, -b)}


VALUE_STATUS = {
    'body_length_m': 'team design target 20 cm (registered regulation envelope)',
    'body_width_m': 'team design target 15 cm (registered regulation envelope)',
    'wheelbase_m': 'provisional fit to 200 mm length, real wheelbase unconfirmed',
    'track_width_m': 'provisional, unconfirmed',
    'wheel_radius_m': 'confirmed 65 mm outer diameter, loaded rolling radius unmeasured',
    'wheel_width_m': 'provisional 12 mm, tire width unconfirmed',
    'contact_patch_length_m': 'evaluation assumption, no measurement',
    'max_wheel_steering_rad': 'simulation joint limit; inspection steering state unconfirmed',
}


def ackermann_wheel_angles(central, geometry=VehicleGeometry()):
    """`arena_gazebo/drivetrain.hpp`의 SteeringAngles와 같은 식. (왼쪽, 오른쪽) 바퀴 조향각."""
    if not math.isfinite(central):
        raise ValueError('invalid steering')
    l, t, limit = geometry.wheelbase_m, geometry.track_width_m, geometry.max_wheel_steering_rad
    central_limit = math.atan(l/(l/math.tan(limit)+t/2))
    central = min(max(central, -central_limit), central_limit)
    curvature = math.tan(central)/l
    return (math.atan(l*curvature/(1-t*curvature/2)), math.atan(l*curvature/(1+t*curvature/2)))


def wheel_shapes(x, y, yaw, central_steering=0., geometry=VehicleGeometry()):
    """바퀴별 (접지 중심, 접지면, 위에서 본 바퀴 외곽). 앞바퀴는 Ackermann 조향각만큼 돌린다."""
    left, right = ackermann_wheel_angles(central_steering, geometry)
    steer = {'front_left': left, 'front_right': right, 'rear_left': 0., 'rear_right': 0.}
    c, s = math.cos(yaw), math.sin(yaw)
    result = {}
    for name, (lx, ly) in geometry.wheel_centers().items():
        cx, cy = x+lx*c-ly*s, y+lx*s+ly*c
        heading = yaw+steer[name]
        result[name] = {
            'center': (cx, cy), 'steering_rad': steer[name],
            'patch': rectangle(cx, cy, heading, geometry.contact_patch_length_m, geometry.wheel_width_m),
            'outline': rectangle(cx, cy, heading, 2*geometry.wheel_radius_m, geometry.wheel_width_m)}
    return result


def steering_footprint(x, y, yaw, central_steering=0., geometry=VehicleGeometry()):
    """차체 사각형과 조향한 바퀴 외곽의 합집합. 검차 외형이 아니라 노면·충돌 평가용이다."""
    shapes = wheel_shapes(x, y, yaw, central_steering, geometry)
    return unary_union([rectangle(x, y, yaw, geometry.body_length_m, geometry.body_width_m)]
                       + [w['outline'] for w in shapes.values()])


class SurfaceMap:
    """평가 전용 지면 분류. 제어기·보호층에 전달하지 않는다."""

    def __init__(self, road, grass, walls, source=None):
        self.road, self.grass, self.walls = road, grass, walls
        self._prepared = {'road': prep(road), 'grass': prep(grass), 'wall': prep(walls)}
        self.source = source or {}

    @classmethod
    def from_world(cls, world_path):
        world_path = Path(world_path)
        model = ET.parse(world_path).find("./world/model[@name='it_arena_track_static']")
        if model is None:
            raise ValueError('track model missing')
        pose = list(map(float, model.findtext('pose', '0 0 0 0 0 0').split()))
        if any(abs(v) > 1e-8 for v in pose):
            raise ValueError('surface evaluation requires a world-origin track model')

        def union(name):
            link = model.find(f"link[@name='{name}']")
            if link is None:
                raise ValueError(f'missing {name} link')
            lp = list(map(float, link.findtext('pose', '0 0 0 0 0 0').split()))
            boxes = []
            for collision in link.findall('collision'):
                cp = list(map(float, collision.findtext('pose', '0 0 0 0 0 0').split()))
                size = collision.findtext('geometry/box/size')
                if size is None or any(abs(v) > 1e-8 for v in (lp[3], lp[4], cp[3], cp[4])):
                    raise ValueError(f'unsupported {name} collision')
                sx, sy, _ = map(float, size.split())
                c, s = math.cos(lp[5]), math.sin(lp[5])
                boxes.append(rectangle(lp[0]+c*cp[0]-s*cp[1], lp[1]+s*cp[0]+c*cp[1], lp[5]+cp[5], sx, sy))
            if not boxes:
                raise ValueError(f'no {name} collisions')
            return unary_union(boxes)
        return cls(union('track_surface'), union('grass'), union('walls'),
                   {'world': str(world_path), 'world_sha256': hashlib.sha256(world_path.read_bytes()).hexdigest()})

    def road_contains(self, polygon):
        return self._prepared['road'].contains(polygon)

    def point_class(self, x, y):
        point = Point(x, y)
        for name in ('wall', 'road', 'grass'):
            if self._prepared[name].covers(point):   # 경계 위 점은 그 지면으로 본다
                return name
        return 'other'

    def fractions(self, polygon):
        """면적 비율. 겹친 지면은 우선순위로 한 번만 센다."""
        area = polygon.area
        if area <= 0:
            raise ValueError('empty polygon')
        if self._prepared['road'].contains(polygon) and not self._prepared['wall'].intersects(polygon):
            return {'wall': 0., 'road': 1., 'grass': 0., 'other': 0.}
        wall = polygon.intersection(self.walls)
        rest = polygon.difference(wall) if not wall.is_empty else polygon
        road = rest.intersection(self.road)
        rest = rest.difference(road) if not road.is_empty else rest
        grass = rest.intersection(self.grass)
        values = {'wall': wall.area/area, 'road': road.area/area, 'grass': grass.area/area}
        values['other'] = max(0., 1.-sum(values.values()))
        return values


def evaluate_pose(surface, x, y, yaw, central_steering=0., geometry=VehicleGeometry()):
    """한 자세의 기존 차체 지표와 바퀴별 지표."""
    body = rectangle(x, y, yaw, geometry.body_length_m, geometry.body_width_m)
    wheels = {}
    shapes = wheel_shapes(x, y, yaw, central_steering, geometry)
    for name, shape in shapes.items():
        fractions = surface.fractions(shape['patch'])
        center = surface.point_class(*shape['center'])
        off = {'any': fractions['road'] < 1.-FRACTION_EPS, 'center': center != 'road',
               'full': fractions['road'] <= FRACTION_EPS}
        outside = [k for k, v in fractions.items() if k != 'road' and v > FRACTION_EPS]
        wheels[name] = {'fractions': fractions, 'center_class': center, 'off': off,
                        'off_surface': outside[0] if len(outside) == 1 else ('mixed' if outside else None),
                        'steering_rad': shape['steering_rad']}
    footprint = unary_union([body]+[s['outline'] for s in shapes.values()])
    counts = {k: sum(w['off'][k] for w in wheels.values()) for k in INTERPRETATIONS}
    inside = surface.road_contains
    return {
        # 기존 보수 지표와 같은 식(SDF 도로 합집합 기준, 벽 우선순위 미적용). 완전 포함은 빠른 경로.
        'body_outside_road_area_m2': 0. if inside(body) else body.difference(surface.road).area,
        'steering_footprint_outside_road_area_m2': 0. if inside(footprint) else footprint.difference(surface.road).area,
        'steering_footprint_area_m2': footprint.area,
        'wheels': wheels, 'off_counts': counts,
        'grass_wheels': {k: sum(w['off'][k] and w['fractions']['grass'] > FRACTION_EPS for w in wheels.values())
                         for k in INTERPRETATIONS},
        'other_wheels': {k: sum(w['off'][k] and w['fractions']['other'] > FRACTION_EPS for w in wheels.values())
                         for k in INTERPRETATIONS},
        'wall_wheels': sum(w['fractions']['wall'] > FRACTION_EPS for w in wheels.values()),
    }


def _pair_kind(names):
    names = set(names)
    if len(names) != 2:
        return None
    if names in ({'front_left', 'rear_left'}, {'front_right', 'rear_right'}):
        return 'same_side'
    if names in ({'front_left', 'front_right'}, {'rear_left', 'rear_right'}):
        return 'same_axle'
    return 'diagonal'


def _runs(times, flags, weights):
    """참인 연속 구간 [(시작, 끝, 지속)]. 끝은 다음 표본 시각(마지막은 가중 시간)."""
    runs, start = [], None
    for i, flag in enumerate(flags):
        if flag and start is None:
            start = i
        if start is not None and (not flag or i == len(flags)-1):
            last = i if flag else i-1
            end = times[last]+weights[last]
            runs.append((times[start], end, sum(weights[start:last+1])))
            start = None
    return runs


def analyze_series(times, evaluations, *, merge_gap_s=.5, max_sample_gap_s=.05, penalty_wheels=3,
                   long_grass_s=(1., 3., 5.)):
    """시간열 요약. 표본 시간 가중치는 다음 표본까지(최대 max_sample_gap_s로 절단)."""
    if len(times) != len(evaluations) or not times:
        raise ValueError('times/evaluations mismatch')
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('nonincreasing sample times')
    gaps = [b-a for a, b in zip(times, times[1:])]
    weights = [min(g, max_sample_gap_s) for g in gaps]+[min(gaps[-1], max_sample_gap_s) if gaps else 0.]
    over_gap = sum(g > max_sample_gap_s for g in gaps)
    result = {
        'samples': len(times), 'first_s': times[0], 'last_s': times[-1],
        'max_sample_gap_s': max(gaps, default=0.), 'sample_gaps_over_limit': over_gap,
        'sample_gap_limit_s': max_sample_gap_s,
        'body_outside_samples_over_1e-6_m2': sum(e['body_outside_road_area_m2'] > 1e-6 for e in evaluations),
        'body_outside_samples_over_1cm2': sum(e['body_outside_road_area_m2'] > 1e-4 for e in evaluations),
        'max_body_outside_road_area_m2': max(e['body_outside_road_area_m2'] for e in evaluations),
        'steering_footprint_outside_samples_over_1e-6_m2': sum(
            e['steering_footprint_outside_road_area_m2'] > 1e-6 for e in evaluations),
        'max_steering_footprint_outside_road_area_m2': max(
            e['steering_footprint_outside_road_area_m2'] for e in evaluations),
        'wall_wheel_samples': sum(e['wall_wheels'] > 0 for e in evaluations),
        'merge_gap_s': merge_gap_s, 'penalty_wheels_threshold': penalty_wheels,
        'interpretations': {}}
    for kind in INTERPRETATIONS:
        counts = [e['off_counts'][kind] for e in evaluations]
        histogram = {str(n): sum(w for c, w in zip(counts, weights) if c == n) for n in range(5)}
        samples = {str(n): sum(c == n for c in counts) for n in range(5)}
        penalty = [c >= penalty_wheels for c in counts]
        raw = _runs(times, penalty, weights)
        merged = []
        for start, end, duration in raw:
            if merged and start-merged[-1]['end_s'] < merge_gap_s:
                merged[-1].update(end_s=end, duration_s=merged[-1]['duration_s']+duration, raw_edges=merged[-1]['raw_edges']+1)
            else:
                merged.append({'start_s': start, 'end_s': end, 'duration_s': duration, 'raw_edges': 1})
        for event in merged:
            inside = [e for t, e in zip(times, evaluations) if event['start_s'] <= t < event['end_s']]
            event['max_off_wheels'] = max((e['off_counts'][kind] for e in inside), default=None)
            event['off_surfaces'] = sorted({w['off_surface'] for e in inside for w in e['wheels'].values()
                                            if w['off'][kind] and w['off_surface']})
        any_off = [c >= 1 for c in counts]
        grass_any = [e['grass_wheels'][kind] >= 1 for e in evaluations]
        other_any = [e['other_wheels'][kind] >= 1 for e in evaluations]
        grass_runs = _runs(times, grass_any, weights)
        pairs = {}
        for e in evaluations:
            if e['off_counts'][kind] == 2:
                kind2 = _pair_kind([n for n, w in e['wheels'].items() if w['off'][kind]])
                pairs[kind2] = pairs.get(kind2, 0)+1
        wheel_toggles = {}
        for name in WHEELS:
            flags = [e['wheels'][name]['off'][kind] for e in evaluations]
            wheel_toggles[name] = sum(a != b for a, b in zip(flags, flags[1:]))
        result['interpretations'][kind] = {
            'time_by_off_wheel_count_s': histogram, 'samples_by_off_wheel_count': samples,
            'max_off_wheels': max(counts),
            'penalty_state_raw_rising_edges': len(raw),
            'penalty_state_transitions': sum(a != b for a, b in zip(penalty, penalty[1:])),
            'penalty_events_merged': len(merged), 'penalty_events': merged[:50],
            'penalty_state_time_s': sum(d for _, _, d in raw),
            'any_off_time_s': sum(w for f, w in zip(any_off, weights) if f),
            'grass_contact_time_s': sum(w for f, w in zip(grass_any, weights) if f),
            'other_ground_contact_time_s': sum(w for f, w in zip(other_any, weights) if f),
            'longest_continuous_grass_contact_s': max((d for _, _, d in grass_runs), default=0.),
            'grass_contact_runs_over_s': {str(limit): sum(d > limit for _, _, d in grass_runs) for limit in long_grass_s},
            'two_wheel_pair_kinds_samples': pairs, 'wheel_off_toggles': wheel_toggles}
    return result


# ------------------------------------------------------------------ 저장 기록 입력
def load_poses(run):
    """정답 자세 [t, x, y, yaw]. 촘촘한 ego_poses.jsonl을 우선하고 없으면 trajectory.jsonl을 쓴다."""
    run = Path(run)
    dense = run/'ego_poses.jsonl'
    if dense.exists():
        rows = [json.loads(line) for line in dense.read_text(encoding='utf-8').splitlines() if line.strip()]
        return [tuple(map(float, r[:4])) for r in rows], 'ego_poses.jsonl'
    rows = [json.loads(line) for line in (run/'trajectory.jsonl').read_text(encoding='utf-8').splitlines() if line.strip()]
    # 이전 기록은 시각 필드가 'time'이다.
    return [(float(r['t'] if 't' in r else r['time']), float(r['x']), float(r['y']), float(r['yaw']))
            for r in rows], 'trajectory.jsonl'


def load_steering(run):
    """MCU 목표 중앙 조향(now_us 시각). 실제 관절각이 아니다(서보 지연·변화율 제한 미반영)."""
    path = Path(run)/'actuation_trace.json'
    if not path.exists():
        return None
    rows = json.loads(path.read_text(encoding='utf-8'))
    series = sorted((r['now_us']*1e-6, float(r.get('steering_rad') or 0.)) for r in rows
                    if isinstance(r.get('now_us'), (int, float)) and r.get('steering_rad') is not None)
    return series or None


def deduplicate(poses):
    result = []
    for pose in poses:
        if result and pose[0] <= result[-1][0]:
            continue
        result.append(pose)
    return result


def evaluate_run(run, surface=None, geometry=VehicleGeometry(), *, merge_gap_s=.5, start_s=None, end_s=None,
                 steering='actuation'):
    run = Path(run)
    surface = surface or SurfaceMap.from_world(run/'runtime_world.sdf')
    poses, source = load_poses(run)
    poses = deduplicate([p for p in poses if (start_s is None or p[0] >= start_s) and (end_s is None or p[0] <= end_s)])
    series = load_steering(run) if steering == 'actuation' else None
    index = [s[0] for s in series] if series is not None else None
    evaluations = []
    for t, x, y, yaw in poses:
        steer = 0.
        if series is not None:
            i = bisect_right(index, t)-1
            steer = series[i][1] if i >= 0 else 0.
        evaluations.append(evaluate_pose(surface, x, y, yaw, steer, geometry))
    summary = analyze_series([p[0] for p in poses], evaluations, merge_gap_s=merge_gap_s)
    return {
        'scope': 'post-run truth-only surface evaluation; not a controller input, official judgement or retroactive pass',
        'evaluator_version': EVALUATOR_VERSION, 'pose_source': source,
        'steering_source': 'MCU target central steering (actuation_trace)' if series is not None else 'zero (no steering record)',
        'geometry': asdict(geometry), 'geometry_status': VALUE_STATUS, **surface.source, **summary}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='새 결과 파일. 원본 실행 폴더에는 쓰지 않는다.')
    parser.add_argument('--merge-gap', type=float, default=.5)
    parser.add_argument('--patch-length', type=float, default=VehicleGeometry.contact_patch_length_m)
    parser.add_argument('--steering', choices=('actuation', 'zero'), default='actuation')
    args = parser.parse_args()
    if args.output.resolve().is_relative_to(args.run.resolve()):
        raise SystemExit('refusing to write inside the stored run directory')
    geometry = VehicleGeometry(contact_patch_length_m=args.patch_length)
    result = evaluate_run(args.run, geometry=geometry, merge_gap_s=args.merge_gap, steering=args.steering)
    result['run'] = str(args.run)
    result['script_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=1, ensure_ascii=False), encoding='utf-8', newline='\n')
    brief = {k: result[k] for k in ('samples', 'body_outside_samples_over_1e-6_m2',
                                    'steering_footprint_outside_samples_over_1e-6_m2', 'wall_wheel_samples')}
    brief['max_off_wheels'] = {k: v['max_off_wheels'] for k, v in result['interpretations'].items()}
    brief['penalty_events_merged'] = {k: v['penalty_events_merged'] for k, v in result['interpretations'].items()}
    print(json.dumps(brief))


if __name__ == '__main__':
    main()
