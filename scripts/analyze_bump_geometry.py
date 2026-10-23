#!/usr/bin/env python3
"""방지턱의 2D 준정적 기하 비교. 접촉 힘·충격·안전 속도·주행 성공 판정은 하지 않는다."""
import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Geometry:
    length_m: float
    height_m: float = .010
    radius_m: float = .0325
    wheelbase_m: float = .135
    body_length_m: float = .200

    def validate(self):
        values = asdict(self)
        if any(type(v) not in (float, int) or not math.isfinite(v) for v in values.values()):
            raise ValueError('finite numeric dimensions required')
        if not (.02 <= self.length_m <= .5 and 0 <= self.height_m < self.radius_m
                and .01 <= self.radius_m <= .1 and .05 <= self.wheelbase_m <= .5
                and self.wheelbase_m <= self.body_length_m <= .6):
            raise ValueError('outside bounded geometry study')
        # 표본 원호 envelope도 평지 지지 R보다 높은 곳에서 아래 기울기 한계를 갖는다.
        # 이 조건은 rear x 고정 front axle 거리식의 유일성을 보장하는 충분 조건이다.
        slope_bound = math.sqrt(self.radius_m**2-(self.radius_m-self.height_m)**2)/(self.radius_m-self.height_m)
        if self.wheelbase_m**2 <= self.height_m**2*(1+slope_bound**2):
            raise ValueError('unique wheelbase root not established')


def profile(x, geometry):
    x = np.asarray(x, dtype=float)
    h = .5*geometry.height_m*(1-np.cos(2*np.pi*x/geometry.length_m))
    return np.where((x >= 0) & (x <= geometry.length_m), h, 0.)


class WheelEnvelope:
    """강체 원형 바퀴가 지형과 겹치지 않는 최소 중심 높이의 표본 하한/구간 상한."""
    def __init__(self, geometry, step_m):
        geometry.validate()
        if not math.isfinite(step_m) or not 1e-5 <= step_m <= .002:
            raise ValueError('surface step must be 0.01..2 mm')
        self.geometry = geometry
        # 짝수 분할로 꼭대기/양 끝을 항상 포함한다.
        n = int(math.ceil(geometry.length_m/step_m/2))*2
        self.q = np.linspace(0., geometry.length_m, n+1)
        self.h = profile(self.q, geometry)
        self.step_m = geometry.length_m/n
        self.hi = np.maximum(self.h[:-1], self.h[1:])

    def heights(self, x, *, upper=False):
        x = np.atleast_1d(np.asarray(x, dtype=float))
        if x.ndim != 1 or not np.isfinite(x).all():
            raise ValueError('finite 1D positions required')
        result = np.empty(len(x))
        r = self.geometry.radius_m
        for start in range(0, len(x), 128):
            row = x[start:start+128, None]
            if upper:
                # 각 지형 구간의 정확한 최고 h와 원 중심에 가장 가까운 q를 조합한다.
                # 두 항의 최대 위치가 다를 수 있으므로 값은 느슨한 상한이다.
                delta = np.maximum(np.maximum(self.q[:-1]-row, row-self.q[1:]), 0.)
                heights = self.hi
            else:
                delta = np.abs(self.q-row)
                heights = self.h
            cap = heights+np.sqrt(np.maximum(0., r*r-delta*delta))
            cap = np.where(delta <= r, cap, -np.inf)
            result[start:start+128] = np.maximum(r, cap.max(axis=1))
        return result


def axle_pose(envelope, rear_x):
    """수평 간격을 축거로 고정하지 않고 실제 두 축 중심 거리 W를 푼다."""
    g = envelope.geometry
    x = np.asarray(rear_x, dtype=float)
    rear_z = envelope.heights(x)
    lo = x+math.sqrt(g.wheelbase_m**2-g.height_m**2)
    hi = x+g.wheelbase_m
    for _ in range(38):
        mid = (lo+hi)/2
        z = envelope.heights(mid)
        residual = (mid-x)**2+(z-rear_z)**2-g.wheelbase_m**2
        lo, hi = np.where(residual <= 0, mid, lo), np.where(residual > 0, mid, hi)
    front_x = (lo+hi)/2
    front_z = envelope.heights(front_x)
    pitch = np.arctan2(front_z-rear_z, front_x-x)
    return rear_z, front_x, front_z, pitch


def clearance_at_pose(envelope, rear_x, rear_z, pitch):
    """평지 지상고 g가 필요량 이상이어야 하부 선분이 지형 위에 놓이는 2D 조건.

    반환 bound는 이 계산 자세에서 지형 표본 사이의 누락 오차만 포함한다.
    바퀴 envelope/자세/주행 위치 표본 오차·타이어 변형은 별도다.
    """
    g = envelope.geometry
    co, si = math.cos(pitch), math.sin(pitch)
    slope = math.pi*g.height_m/g.length_m
    if co-abs(si)*slope <= 0:
        raise ValueError('terrain projection is not monotone')
    overhang = (g.body_length_m-g.wheelbase_m)/2
    u0, u1 = -overhang, g.wheelbase_m+overhang
    q, h = envelope.q, envelope.h
    u = co*(q-rear_x)+si*(h-rear_z)
    # 무한 평지의 가상 연장은 실제 융기보다 낮다. 양 끝을 포함해도 상향 오판하지 않는다.
    values = [(-rear_z-u0*si)/co, (-rear_z-u1*si)/co]
    keep = (u >= u0) & (u <= u1)
    if keep.any():
        values.append(float((-si*(q[keep]-rear_x)+co*(h[keep]-rear_z)).max()))
    # 하부 선분 끝과 곡면의 투영 교차를 포함해 경계 사이 짧은 구간을 빠뜨리지 않는다.
    for target in (u0, u1):
        if u[0] <= target <= u[-1]:
            lo, hi = 0., g.length_m
            for _ in range(38):
                mid = (lo+hi)/2
                hm = float(profile(mid, g))
                if co*(mid-rear_x)+si*(hm-rear_z) < target:
                    lo = mid
                else:
                    hi = mid
            qt = (lo+hi)/2
            values.append(-si*(qt-rear_x)+co*(float(profile(qt, g))-rear_z))
    required = g.radius_m+max(values)
    sampling_bound = (abs(si)+co*slope)*envelope.step_m/2
    return max(0., required), sampling_bound


def forward_floor_ray(g, rear_x, rear_z, pitch, lidar_x=.060, lidar_z=.095):
    """주어진 자세에서 전방 수평 장착 광선과 z=0 평면의 기하 교점.

    양의 pitch는 이 2D 계산에서 앞이 들리는 방향이다(SDF Euler 부호와 구분).
    빔 가림/반사/범위/샘플링을 판정하지 않는 가상 광선 하나다.
    """
    values = [rear_x, rear_z, pitch, lidar_x, lidar_z]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('finite pose/extrinsics required')
    co, si = math.cos(pitch), math.sin(pitch)
    along = g.wheelbase_m/2+lidar_x
    above = lidar_z-g.radius_m
    sx = rear_x+along*co-above*si
    sz = rear_z+along*si+above*co
    if sz <= 0 or co <= 0:
        raise ValueError('sensor must be above ground and face forward')
    distance = -sz/si if si < -1e-12 else None
    hit_x = sx+distance*co if distance is not None else None
    return dict(sensor_x_m=float(sx), sensor_z_m=float(sz), pitch_nose_up_rad=float(pitch),
                forward_flat_floor_slant_distance_m=float(distance) if distance is not None else None,
                floor_x_m=float(hit_x) if hit_x is not None else None,
                floor_hit_downstream_of_bump=bool(hit_x is not None and hit_x > g.length_m),
                beam_visibility_verified=False, lidar_return_verified=False)


def sweep(g, surface_step_m=.0002, travel_step_m=.001):
    if not math.isfinite(travel_step_m) or not .0001 <= travel_step_m <= .005:
        raise ValueError('travel step must be 0.1..5 mm')
    envelope = WheelEnvelope(g, surface_step_m)
    x0 = -g.wheelbase_m-g.radius_m-g.body_length_m
    x1 = g.length_m+g.radius_m+g.body_length_m
    x = np.linspace(x0, x1, int(math.ceil((x1-x0)/travel_step_m))+1)
    rear_z, front_x, front_z, pitch = axle_pose(envelope, x)
    clearance = np.array([clearance_at_pose(envelope, a, b, c) for a, b, c in zip(x, rear_z, pitch)])
    lower = np.concatenate([rear_z, front_z])
    upper = envelope.heights(np.concatenate([x, front_x]), upper=True)
    i = int(clearance[:, 0].argmax())
    down = int(pitch.argmin())
    return dict(parameters=asdict(g), surface_step_m=envelope.step_m,
                travel_step_m=float(x[1]-x[0]), sampled_poses=len(x),
                peak_abs_pitch_deg=float(np.rad2deg(np.abs(pitch)).max()),
                sampled_required_clearance_m=float(clearance[i, 0]),
                peak_clearance_rear_x_m=float(x[i]),
                terrain_sampling_bound_at_fixed_pose_m=float(clearance[:, 1].max()),
                wheel_center_height_bracket_max_m=float((upper-lower).max()),
                axle_distance_residual_max_m=float(np.abs(np.hypot(front_x-x, front_z-rear_z)-g.wheelbase_m).max()),
                road_peak_slope_deg=math.degrees(math.atan(math.pi*g.height_m/g.length_m)),
                maximum_nose_down_floor_witness=forward_floor_ray(g, x[down], rear_z[down], pitch[down]),
                continuous_travel_certified=False, dynamic_contact_verified=False,
                vehicle_pass_verified=False, safe_speed_mps=None)


def study():
    rows = []
    for length in (.05, .10, .15, .20):
        g = Geometry(length)
        coarse = sweep(g)
        fine = sweep(g, .0001, .0005)
        rows.append(dict(length_status='published_file' if length == .05 else
                         ('organizer_proposal_not_adopted' if length in (.10, .15) else 'old_team_experiment'),
                         coarse=coarse, fine=fine,
                         clearance_refinement_delta_m=abs(coarse['sampled_required_clearance_m']-fine['sampled_required_clearance_m']),
                         pitch_refinement_delta_deg=abs(coarse['peak_abs_pitch_deg']-fine['peak_abs_pitch_deg'])))
    return dict(schema='arena_bump_geometry_v1', model='rigid_circles_quasistatic_2d_no_suspension',
                wheel_diameter_status='team_selected_65mm_effective_loaded_radius_unmeasured',
                wheelbase_status='provisional_135mm', actual_ground_clearance_m=None,
                current_model_box_flat_clearance_m=.0325,
                current_runtime_changed=False, cases=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output exists; preserve previous evidence')
    result = study()
    result['script_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    # 직렬화를 먼저 마쳐 형식 오류로 부분 report를 생성하지 않는다.
    # 디스크 쓰기/전원 장애의 원자성을 보장하는 장치는 아니다.
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    with args.output.open('x', encoding='utf-8') as target:
        target.write(text)
    print(json.dumps({k: v for k, v in result.items() if k != 'cases'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
